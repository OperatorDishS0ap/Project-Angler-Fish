import json
import socket
import struct
import time
from collections import OrderedDict
import pigpio

# -------------------------
# NETWORK
# -------------------------
LISTEN_IP = "0.0.0.0"
LISTEN_PORT = 9000
SOCK_TIMEOUT_S = 0.2

# -------------------------
# GPIO MAP
# -------------------------
GPIO_M1 = 13
GPIO_M2 = 19
GPIO_M3 = 18
GPIO_M4 = 12
ALL_GPIOS = (GPIO_M1, GPIO_M2, GPIO_M3, GPIO_M4)

# -------------------------
# ESC / DSHOT SETTINGS
# -------------------------
# DShot frame timing nominal for DShot600 is 1.67us/bit, but pigpio wave
# timing is integer microseconds. We use 2us bits (closest practical timing
# on pigpio), which many ESCs still accept as DShot-class digital signaling.
DSHOT_BIT_TOTAL_US = 2
DSHOT_T0H_US = 1
DSHOT_T1H_US = 2
DSHOT_FRAME_GAP_US = 40

# DShot throttle code ranges
DSHOT_STOP = 0
DSHOT_MIN = 48
DSHOT_MAX = 2047
DSHOT_3D_REVERSE_MAX = 1047
DSHOT_3D_FORWARD_MIN = 1048

# When True, map negative commands to reverse range and positive commands to
# forward range (ESC firmware must be configured for reversible/3D DShot).
DSHOT_3D_MODE = True

# Cache compiled waveforms for repeated command tuples.
MAX_WAVE_CACHE = 256

ARM_TIME_S = 3.0
COMMAND_TIMEOUT_S = 0.5
LOOP_SLEEP_S = 0.005

# Small command deadband: treat tiny commands as neutral
PCT_DEADBAND = 2.0  # percent

# -------------------------
# LEGACY BINARY PROTOCOL
# -------------------------
CMD_FMT = "<4sI4h"
CMD_MAGIC = b"SUB1"
CMD_SIZE = struct.calcsize(CMD_FMT)


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def i16_to_pct(v_i16: int) -> float:
    # -1000..+1000 => -100..+100
    return clamp((float(v_i16) / 1000.0) * 100.0, -100.0, 100.0)


def pct_to_dshot_value(pct: float) -> int:
    """Map -100..+100 command to DShot throttle code."""
    pct = clamp(pct, -100.0, 100.0)

    if abs(pct) <= PCT_DEADBAND:
        return DSHOT_STOP

    if DSHOT_3D_MODE:
        if pct < 0:
            mag = abs(pct) / 100.0
            return int(DSHOT_MIN + mag * (DSHOT_3D_REVERSE_MAX - DSHOT_MIN))
        return int(DSHOT_3D_FORWARD_MIN + (pct / 100.0) * (DSHOT_MAX - DSHOT_3D_FORWARD_MIN))

    if pct < 0:
        return DSHOT_STOP

    return int(DSHOT_MIN + (pct / 100.0) * (DSHOT_MAX - DSHOT_MIN))


def dshot_make_frame(value: int, telemetry: bool = False) -> int:
    """Build 16-bit DShot frame with CRC nibble."""
    value = int(clamp(value, 0, DSHOT_MAX))
    packet = (value << 1) | (1 if telemetry else 0)
    crc = (packet ^ (packet >> 4) ^ (packet >> 8)) & 0x0F
    return (packet << 4) | crc


def setup_dshot_gpio(pi: pigpio.pi, gpio: int):
    pi.set_mode(gpio, pigpio.OUTPUT)
    pi.set_servo_pulsewidth(gpio, 0)
    pi.set_PWM_dutycycle(gpio, 0)
    pi.write(gpio, 0)


class DShotSender:
    def __init__(self, pi: pigpio.pi, gpios):
        self.pi = pi
        self.gpios = tuple(gpios)
        self.all_mask = 0
        for g in self.gpios:
            self.all_mask |= (1 << g)
        self.wave_cache = OrderedDict()

    def _build_wave(self, frames):
        pulses = []
        t0 = DSHOT_T0H_US
        t1 = DSHOT_T1H_US - DSHOT_T0H_US
        t2 = DSHOT_BIT_TOTAL_US - DSHOT_T1H_US

        if t0 <= 0 or t1 < 0 or t2 < 0:
            raise ValueError("Invalid DShot timing constants")

        for bit in range(15, -1, -1):
            mask_ones = 0
            for gpio, frame in zip(self.gpios, frames):
                if (frame >> bit) & 0x1:
                    mask_ones |= (1 << gpio)
            mask_zeros = self.all_mask & (~mask_ones)

            pulses.append(pigpio.pulse(self.all_mask, 0, t0))
            if t1 > 0:
                pulses.append(pigpio.pulse(0, mask_zeros, t1))
            if t2 > 0:
                pulses.append(pigpio.pulse(0, mask_ones, t2))

        pulses.append(pigpio.pulse(0, 0, DSHOT_FRAME_GAP_US))

        self.pi.wave_add_generic(pulses)
        return self.pi.wave_create()

    def send(self, values):
        frames = tuple(dshot_make_frame(v) for v in values)
        wave_id = self.wave_cache.get(frames)
        if wave_id is None:
            wave_id = self._build_wave(frames)
            if wave_id < 0:
                return
            self.wave_cache[frames] = wave_id
            if len(self.wave_cache) > MAX_WAVE_CACHE:
                old_frames, old_wave = self.wave_cache.popitem(last=False)
                if old_frames != frames:
                    self.pi.wave_delete(old_wave)
        else:
            self.wave_cache.move_to_end(frames)

        while self.pi.wave_tx_busy():
            time.sleep(0.00005)
        self.pi.wave_send_once(wave_id)

    def shutdown(self):
        for _ in range(30):
            self.send((DSHOT_STOP, DSHOT_STOP, DSHOT_STOP, DSHOT_STOP))
            time.sleep(0.005)
        self.pi.wave_tx_stop()
        for _frames, wave_id in self.wave_cache.items():
            self.pi.wave_delete(wave_id)
        self.wave_cache.clear()


def main():
    pi = pigpio.pi()
    if not pi.connected:
        raise SystemExit("pigpio daemon not running. Start with: sudo systemctl start pigpiod")

    # Setup all DShot outputs
    for g in ALL_GPIOS:
        setup_dshot_gpio(pi, g)

    sender = DShotSender(pi, ALL_GPIOS)

    print(f"[sub_motors_dshot600] Arming with DShot STOP for {ARM_TIME_S:.1f}s ...")
    arm_end = time.time() + ARM_TIME_S
    while time.time() < arm_end:
        sender.send((DSHOT_STOP, DSHOT_STOP, DSHOT_STOP, DSHOT_STOP))
        time.sleep(LOOP_SLEEP_S)

    # UDP socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((LISTEN_IP, LISTEN_PORT))
    sock.settimeout(SOCK_TIMEOUT_S)

    last_rx = time.time()
    last = {"m1": 0.0, "m2": 0.0, "m3": 0.0, "m4": 0.0}

    print(f"[sub_motors_dshot600] Listening UDP on {LISTEN_IP}:{LISTEN_PORT}")

    try:
        while True:
            # Receive
            try:
                data, _addr = sock.recvfrom(2048)

                # 1) Legacy binary
                if len(data) >= CMD_SIZE:
                    magic, _seq, m1_i16, m2_i16, m3_i16, m4_i16 = struct.unpack(CMD_FMT, data[:CMD_SIZE])
                    if magic == CMD_MAGIC:
                        last = {
                            "m1": i16_to_pct(m1_i16),
                            "m2": i16_to_pct(m2_i16),
                            "m3": i16_to_pct(m3_i16),
                            "m4": i16_to_pct(m4_i16),
                        }
                        last_rx = time.time()
                    else:
                        raise ValueError("Not legacy magic")
                else:
                    # JSON fallback
                    msg = json.loads(data.decode("utf-8", errors="ignore"))
                    last = {
                        "m1": float(msg.get("m1", last["m1"])),
                        "m2": float(msg.get("m2", last["m2"])),
                        "m3": float(msg.get("m3", last["m3"])),
                        "m4": float(msg.get("m4", last["m4"])),
                    }
                    last_rx = time.time()

            except socket.timeout:
                pass
            except Exception:
                pass

            # Failsafe -> neutral
            if time.time() - last_rx > COMMAND_TIMEOUT_S:
                last = {"m1": 0.0, "m2": 0.0, "m3": 0.0, "m4": 0.0}

            # Apply outputs (DShot digital throttle)
            sender.send(
                (
                    pct_to_dshot_value(last["m1"]),
                    pct_to_dshot_value(last["m2"]),
                    pct_to_dshot_value(last["m3"]),
                    pct_to_dshot_value(last["m4"]),
                )
            )

            time.sleep(LOOP_SLEEP_S)

    finally:
        sender.shutdown()
        pi.stop()


if __name__ == "__main__":
    main()
