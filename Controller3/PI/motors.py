import json
import socket
import struct
import time
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
GPIO_M1 = 19
GPIO_M2 = 13
GPIO_M3 = 18
GPIO_M4 = 12
ALL_GPIOS = (GPIO_M1, GPIO_M2, GPIO_M3, GPIO_M4)

# -------------------------
# ESC / PWM SETTINGS
# -------------------------
ESC_FREQ_HZ = 400
PERIOD_US = int(1_000_000 / ESC_FREQ_HZ)  # 2500us @ 400Hz
PWM_RANGE = PERIOD_US                      # range=2500 => dutycycle "counts" == microseconds

PULSE_MIN = 1350
PULSE_MAX = 1750

PULSE_NEUTRAL = 1460

# "Creep zone" to avoid sending (neutral, 1600]
AVOID_LO = 1406
AVOID_HI = 1514
FORWARD_START = AVOID_HI + 1  # 1601

ARM_TIME_S = 3.0
LOOP_SLEEP_S = 0.005

# Small command deadband: treat tiny commands as neutral
PCT_DEADBAND = 2.0  # percent

# -------------------------
# LEGACY BINARY PROTOCOL
# -------------------------
CMD_FMT = "<4sI5h"
CMD_FMT_OLD = "<4sI4h"
CMD_MAGIC = b"SUB1"
CMD_SIZE = struct.calcsize(CMD_FMT)
CMD_SIZE_OLD = struct.calcsize(CMD_FMT_OLD)


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def connect_pigpio_with_retry(retries: int = 5, delay_s: float = 0.5) -> pigpio.pi:
    last_pi = None
    for _ in range(retries):
        pi = pigpio.pi()
        last_pi = pi
        if pi.connected:
            return pi
        try:
            pi.stop()
        except Exception:
            pass
        time.sleep(delay_s)
    if last_pi is not None:
        try:
            last_pi.stop()
        except Exception:
            pass
    raise SystemExit("pigpio daemon not running or unstable. Start with: sudo systemctl restart pigpiod")


def pigpio_call(pi: pigpio.pi, method_name: str, *args):
    method = getattr(pi, method_name)
    try:
        return pi, method(*args)
    except (BrokenPipeError, OSError):
        try:
            pi.stop()
        except Exception:
            pass
        time.sleep(0.2)
        pi = connect_pigpio_with_retry(retries=5, delay_s=0.3)
        method = getattr(pi, method_name)
        return pi, method(*args)


def i16_to_pct(v_i16: int) -> float:
    # -1000..+1000 => -100..+100
    return clamp((float(v_i16) / 1000.0) * 100.0, -100.0, 100.0)


def pct_to_pulse_us(pct: float, pulse_min_us: int, pulse_max_us: int) -> int:
    pct = clamp(pct, -100.0, 100.0)

    if abs(pct) <= PCT_DEADBAND:
        return PULSE_NEUTRAL

    pulse_min_us = int(clamp(pulse_min_us, 1000, AVOID_LO))
    pulse_max_us = int(clamp(pulse_max_us, AVOID_HI, 2000))

    if pct < 0:
        # Reverse: -100 => 800, 0 => 1500
        # Linear map: pulse = 1500 + (1500-800)*(pct/100)
        return int(PULSE_NEUTRAL + (PULSE_NEUTRAL - pulse_min_us) * (pct / 100.0))

    # Forward: +0 => 1601, +100 => 2100
    # Linear map: pulse = 1601 + (2100-1601)*(pct/100)
    return int(FORWARD_START + (pulse_max_us - FORWARD_START) * (pct / 100.0))


def setup_pwm_esc(pi: pigpio.pi, gpio: int):
    pi, _ = pigpio_call(pi, "set_mode", gpio, pigpio.OUTPUT)

    # Ensure servo mode is off (servo mode is ~50Hz)
    pi, _ = pigpio_call(pi, "set_servo_pulsewidth", gpio, 0)

    # Configure PWM
    pi, _ = pigpio_call(pi, "set_PWM_frequency", gpio, ESC_FREQ_HZ)
    pi, _ = pigpio_call(pi, "set_PWM_range", gpio, PWM_RANGE)

    # Neutral output to arm
    pi, _ = pigpio_call(pi, "set_PWM_dutycycle", gpio, PULSE_NEUTRAL)
    return pi


def set_pulse_us(pi: pigpio.pi, gpio: int, pulse_us: int, pulse_min_us: int, pulse_max_us: int):
    pulse_us = int(pulse_us)

    # Avoid creep zone (1501..1600)
    if AVOID_LO < pulse_us <= AVOID_HI:
        pulse_us = PULSE_NEUTRAL

    pulse_min_us = int(clamp(pulse_min_us, 1000, AVOID_LO))
    pulse_max_us = int(clamp(pulse_max_us, AVOID_HI, 2000))
    pulse_us = clamp(pulse_us, pulse_min_us, pulse_max_us)
    pi, _ = pigpio_call(pi, "set_PWM_dutycycle", gpio, pulse_us)
    return pi


def main():
    pi = connect_pigpio_with_retry()

    # Setup all ESC outputs
    for g in ALL_GPIOS:
        pi = setup_pwm_esc(pi, g)

    # Debug: confirm PWM config (PWM mode; do NOT call get_servo_pulsewidth)
    for g in ALL_GPIOS:
        pi, pwm_freq = pigpio_call(pi, "get_PWM_frequency", g)
        pi, pwm_range = pigpio_call(pi, "get_PWM_range", g)
        pi, duty = pigpio_call(pi, "get_PWM_dutycycle", g)
        print(
            f"GPIO {g}: pwm_freq={pwm_freq}Hz "
            f"pwm_range={pwm_range} duty={duty}"
        )

    print(f"[sub_motors_400hz] Arming at neutral ({PULSE_NEUTRAL}us) for {ARM_TIME_S:.1f}s ...")
    time.sleep(ARM_TIME_S)

    # UDP socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((LISTEN_IP, LISTEN_PORT))
    sock.settimeout(SOCK_TIMEOUT_S)

    last = {"m1": 0.0, "m2": 0.0, "m3": 0.0, "m4": 0.0}
    arm_requested = False
    arm_active = False
    arm_started_at = None
    pulse_min_us = PULSE_MIN
    pulse_max_us = PULSE_MAX

    print(f"[sub_motors_400hz] Listening UDP on {LISTEN_IP}:{LISTEN_PORT}")

    try:
        while True:
            # Receive
            try:
                data, _addr = sock.recvfrom(2048)

                # 1) Legacy binary
                if len(data) >= CMD_SIZE_OLD and data[:4] == CMD_MAGIC:
                    if len(data) >= CMD_SIZE:
                        magic, _seq, m1_i16, m2_i16, m3_i16, m4_i16, arm_i16 = struct.unpack(CMD_FMT, data[:CMD_SIZE])
                    else:
                        magic, _seq, m1_i16, m2_i16, m3_i16, m4_i16 = struct.unpack(CMD_FMT_OLD, data[:CMD_SIZE_OLD])
                        arm_i16 = 1000  # Backward-compatible: old packets are treated as armed.

                    if magic != CMD_MAGIC:
                        raise ValueError("Not legacy magic")

                    last = {
                        "m1": i16_to_pct(m1_i16),
                        "m2": i16_to_pct(m2_i16),
                        "m3": i16_to_pct(m3_i16),
                        "m4": i16_to_pct(m4_i16),
                    }
                    arm_requested = arm_i16 > 0

                else:
                    # JSON fallback
                    msg = json.loads(data.decode("utf-8", errors="ignore"))

                    if msg.get("type") == "tune":
                        if "pulse_min_us" in msg:
                            pulse_min_us = int(clamp(float(msg["pulse_min_us"]), 1000, AVOID_LO))
                        if "pulse_max_us" in msg:
                            pulse_max_us = int(clamp(float(msg["pulse_max_us"]), AVOID_HI, 2000))

                        if pulse_min_us >= pulse_max_us:
                            pulse_min_us = min(pulse_min_us, AVOID_LO)
                            pulse_max_us = max(pulse_max_us, AVOID_HI)
                        print(f"[sub_motors_400hz] Tune update: PULSE_MIN={pulse_min_us} PULSE_MAX={pulse_max_us}")
                    else:
                        last = {
                            "m1": float(msg.get("m1", last["m1"])),
                            "m2": float(msg.get("m2", last["m2"])),
                            "m3": float(msg.get("m3", last["m3"])),
                            "m4": float(msg.get("m4", last["m4"])),
                        }
                        arm_requested = bool(msg.get("arm", arm_requested))


            except socket.timeout:
                pass
            except Exception:
                pass

            # Arm/disarm state machine for ESC safety.
            if not arm_requested:
                if arm_active:
                    print("[sub_motors_400hz] DISARM command received; forcing neutral.")
                arm_active = False
                arm_started_at = None
                last = {"m1": 0.0, "m2": 0.0, "m3": 0.0, "m4": 0.0}
            else:
                if arm_started_at is None:
                    arm_started_at = time.time()
                    arm_active = False
                    print(f"[sub_motors_400hz] ARM command received; holding neutral for {ARM_TIME_S:.1f}s.")
                elif not arm_active and (time.time() - arm_started_at) >= ARM_TIME_S:
                    arm_active = True
                    print("[sub_motors_400hz] ESC output armed.")

            # Apply outputs (PWM @ 400Hz)
            if arm_active:
                pi = set_pulse_us(pi, GPIO_M1, pct_to_pulse_us(last["m1"], pulse_min_us, pulse_max_us), pulse_min_us, pulse_max_us)
                pi = set_pulse_us(pi, GPIO_M2, pct_to_pulse_us(last["m2"], pulse_min_us, pulse_max_us), pulse_min_us, pulse_max_us)
                pi = set_pulse_us(pi, GPIO_M3, pct_to_pulse_us(last["m3"], pulse_min_us, pulse_max_us), pulse_min_us, pulse_max_us)
                pi = set_pulse_us(pi, GPIO_M4, pct_to_pulse_us(last["m4"], pulse_min_us, pulse_max_us), pulse_min_us, pulse_max_us)
            else:
                for g in ALL_GPIOS:
                    pi = set_pulse_us(pi, g, PULSE_NEUTRAL, pulse_min_us, pulse_max_us)

            time.sleep(LOOP_SLEEP_S)

    finally:
        # Neutral on exit
        for g in ALL_GPIOS:
            pi = set_pulse_us(pi, g, PULSE_NEUTRAL, pulse_min_us, pulse_max_us)
        time.sleep(0.5)
        pi.stop()


if __name__ == "__main__":
    main()
