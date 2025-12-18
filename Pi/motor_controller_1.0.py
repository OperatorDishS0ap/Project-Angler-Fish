
"""motor_controller.py (Raspberry Pi)

- Drives 4 bidirectional ESCs via 50 Hz PWM (RPi.GPIO).
- Runs a UDP "network server" that listens for motor commands from the PC.
- Includes a watchdog failsafe: if packets stop, motors return to NEUTRAL.

Packet format (binary, little-endian):
  struct '<4s I 4h'
    magic: b'SUB1'
    seq:   uint32
    m1..m4 int16 each in [-1000, 1000] representing throttle [-1.0, +1.0]

Telemetry back to PC (optional, binary):
  struct '<4s I H 4h B'
    magic: b'STAT'
    seq:   uint32
    age_ms uint16 (age of last command when sent)
    m1..m4 int16 (last applied)
    armed  uint8 (1 if receiving commands recently, else 0)
"""

import time
import socket
import struct

import RPi.GPIO as GPIO

# ==========================
# ESC / GPIO CONFIG
# ==========================
ESC_PINS = [17, 18, 27, 22]  # BCM numbering (M1..M4)
PWM_FREQ = 50               # 50 Hz -> 20 ms period

NEUTRAL_US = 1500           # 1.5 ms = stopped/neutral
RANGE_US   = 500            # +/- 0.5 ms around neutral -> 1000–2000 us total

# ==========================
# NETWORK CONFIG
# ==========================
LISTEN_HOST = "0.0.0.0"
LISTEN_PORT = 9000

# Optional telemetry back to PC (set to None to disable)
# If your PC is always the sender, we can just "reply" to the last sender automatically.
TELEMETRY_HZ = 10.0  # send telemetry ~10 Hz (if a client has connected)

# Watchdog: if we don't receive a packet for this long, go to neutral
FAILSAFE_TIMEOUT_S = 0.5

# ==========================
# PACKET FORMATS
# ==========================
CMD_FMT = "<4sI4h"
CMD_SIZE = struct.calcsize(CMD_FMT)
CMD_MAGIC = b"SUB1"

STAT_FMT = "<4sIH4hB"
STAT_MAGIC = b"STAT"

# ==========================
# HELPER FUNCTIONS
# ==========================
def us_to_duty_cycle(pulse_us: float, period_ms: float = 20.0) -> float:
    period_us = period_ms * 1000.0
    return (pulse_us / period_us) * 100.0


def throttle_to_pulse_us(throttle: float) -> float:
    throttle = max(-1.0, min(1.0, throttle))  # clamp
    return NEUTRAL_US + throttle * RANGE_US


def i16_to_throttle(v: int) -> float:
    # v in [-1000, 1000] -> throttle [-1, 1]
    if v > 1000:
        v = 1000
    elif v < -1000:
        v = -1000
    return v / 1000.0


def throttle_to_i16(t: float) -> int:
    t = max(-1.0, min(1.0, t))
    return int(round(t * 1000.0))


# ==========================
# MAIN
# ==========================
def main():
    GPIO.setmode(GPIO.BCM)

    esc_pwms = []
    for pin in ESC_PINS:
        GPIO.setup(pin, GPIO.OUT, initial=GPIO.LOW)
        pwm = GPIO.PWM(pin, PWM_FREQ)
        pwm.start(0.0)
        esc_pwms.append(pwm)

    def apply_throttles(throttles):
        # throttles: list[float] length 4 in [-1,1]
        for i, pwm in enumerate(esc_pwms):
            pulse = throttle_to_pulse_us(throttles[i])
            duty = us_to_duty_cycle(pulse)
            pwm.ChangeDutyCycle(duty)

    neutral = [0.0, 0.0, 0.0, 0.0]

    # Arm ESCs
    try:
        print("Arming ESCs... sending NEUTRAL for 3 seconds.")
        apply_throttles(neutral)
        time.sleep(3.0)
        print("ESCs armed.")

        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind((LISTEN_HOST, LISTEN_PORT))
        sock.settimeout(0.05)

        print(f"UDP motor server listening on {LISTEN_HOST}:{LISTEN_PORT}")
        print("Waiting for motor commands from PC... (Ctrl+C to stop)")

        last_rx_t = 0.0
        last_seq = 0
        last_sender = None

        last_applied = neutral[:]
        last_telem_t = 0.0

        while True:
            now = time.time()

            # ---- Receive command packet (non-blocking via timeout)
            got_packet = False
            try:
                data, addr = sock.recvfrom(2048)
                got_packet = True
                last_sender = addr
            except socket.timeout:
                data = None
            except Exception as e:
                print("Socket error:", e)
                data = None

            if got_packet and data:
                if len(data) >= CMD_SIZE:
                    magic, seq, m1, m2, m3, m4 = struct.unpack(CMD_FMT, data[:CMD_SIZE])
                    if magic == CMD_MAGIC:
                        throttles = [
                            i16_to_throttle(m1),
                            i16_to_throttle(m2),
                            i16_to_throttle(m3),
                            i16_to_throttle(m4),
                        ]
                        apply_throttles(throttles)
                        last_applied = throttles
                        last_rx_t = now
                        last_seq = seq
                    else:
                        # Unknown packet; ignore
                        pass

            # ---- Failsafe watchdog
            age = now - last_rx_t if last_rx_t > 0 else 999.0
            armed = 1 if age <= FAILSAFE_TIMEOUT_S else 0
            if armed == 0 and last_applied != neutral:
                apply_throttles(neutral)
                last_applied = neutral[:]

            # ---- Telemetry reply (optional)
            if last_sender is not None and TELEMETRY_HZ and TELEMETRY_HZ > 0:
                if now - last_telem_t >= (1.0 / TELEMETRY_HZ):
                    age_ms = int(min(65535, max(0, round(age * 1000.0))))
                    payload = struct.pack(
                        STAT_FMT,
                        STAT_MAGIC,
                        int(last_seq) & 0xFFFFFFFF,
                        age_ms,
                        throttle_to_i16(last_applied[0]),
                        throttle_to_i16(last_applied[1]),
                        throttle_to_i16(last_applied[2]),
                        throttle_to_i16(last_applied[3]),
                        armed,
                    )
                    try:
                        sock.sendto(payload, last_sender)
                    except Exception:
                        pass
                    last_telem_t = now

    except KeyboardInterrupt:
        print("\nStopping (Ctrl+C).")
    finally:
        print("Failsafe: sending NEUTRAL and cleaning up GPIO...")
        try:
            apply_throttles(neutral)
            time.sleep(1.0)
        except Exception:
            pass
        for pwm in esc_pwms:
            try:
                pwm.stop()
            except Exception:
                pass
        GPIO.cleanup()
        print("Done.")


if __name__ == "__main__":
    main()
