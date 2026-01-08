#!/usr/bin/env python3
import json
import socket
import struct
import time

import pigpio

LISTEN_IP = "0.0.0.0"
LISTEN_PORT = 9000

# GPIO assignments
GPIO_M1 = 18  # Port Bow
GPIO_M2 = 12  # Starboard Bow
GPIO_M3 = 13  # Port Stern
GPIO_M4 = 19  # Starboard Stern

# ESC pulse widths (microseconds)
PULSE_NEUTRAL = 1500
PULSE_MIN = 1300
PULSE_MAX = 1700

COMMAND_TIMEOUT_S = 0.5

# Legacy binary protocol
CMD_FMT = "<4sI4h"
CMD_MAGIC = b"SUB1"
CMD_SIZE = struct.calcsize(CMD_FMT)


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def i16_to_pct(v_i16: int) -> float:
    return clamp((float(v_i16) / 1000.0) * 100.0, -100.0, 100.0)


def pct_to_pulse(pct: float) -> int:
    pct = clamp(pct, -100.0, 100.0)
    if pct >= 0:
        return int(PULSE_NEUTRAL + (PULSE_MAX - PULSE_NEUTRAL) * (pct / 100.0))
    else:
        return int(PULSE_NEUTRAL + (PULSE_NEUTRAL - PULSE_MIN) * (pct / 100.0))


def setup_esc(pi, gpio):
    pi.set_mode(gpio, pigpio.OUTPUT)

    # Disable servo mode so PWM frequency can be changed
    pi.set_servo_pulsewidth(gpio, 0)

    # Set 400 Hz PWM base frequency
    pi.set_PWM_frequency(gpio, 400)

    # 1 MHz PWM clock gives ~1 µs resolution
    pi.set_PWM_range(gpio, 2500)  # 2.5ms period @ 400Hz

    # Start at neutral
    pi.set_PWM_dutycycle(gpio, PULSE_NEUTRAL / 2500 * 255)


def set_esc_pulse(pi, gpio, pulse_us):
    pulse_us = clamp(pulse_us, PULSE_MIN, PULSE_MAX)

    # Convert microseconds to dutycycle (0–255)
    duty = int((pulse_us / 2500.0) * 255)
    pi.set_PWM_dutycycle(gpio, duty)


def main():
    pi = pigpio.pi()
    if not pi.connected:
        raise SystemExit("pigpio daemon not running")

    for gpio in (GPIO_M1, GPIO_M2, GPIO_M3, GPIO_M4):
        setup_esc(pi, gpio)

    print("[sub_motors_pigpio_400hz] ESCs armed at neutral")
    time.sleep(2.0)

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((LISTEN_IP, LISTEN_PORT))
    sock.settimeout(0.2)

    last_rx = time.time()
    last = dict(m1=0.0, m2=0.0, m3=0.0, m4=0.0)

    print(f"[sub_motors_pigpio_400hz] Listening on {LISTEN_IP}:{LISTEN_PORT}")

    while True:
        try:
            data, _ = sock.recvfrom(2048)

            if len(data) >= CMD_SIZE:
                magic, _seq, m1, m2, m3, m4 = struct.unpack(CMD_FMT, data[:CMD_SIZE])
                if magic == CMD_MAGIC:
                    last = {
                        "m1": i16_to_pct(m1),
                        "m2": i16_to_pct(m2),
                        "m3": i16_to_pct(m3),
                        "m4": i16_to_pct(m4),
                    }
                    last_rx = time.time()
                    continue

            msg = json.loads(data.decode("utf-8", errors="ignore"))
            last.update({
                "m1": float(msg.get("m1", last["m1"])),
                "m2": float(msg.get("m2", last["m2"])),
                "m3": float(msg.get("m3", last["m3"])),
                "m4": float(msg.get("m4", last["m4"])),
            })
            last_rx = time.time()

        except socket.timeout:
            pass
        except Exception:
            pass

        if time.time() - last_rx > COMMAND_TIMEOUT_S:
            last = dict(m1=0.0, m2=0.0, m3=0.0, m4=0.0)

        set_esc_pulse(pi, GPIO_M1, pct_to_pulse(last["m1"]))
        set_esc_pulse(pi, GPIO_M2, pct_to_pulse(last["m2"]))
        set_esc_pulse(pi, GPIO_M3, pct_to_pulse(last["m3"]))
        set_esc_pulse(pi, GPIO_M4, pct_to_pulse(last["m4"]))

        time.sleep(0.002)  # ~500 Hz update loop


if __name__ == "__main__":
    main()
