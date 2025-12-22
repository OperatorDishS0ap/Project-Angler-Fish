#!/usr/bin/env python3
import json
import socket
import time

import pigpio


# ------------ CONFIG -------------
LISTEN_IP = "0.0.0.0"
LISTEN_PORT = 9000

# GPIO pins for ESC signal (edit to match your wiring)
GPIO_M1 = 18  # Port Bow (Z)  # Port Bow (Z)
GPIO_M2 = 12  # Starboard Bow (Z)  # Starboard Bow (Z)
GPIO_M3 = 13  # Port Stern (Y)  # Port Stern (Y)
GPIO_M4 = 19  # Starboard Stern (Y)  # Starboard Stern (Y)

# ESC pulse widths (microseconds)
PULSE_NEUTRAL = 1500
PULSE_MIN = 1100     # full reverse
PULSE_MAX = 1900     # full forward

# Safety timeout: if no command received, return to neutral.
COMMAND_TIMEOUT_S = 0.5


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def pct_to_pulse(pct: float) -> int:
    """
    Map percent (-100..100) to pulse width.
    """
    pct = clamp(pct, -100.0, 100.0)
    if pct >= 0:
        return int(PULSE_NEUTRAL + (PULSE_MAX - PULSE_NEUTRAL) * (pct / 100.0))
    else:
        return int(PULSE_NEUTRAL + (PULSE_NEUTRAL - PULSE_MIN) * (pct / 100.0))


def main():
    pi = pigpio.pi()
    if not pi.connected:
        raise SystemExit("pigpio daemon not running. Start with: sudo systemctl start pigpiod")

    for gpio in [GPIO_M1, GPIO_M2, GPIO_M3, GPIO_M4]:
        pi.set_mode(gpio, pigpio.OUTPUT)
        pi.set_servo_pulsewidth(gpio, PULSE_NEUTRAL)

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((LISTEN_IP, LISTEN_PORT))
    sock.settimeout(0.2)

    last_rx = time.time()
    last = {"m1": 0.0, "m2": 0.0, "m3": 0.0, "m4": 0.0}

    print(f"[sub_motors] Listening UDP on {LISTEN_IP}:{LISTEN_PORT}")

    while True:
        try:
            data, _addr = sock.recvfrom(2048)
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

        # Failsafe
        if time.time() - last_rx > COMMAND_TIMEOUT_S:
            last = {"m1": 0.0, "m2": 0.0, "m3": 0.0, "m4": 0.0}

        # Apply: m1/m2 intended range is about +/-20, but we still map to ESC full-range after clamping.
        pi.set_servo_pulsewidth(GPIO_M1, pct_to_pulse(last["m1"]))
        pi.set_servo_pulsewidth(GPIO_M2, pct_to_pulse(last["m2"]))
        pi.set_servo_pulsewidth(GPIO_M3, pct_to_pulse(last["m3"]))
        pi.set_servo_pulsewidth(GPIO_M4, pct_to_pulse(last["m4"]))

        time.sleep(0.01)


if __name__ == "__main__":
    main()
