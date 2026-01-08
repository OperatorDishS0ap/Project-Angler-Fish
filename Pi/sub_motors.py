#!/usr/bin/env python3
import json
import socket
import struct
import time

from gpiozero import Servo

LISTEN_IP = "0.0.0.0"
LISTEN_PORT = 9000

# Your wiring:
GPIO_M1 = 18  # Port Bow (Z)
GPIO_M2 = 12  # Starboard Bow (Z)
GPIO_M3 = 13  # Port Stern (Y)
GPIO_M4 = 19  # Starboard Stern (Y)

# ESC pulse settings (microseconds)
PULSE_NEUTRAL = 1500
PULSE_MIN = 1100
PULSE_MAX = 1900

COMMAND_TIMEOUT_S = 0.5

# Legacy binary protocol
CMD_FMT = "<4sI4h"
CMD_MAGIC = b"SUB1"
CMD_SIZE = struct.calcsize(CMD_FMT)


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def i16_to_pct(v_i16: int) -> float:
    # -1000..+1000 => -100..+100
    return clamp((float(v_i16) / 1000.0) * 100.0, -100.0, 100.0)


def pct_to_us(pct: float) -> int:
    pct = clamp(pct, -100.0, 100.0)
    if pct >= 0:
        return int(PULSE_NEUTRAL + (PULSE_MAX - PULSE_NEUTRAL) * (pct / 100.0))
    else:
        return int(PULSE_NEUTRAL + (PULSE_NEUTRAL - PULSE_MIN) * (pct / 100.0))


def us_to_servo_value(pulse_us: int, min_us: int, max_us: int) -> float:
    # Map [min_us..max_us] -> [-1..+1]
    pulse_us = clamp(pulse_us, min_us, max_us)
    span = (max_us - min_us)
    return (2.0 * (pulse_us - min_us) / span) - 1.0


def make_esc(gpio: int) -> Servo:
    # NOTE: No pigpio factory used here -> software timing backend (lgpio/RPi.GPIO)
    return Servo(
        gpio,
        min_pulse_width=PULSE_MIN / 1_000_000.0,
        max_pulse_width=PULSE_MAX / 1_000_000.0,
        frame_width=0.0025,  # 20ms (50Hz)
    )


def set_esc_percent(esc: Servo, pct: float):
    pulse = pct_to_us(pct)
    esc.value = us_to_servo_value(pulse, PULSE_MIN, PULSE_MAX)


def main():
    esc1 = make_esc(GPIO_M1)
    esc2 = make_esc(GPIO_M2)
    esc3 = make_esc(GPIO_M3)
    esc4 = make_esc(GPIO_M4)

    # Neutral on start (helps arming)
    for esc in (esc1, esc2, esc3, esc4):
        set_esc_percent(esc, 0.0)

    # Give ESCs time to arm at neutral
    time.sleep(2.0)

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((LISTEN_IP, LISTEN_PORT))
    sock.settimeout(0.2)

    last_rx = time.time()
    last = {"m1": 0.0, "m2": 0.0, "m3": 0.0, "m4": 0.0}

    print(f"[sub_motors_gpiozero_nopigpio] Listening UDP on {LISTEN_IP}:{LISTEN_PORT}")

    while True:
        try:
            data, _addr = sock.recvfrom(2048)

            # 1) Legacy binary packet
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
                    continue

            # 2) JSON fallback
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

        # Safety timeout -> neutral
        if time.time() - last_rx > COMMAND_TIMEOUT_S:
            last = {"m1": 0.0, "m2": 0.0, "m3": 0.0, "m4": 0.0}

        set_esc_percent(esc1, last["m1"])
        set_esc_percent(esc2, last["m2"])
        set_esc_percent(esc3, last["m3"])
        set_esc_percent(esc4, last["m4"])

        time.sleep(0.01)


if __name__ == "__main__":
    main()
