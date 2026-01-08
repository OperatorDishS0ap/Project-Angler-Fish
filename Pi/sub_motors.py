#!/usr/bin/env python3
"""
sub_motors_400hz_pigpio.py

UDP-controlled 4x ESC output at 400 Hz using pigpio *PWM mode*
(scope should show ~400 Hz, period ~2.5 ms).

- Supports legacy binary packets: "<4sI4h" with magic b"SUB1"
- Supports JSON fallback: {"m1":..,"m2":..,"m3":..,"m4":..}
- Failsafe: if no command for COMMAND_TIMEOUT_S -> all motors neutral band

Wiring:
M1=GPIO18, M2=GPIO12, M3=GPIO13, M4=GPIO19

Calibration (per your measurement):
- Valid pulse range: 800..2100 us
- Neutral band: 1500..1600 us
  (we hold center at 1550 us and clamp small commands into the band)
"""

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
GPIO_M1 = 18
GPIO_M2 = 12
GPIO_M3 = 13
GPIO_M4 = 19
ALL_GPIOS = (GPIO_M1, GPIO_M2, GPIO_M3, GPIO_M4)

# -------------------------
# ESC / PWM SETTINGS
# -------------------------
ESC_FREQ_HZ = 400
PERIOD_US = int(1_000_000 / ESC_FREQ_HZ)  # 2500us @ 400Hz
PWM_RANGE = PERIOD_US                      # range=2500 => dutycycle "counts" == microseconds

# Calibrated limits
PULSE_MIN = 800
PULSE_MAX = 2100

# Neutral band (measured)
PULSE_NEUTRAL_LO = 1500
PULSE_NEUTRAL_HI = 1600
PULSE_NEUTRAL_C  = (PULSE_NEUTRAL_LO + PULSE_NEUTRAL_HI) // 2  # 1550

ARM_TIME_S = 3.0
COMMAND_TIMEOUT_S = 0.5
LOOP_SLEEP_S = 0.005

# Optional: if command magnitude is very small, force neutral band
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


def pct_to_pulse_us(pct: float) -> int:
    """
    Map -100..+100% to [PULSE_MIN..PULSE_MAX] with a neutral *band*.

    We treat neutral as centered at PULSE_NEUTRAL_C (1550us),
    and for small pct values we force the output into the neutral band
    so the ESC doesn't creep.
    """
    pct = clamp(pct, -100.0, 100.0)

    # Deadband near zero -> hold neutral center (or you can randomize within band)
    if abs(pct) <= PCT_DEADBAND:
        return PULSE_NEUTRAL_C

    if pct > 0:
        # scale forward from neutral_hi -> max
        return int(PULSE_NEUTRAL_HI + (PULSE_MAX - PULSE_NEUTRAL_HI) * (pct / 100.0))
    else:
        # scale reverse from neutral_lo -> min
        return int(PULSE_NEUTRAL_LO + (PULSE_NEUTRAL_LO - PULSE_MIN) * (pct / 100.0))


def setup_pwm_esc(pi: pigpio.pi, gpio: int):
    """
    Configure this GPIO for 400 Hz PWM where dutycycle units == microseconds.
    """
    pi.set_mode(gpio, pigpio.OUTPUT)

    # IMPORTANT: ensure servo mode is off (servo mode is ~50Hz)
    pi.set_servo_pulsewidth(gpio, 0)

    # Configure PWM
    pi.set_PWM_frequency(gpio, ESC_FREQ_HZ)
    pi.set_PWM_range(gpio, PWM_RANGE)

    # Neutral output to arm (use center of neutral band)
    pi.set_PWM_dutycycle(gpio, PULSE_NEUTRAL_C)


def set_pulse_us(pi: pigpio.pi, gpio: int, pulse_us: int):
    """
    In this configuration, dutycycle == pulse width in microseconds.
    """
    pulse_us = clamp(int(pulse_us), PULSE_MIN, PULSE_MAX)
    pi.set_PWM_dutycycle(gpio, pulse_us)


def main():
    pi = pigpio.pi()
    if not pi.connected:
        raise SystemExit("pigpio daemon not running. Start with: sudo systemctl start pigpiod")

    # Setup all ESC outputs
    for g in ALL_GPIOS:
        setup_pwm_esc(pi, g)

    # Debug: confirm PWM config (PWM mode; do NOT call get_servo_pulsewidth)
    for g in ALL_GPIOS:
        print(
            f"GPIO {g}: pwm_freq={pi.get_PWM_frequency(g)}Hz "
            f"pwm_range={pi.get_PWM_range(g)} duty={pi.get_PWM_dutycycle(g)}"
        )

    print(f"[sub_motors_400hz] Arming at neutral (center={PULSE_NEUTRAL_C}us) for {ARM_TIME_S:.1f}s ...")
    time.sleep(ARM_TIME_S)

    # UDP socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((LISTEN_IP, LISTEN_PORT))
    sock.settimeout(SOCK_TIMEOUT_S)

    last_rx = time.time()
    last = {"m1": 0.0, "m2": 0.0, "m3": 0.0, "m4": 0.0}

    print(f"[sub_motors_400hz] Listening UDP on {LISTEN_IP}:{LISTEN_PORT}")

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
                        # Not our magic; try JSON below
                        raise ValueError("Not legacy magic")
                else:
                    # Too small for legacy; try JSON
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
                # ignore malformed packets
                pass

            # Failsafe -> neutral band center
            if time.time() - last_rx > COMMAND_TIMEOUT_S:
                last = {"m1": 0.0, "m2": 0.0, "m3": 0.0, "m4": 0.0}

            # Apply outputs (PWM @ 400Hz)
            set_pulse_us(pi, GPIO_M1, pct_to_pulse_us(last["m1"]))
            set_pulse_us(pi, GPIO_M2, pct_to_pulse_us(last["m2"]))
            set_pulse_us(pi, GPIO_M3, pct_to_pulse_us(last["m3"]))
            set_pulse_us(pi, GPIO_M4, pct_to_pulse_us(last["m4"]))

            time.sleep(LOOP_SLEEP_S)

    finally:
        # Neutral on exit (center of neutral band)
        for g in ALL_GPIOS:
            set_pulse_us(pi, g, PULSE_NEUTRAL_C)
        time.sleep(0.5)
        pi.stop()


if __name__ == "__main__":
    main()
