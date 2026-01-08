#!/usr/bin/env python3
import time
import pigpio

# =========================
# CONFIG
# =========================
GPIO_ESC = 18        # Change if needed
ESC_FREQ = 400       # Hz

PULSE_NEUTRAL = 1500 # µs
PULSE_FORWARD = 1700 # µs (adjust as needed)
PULSE_REVERSE = 1300 # µs (adjust as needed)

PERIOD_US = int(1_000_000 / ESC_FREQ)  # 2500 µs @ 400 Hz
PWM_RANGE = PERIOD_US                  # 1 count per µs

# =========================
# SETUP
# =========================
pi = pigpio.pi()
if not pi.connected:
    raise SystemExit("pigpio daemon not running")

pi.set_mode(GPIO_ESC, pigpio.OUTPUT)

# Disable servo mode
pi.set_servo_pulsewidth(GPIO_ESC, 0)

# Configure PWM
pi.set_PWM_frequency(GPIO_ESC, ESC_FREQ)
pi.set_PWM_range(GPIO_ESC, PWM_RANGE)

def set_pulse(us):
    us = max(1100, min(1900, us))
    pi.set_PWM_dutycycle(GPIO_ESC, us)

# =========================
# ARM ESC
# =========================
print("Arming ESC (neutral)...")
set_pulse(PULSE_NEUTRAL)
time.sleep(3.0)

# =========================
# TEST LOOP
# =========================
print("Starting forward / reverse test")

try:
    while True:
        print("Forward")
        set_pulse(PULSE_FORWARD)
        time.sleep(1.0)

        print("Neutral")
        set_pulse(PULSE_NEUTRAL)
        time.sleep(0.3)

        print("Reverse")
        set_pulse(PULSE_REVERSE)
        time.sleep(1.0)

        print("Neutral")
        set_pulse(PULSE_NEUTRAL)
        time.sleep(0.3)

except KeyboardInterrupt:
    print("\nStopping motor")

finally:
    set_pulse(PULSE_NEUTRAL)
    time.sleep(0.5)
    pi.stop()
