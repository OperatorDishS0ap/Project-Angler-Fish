#!/usr/bin/env python3
import time
import pigpio

# =========================
# CONFIG
# =========================
GPIO_ESC = 13          # GPIO to probe
ESC_FREQ = 400         # Hz
PERIOD_US = int(1_000_000 / ESC_FREQ)  # 2500 us

START_US = 500
END_US   = 2500
STEP_US  = 50

DWELL_TIME = 1.0       # seconds per step

# =========================
# SETUP
# =========================
pi = pigpio.pi()
if not pi.connected:
    raise SystemExit("pigpio daemon not running")

pi.set_mode(GPIO_ESC, pigpio.OUTPUT)

# Disable servo mode (important)
pi.set_servo_pulsewidth(GPIO_ESC, 0)

# Configure PWM @ 400 Hz
pi.set_PWM_frequency(GPIO_ESC, ESC_FREQ)
pi.set_PWM_range(GPIO_ESC, PERIOD_US)   # 1 count = 1 µs

print("PWM pulse-width sweep @ 400 Hz")
print(f"GPIO {GPIO_ESC}")
print(f"Period: {PERIOD_US} us")
print(f"Sweeping {START_US} → {END_US} us in {STEP_US} us steps")
print("Probe with oscilloscope\n")

try:
    for pulse in range(START_US, END_US + STEP_US, STEP_US):
        print(f"Pulse width: {pulse} us")
        pi.set_PWM_dutycycle(GPIO_ESC, pulse)
        time.sleep(DWELL_TIME)

    print("\nSweep complete. Holding neutral (1500 us).")

finally:
    pi.set_PWM_dutycycle(GPIO_ESC, 1500)
    time.sleep(0.5)
    pi.stop()
