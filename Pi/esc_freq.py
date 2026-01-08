#!/usr/bin/env python3
import time
import pigpio

# =========================
# CONFIG
# =========================
GPIO_ESC = 13          # GPIO to probe
ESC_FREQ = 400         # Hz (fixed)
PERIOD_US = int(1_000_000 / ESC_FREQ)  # 2500 us

# Pulse widths to test (µs)
TEST_PULSES = [
    1100,
    1300,
    1500,  # NEUTRAL
    1700,
    1900,
]

DWELL_TIME = 4.0       # seconds per pulse width

# =========================
# SETUP
# =========================
pi = pigpio.pi()
if not pi.connected:
    raise SystemExit("pigpio daemon not running")

pi.set_mode(GPIO_ESC, pigpio.OUTPUT)

# IMPORTANT: disable servo mode (50 Hz)
pi.set_servo_pulsewidth(GPIO_ESC, 0)

# Configure PWM for 400 Hz
pi.set_PWM_frequency(GPIO_ESC, ESC_FREQ)
pi.set_PWM_range(GPIO_ESC, PERIOD_US)  # 1 count = 1 us

print("PWM pulse-width test @ 400 Hz")
print("GPIO:", GPIO_ESC)
print("Period:", PERIOD_US, "us")
print("Probe with oscilloscope\n")

try:
    for pulse in TEST_PULSES:
        print(f"Pulse width: {pulse} us")
        pi.set_PWM_dutycycle(GPIO_ESC, pulse)
        time.sleep(DWELL_TIME)

    print("\nTest complete. Holding neutral.")

finally:
    pi.set_PWM_dutycycle(GPIO_ESC, 1500)
    time.sleep(0.5)
    pi.stop()
