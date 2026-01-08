#!/usr/bin/env python3
import time
import pigpio

# =========================
# CONFIG
# =========================
GPIO_ESC = 18          # GPIO to probe with scope
PULSE_NEUTRAL = 1500   # µs (constant)
TEST_FREQS = [50, 100, 200, 400]  # Hz
DWELL_TIME = 5.0       # seconds at each frequency

# =========================
# SETUP
# =========================
pi = pigpio.pi()
if not pi.connected:
    raise SystemExit("pigpio daemon not running")

pi.set_mode(GPIO_ESC, pigpio.OUTPUT)

# IMPORTANT: disable servo mode
pi.set_servo_pulsewidth(GPIO_ESC, 0)

print("Starting PWM frequency test")
print("Pulse width fixed at 1500us (neutral)")
print("Probe GPIO", GPIO_ESC, "with oscilloscope\n")

try:
    for freq in TEST_FREQS:
        period_us = int(1_000_000 / freq)

        print(f"Setting frequency: {freq} Hz  (period {period_us} us)")

        # Configure PWM
        pi.set_PWM_frequency(GPIO_ESC, freq)
        pi.set_PWM_range(GPIO_ESC, period_us)

        # Dutycycle equals pulse width in microseconds
        pi.set_PWM_dutycycle(GPIO_ESC, PULSE_NEUTRAL)

        # Let it run so you can measure
        time.sleep(DWELL_TIME)

    print("\nTest complete. Returning to neutral.")

finally:
    pi.set_PWM_dutycycle(GPIO_ESC, PULSE_NEUTRAL)
    time.sleep(0.5)
    pi.stop()
