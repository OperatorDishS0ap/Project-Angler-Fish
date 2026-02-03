#!/usr/bin/env python3
"""
ppmrx_xbox360_betaflight.py

Raspberry Pi Zero PPM "receiver" with bidirectional throttle:
- Xbox 360 controller over USB
- PPM output (8 channels) via pigpio
- RT = Forward throttle
- LT = Reverse throttle
- Throttle neutral = 1500 us
- Terminal debug output

Channel mapping:
CH1 Roll
CH2 Pitch
CH3 Throttle (bidirectional, 1500 neutral)
CH4 Yaw
CH5 AUX1 (ARM)
CH6–CH8 unused (1500)

PPM frame: 50 Hz, 8 channels
"""

import time
import signal
import sys

import pygame
import pigpio

# ============================================================
# PPM CONFIG
# ============================================================
PPM_GPIO = 18          # BCM GPIO pin
FRAME_HZ = 50
FRAME_US = int(1_000_000 / FRAME_HZ)
CHANNELS = 8
PULSE_US = 300

MIN_US = 1000
MID_US = 1500
MAX_US = 2000
THR_NEUTRAL_US = 1500

FAILSAFE_S = 0.6
DEADZONE = 0.08

DEBUG_HZ = 10
DEBUG_PERIOD = 1.0 / DEBUG_HZ

# ============================================================
# XBOX 360 MAPPING (LINUX)
# ============================================================
AXIS_LX = 0
AXIS_LY = 1
AXIS_RX = 3
AXIS_LT = 2     # Left Trigger (reverse)
AXIS_RT = 5     # Right Trigger (forward)

BTN_A = 0       # ARM
BTN_BACK = 6
BTN_START = 7   # Pause PPM

# ============================================================
def clamp(x, lo, hi):
    return lo if x < lo else hi if x > hi else x

def apply_deadzone(x):
    if abs(x) < DEADZONE:
        return 0.0
    return (abs(x) - DEADZONE) / (1.0 - DEADZONE) * (1 if x > 0 else -1)

def axis_to_us(x):
    return int(MID_US + 500 * clamp(x, -1.0, 1.0))

def trigger_to_01(v):
    # Triggers report [-1..1], map to [0..1]
    return clamp((v + 1.0) / 2.0, 0.0, 1.0)

def bidirectional_throttle(rt, lt):
    """
    RT = forward (1500 → 2000)
    LT = reverse (1500 → 1000)
    """
    forward = trigger_to_01(rt)
    reverse = trigger_to_01(lt)
    thrust = forward - reverse
    return int(THR_NEUTRAL_US + thrust * 500)

# ============================================================
class PPMOut:
    def __init__(self, pi, gpio):
        self.pi = pi
        self.gpio = gpio
        self.pi.set_mode(gpio, pigpio.OUTPUT)
        self.wave_id = None

    def stop(self):
        self.pi.wave_tx_stop()
        if self.wave_id is not None:
            self.pi.wave_delete(self.wave_id)
            self.wave_id = None
        self.pi.write(self.gpio, 0)

    def send(self, ch):
        ch = [clamp(v, MIN_US, MAX_US) for v in ch]
        used = sum(ch)
        sync = max(FRAME_US - used, 3000)

        pulses = []
        for v in ch:
            pulses.append(pigpio.pulse(1 << self.gpio, 0, PULSE_US))
            pulses.append(pigpio.pulse(0, 1 << self.gpio, v - PULSE_US))
        pulses.append(pigpio.pulse(0, 1 << self.gpio, sync))

        self.pi.wave_tx_stop()
        if self.wave_id is not None:
            self.pi.wave_delete(self.wave_id)

        self.pi.wave_add_generic(pulses)
        self.wave_id = self.pi.wave_create()
        self.pi.wave_send_repeat(self.wave_id)

# ============================================================
running = True
def shutdown(sig, frame):
    global running
    running = False

signal.signal(signal.SIGINT, shutdown)
signal.signal(signal.SIGTERM, shutdown)

# ============================================================
def main():
    pygame.init()
    pygame.joystick.init()

    if pygame.joystick.get_count() == 0:
        raise RuntimeError("No Xbox controller detected")

    joy = pygame.joystick.Joystick(0)
    joy.init()
    print(f"[OK] Controller: {joy.get_name()}")

    pi = pigpio.pi()
    if not pi.connected:
        raise RuntimeError("pigpio daemon not running")

    ppm = PPMOut(pi, PPM_GPIO)
    print(f"[OK] PPM output on GPIO{PPM_GPIO}")

    last_event = time.monotonic()
    last_debug = 0
    failsafe = False
    sending = True
    armed = False

    ch = [
        MID_US, MID_US, THR_NEUTRAL_US, MID_US,
        MIN_US, MID_US, MID_US, MID_US
    ]

    def apply_failsafe():
        nonlocal failsafe, armed
        if not failsafe:
            print("[FAILSAFE] Throttle neutral + disarm")
        failsafe = True
        armed = False
        ch[0] = MID_US
        ch[1] = MID_US
        ch[2] = THR_NEUTRAL_US
        ch[3] = MID_US
        ch[4] = MIN_US

    apply_failsafe()
    ppm.send(ch)

    print("[RUN] RT=Forward  LT=Reverse  A=ARM  START=Pause  BACK=Exit")

    try:
        while running:
            for e in pygame.event.get():
                if e.type in (pygame.JOYAXISMOTION, pygame.JOYBUTTONDOWN):
                    last_event = time.monotonic()

                if e.type == pygame.JOYBUTTONDOWN:
                    if e.button == BTN_A:
                        armed = not armed
                        ch[4] = MAX_US if armed else MIN_US
                        print(f"[ARM] {'ON' if armed else 'OFF'}")

                    elif e.button == BTN_START:
                        sending = not sending
                        print(f"[PPM] {'ENABLED' if sending else 'PAUSED'}")

                    elif e.button == BTN_BACK:
                        return

            now = time.monotonic()
            if now - last_event > FAILSAFE_S:
                apply_failsafe()
            else:
                failsafe = False

            if sending and not failsafe:
                lx = apply_deadzone(joy.get_axis(AXIS_LX))
                ly = apply_deadzone(joy.get_axis(AXIS_LY))
                rx = apply_deadzone(joy.get_axis(AXIS_RX))

                rt = joy.get_axis(AXIS_RT)
                lt = joy.get_axis(AXIS_LT)

                ch[0] = axis_to_us(lx)
                ch[1] = axis_to_us(-ly)
                ch[3] = axis_to_us(rx)
                ch[2] = bidirectional_throttle(rt, lt)

                ppm.send(ch)

            if now - last_debug >= DEBUG_PERIOD:
                print(
                    f"AXES LX={joy.get_axis(AXIS_LX): .2f} "
                    f"LY={joy.get_axis(AXIS_LY): .2f} "
                    f"RX={joy.get_axis(AXIS_RX): .2f} "
                    f"RT={rt: .2f} LT={lt: .2f} | "
                    f"THR={ch[2]} | "
                    f"{'ARMED' if armed else 'DISARMED'}"
                    f"{' FAILSAFE' if failsafe else ''}"
                )
                last_debug = now

            time.sleep(0.01)

    finally:
        print("\n[EXIT] Neutral throttle, stop PPM")
        apply_failsafe()
        ppm.send(ch)
        time.sleep(0.2)
        ppm.stop()
        pi.stop()
        pygame.quit()

# ============================================================
if __name__ == "__main__":
    main()
