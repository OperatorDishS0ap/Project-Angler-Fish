#!/usr/bin/env python3
"""
INAV MAVLink "receiver" on Raspberry Pi:
- Xbox 360 controller (USB) -> RC_CHANNELS_OVERRIDE over UART (/dev/serial0)

Recommended INAV setup:
- Ports tab: set the wired UART to "MAVLink telemetry" and match baud rate.
- Modes tab: map ARM to AUX1 (CH5), ANGLE/HORIZON/etc to other AUX channels if desired.

Notes:
- INAV generally expects arming/modes via AUX channels. So we toggle CH5 PWM high/low for ARM.
- RC_CHANNELS_OVERRIDE semantics: 1000=low, 1500=mid, 2000=high. :contentReference[oaicite:4]{index=4}
"""

import time
import signal
import sys

import pygame
from pymavlink import mavutil

# ----------------------------
# MAVLink UART settings
# ----------------------------
DEVICE = "/dev/serial0"
BAUD = 115200
SEND_HZ = 50

# ----------------------------
# PWM settings
# ----------------------------
PWM_MIN = 1000
PWM_MAX = 2000
PWM_MID = 1500
PWM_THR_SAFE = 1000  # throttle at minimum for failsafe / idle

# Deadzone for sticks
DEADZONE = 0.08

# If no joystick events for this long -> failsafe
FAILSAFE_S = 0.6

# ----------------------------
# INAV channel mapping
# ----------------------------
# CH1 Roll, CH2 Pitch, CH3 Throttle, CH4 Yaw
# CH5 AUX1 (ARM switch in INAV Modes tab, recommended)
# Set these to 0 if you don't want to override them.
AUX1_ARM_PWM_OFF = 1000
AUX1_ARM_PWM_ON  = 2000

# Optional other AUX channels (examples):
AUX2_CH6 = 0  # e.g., ANGLE mode switch
AUX3_CH7 = 0
AUX4_CH8 = 0

# ----------------------------
# Xbox 360 mappings (Linux often matches these, but verify if needed)
# ----------------------------
AXIS_LX = 0
AXIS_LY = 1
AXIS_RX = 3

# Triggers may be axis 5 (RT) on Linux; sometimes different.
AXIS_RT = 5

BTN_A = 0  # toggle ARM (AUX1)
BTN_BACK = 6
BTN_START = 7

# ----------------------------
def clamp(x, lo, hi):
    return lo if x < lo else hi if x > hi else x

def apply_deadzone(x, dz=DEADZONE):
    if abs(x) < dz:
        return 0.0
    s = 1.0 if x >= 0 else -1.0
    return s * (abs(x) - dz) / (1.0 - dz)

def axis_to_pwm(x, center=PWM_MID, span=500):
    # x in [-1,1] -> [center-span, center+span]
    x = clamp(x, -1.0, 1.0)
    return int(round(center + span * x))

def throttle_from_rt(joy):
    """
    RT axis often reports [-1..+1] with -1 released, +1 pressed.
    Map to [0..1]. If your RT behaves differently, adjust here.
    """
    v = joy.get_axis(AXIS_RT)
    t = (v + 1.0) / 2.0
    return clamp(t, 0.0, 1.0)

def throttle_to_pwm(t):
    return int(round(PWM_MIN + t * (PWM_MAX - PWM_MIN)))

def send_override(master, ch1, ch2, ch3, ch4, ch5=0, ch6=0, ch7=0, ch8=0):
    master.mav.rc_channels_override_send(
        master.target_system,
        master.target_component,
        ch1, ch2, ch3, ch4, ch5, ch6, ch7, ch8
    )

running = True
def _sig(_a, _b):
    global running
    running = False

signal.signal(signal.SIGINT, _sig)
signal.signal(signal.SIGTERM, _sig)

def main():
    # ---- Joystick init
    pygame.init()
    pygame.joystick.init()

    if pygame.joystick.get_count() == 0:
        raise RuntimeError("No Xbox 360 controller detected over USB.")

    joy = pygame.joystick.Joystick(0)
    joy.init()
    print(f"[OK] Controller: {joy.get_name()} axes={joy.get_numaxes()} buttons={joy.get_numbuttons()}")

    # ---- MAVLink init
    print(f"[INIT] MAVLink on {DEVICE} @ {BAUD} ...")
    master = mavutil.mavlink_connection(DEVICE, baud=BAUD)

    # INAV may or may not send a MAVLink heartbeat depending on config.
    # We'll try briefly, but do not fail if it doesn't show up.
    try:
        hb = master.wait_heartbeat(timeout=5)
        print(f"[OK] Heartbeat received sys={master.target_system} comp={master.target_component}")
    except Exception:
        print("[WARN] No heartbeat seen (continuing). Check INAV Ports: MAVLink telemetry enabled on this UART.")

    # State
    last_event_t = time.monotonic()
    arm_on = False
    sending_enabled = True

    period = 1.0 / SEND_HZ
    next_send = time.monotonic()

    # Start neutral + disarmed
    roll = PWM_MID
    pitch = PWM_MID
    yaw = PWM_MID
    thr = PWM_THR_SAFE
    aux1 = AUX1_ARM_PWM_OFF

    print("[RUN] Sending RC overrides at {} Hz".format(SEND_HZ))
    print("      A = toggle ARM (CH5/AUX1), START = pause/resume overrides, BACK = exit")

    while running:
        # Pump events
        for event in pygame.event.get():
            if event.type in (pygame.JOYAXISMOTION, pygame.JOYBUTTONDOWN, pygame.JOYBUTTONUP):
                last_event_t = time.monotonic()

            if event.type == pygame.JOYBUTTONDOWN:
                if event.button == BTN_A:
                    arm_on = not arm_on
                    aux1 = AUX1_ARM_PWM_ON if arm_on else AUX1_ARM_PWM_OFF
                    print(f"[ARM] {'ON' if arm_on else 'OFF'} (CH5={aux1})")

                elif event.button == BTN_START:
                    sending_enabled = not sending_enabled
                    print(f"[SEND] sending_enabled={sending_enabled}")

                elif event.button == BTN_BACK:
                    print("[EXIT] BACK pressed")
                    return

        # Read current axes each loop
        lx = apply_deadzone(joy.get_axis(AXIS_LX))
        ly = apply_deadzone(joy.get_axis(AXIS_LY))
        rx = apply_deadzone(joy.get_axis(AXIS_RX))

        # Map to RC:
        roll = axis_to_pwm(lx)
        pitch = axis_to_pwm(-ly)  # invert so stick up = pitch forward
        yaw = axis_to_pwm(rx)

        # Throttle from RT
        t = throttle_from_rt(joy)
        thr = throttle_to_pwm(t)

        # Failsafe: no updates recently -> neutralize + disarm
        now = time.monotonic()
        if (now - last_event_t) > FAILSAFE_S:
            roll = PWM_MID
            pitch = PWM_MID
            yaw = PWM_MID
            thr = PWM_THR_SAFE
            arm_on = False
            aux1 = AUX1_ARM_PWM_OFF

        # Send at fixed rate
        if sending_enabled and now >= next_send:
            send_override(
                master,
                ch1=roll,
                ch2=pitch,
                ch3=thr,
                ch4=yaw,
                ch5=aux1,
                ch6=AUX2_CH6,
                ch7=AUX3_CH7,
                ch8=AUX4_CH8
            )
            next_send += period

        time.sleep(0.001)

    # On SIGINT/SIGTERM: neutral + stop overriding
    print("\n[EXIT] Neutral + stop overrides")
    try:
        for _ in range(3):
            send_override(master, PWM_MID, PWM_MID, PWM_THR_SAFE, PWM_MID, AUX1_ARM_PWM_OFF, 0, 0, 0)
            time.sleep(0.05)
        # Stop overriding (0 = no override)
        send_override(master, 0, 0, 0, 0, 0, 0, 0, 0)
    except Exception as e:
        print("[WARN] Failed final override:", e)

if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        print("[ERROR]", e)
        sys.exit(1)
