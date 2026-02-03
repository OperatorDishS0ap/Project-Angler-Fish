#!/usr/bin/env python3
"""
ppmrx_xbox360_betaflight.py

Raspberry Pi Zero "PPM receiver":
- Reads Xbox 360 controller via pygame (USB)
- Outputs 8-channel PPM (CPPM) at 50 Hz using pigpio waveforms
- Intended for Betaflight PPM RX input

Default channel order:
CH1 Roll
CH2 Pitch
CH3 Throttle
CH4 Yaw
CH5 AUX1 (Arm toggle)
CH6 AUX2 (optional)
CH7 AUX3 (optional)
CH8 AUX4 (optional)

PPM format (common):
- Fixed HIGH pulse (PULSE_US), followed by LOW gap so that each channel slot totals ch_us
- Sync gap at end so total frame length = FRAME_US
"""

import time
import signal
import sys

import pygame
import pigpio

# ----------------------------
# PPM settings
# ----------------------------
PPM_GPIO = 18          # GPIO pin to output PPM (BCM numbering). GPIO18 is convenient.
FRAME_HZ = 50
FRAME_US = int(1_000_000 / FRAME_HZ)  # 20,000 us
CHANNELS = 8
PULSE_US = 300         # fixed HIGH pulse width (typical 300 us)
MIN_US = 1000
MID_US = 1500
MAX_US = 2000

# Failsafe if no joystick updates for this long
FAILSAFE_S = 0.6

# Stick deadzone
DEADZONE = 0.08

# ----------------------------
# Xbox 360 mappings (may vary; use the debug snippet below if needed)
# ----------------------------
AXIS_LX = 0
AXIS_LY = 1
AXIS_RX = 3
AXIS_RT = 5           # RT commonly axis 5 on Linux; may differ

BTN_A = 0             # toggle ARM on CH5
BTN_BACK = 6          # exit
BTN_START = 7         # pause/resume PPM updates

# ----------------------------
def clamp(x, lo, hi):
    return lo if x < lo else hi if x > hi else x

def apply_deadzone(x, dz=DEADZONE):
    if abs(x) < dz:
        return 0.0
    s = 1.0 if x >= 0 else -1.0
    return s * (abs(x) - dz) / (1.0 - dz)

def axis_to_us(x):
    # x in [-1,1] -> [1000,2000] with 1500 center
    x = clamp(x, -1.0, 1.0)
    return int(round(MID_US + 500 * x))

def throttle_from_rt(joy):
    """
    RT axis often reports [-1..+1] where -1 = released, +1 = pressed.
    Map to [0..1].
    """
    v = joy.get_axis(AXIS_RT)
    t = (v + 1.0) / 2.0
    return clamp(t, 0.0, 1.0)

def throttle_to_us(t):
    return int(round(MIN_US + t * (MAX_US - MIN_US)))

class PPMOut:
    def __init__(self, pi: pigpio.pi, gpio: int):
        self.pi = pi
        self.gpio = gpio
        self.pi.set_mode(gpio, pigpio.OUTPUT)
        self.pi.write(gpio, 0)
        self._wave_id = None

    def stop(self):
        self.pi.wave_tx_stop()
        if self._wave_id is not None:
            try:
                self.pi.wave_delete(self._wave_id)
            except pigpio.error:
                pass
            self._wave_id = None
        self.pi.write(self.gpio, 0)

    def send_frame_repeat(self, ch_us_list):
        """
        Build one PPM frame waveform and transmit it in repeat mode.
        ch_us_list must be length CHANNELS, each in [MIN_US..MAX_US].
        """
        if len(ch_us_list) != CHANNELS:
            raise ValueError(f"Expected {CHANNELS} channels")

        # Clamp channels
        ch = [clamp(int(v), MIN_US, MAX_US) for v in ch_us_list]

        # Total time used by channel slots:
        # each slot is ch_us (includes PULSE_US high + (ch_us - PULSE_US) low)
        used_us = sum(ch)

        # Sync gap to complete the frame length
        sync_us = FRAME_US - used_us
        # Safety: ensure we always have a reasonable sync gap
        if sync_us < 3000:
            # If channels consume too much time, force a minimum sync,
            # and effectively increase frame length (rare unless extreme settings).
            sync_us = 3000

        pulses = []

        # Build pulses: for each channel -> HIGH for PULSE_US, LOW for (ch_us - PULSE_US)
        for v in ch:
            pulses.append(pigpio.pulse(1 << self.gpio, 0, PULSE_US))
            pulses.append(pigpio.pulse(0, 1 << self.gpio, v - PULSE_US))

        # Sync: just keep LOW for sync_us
        pulses.append(pigpio.pulse(0, 1 << self.gpio, sync_us))

        # Replace previous waveform
        self.pi.wave_tx_stop()
        if self._wave_id is not None:
            try:
                self.pi.wave_delete(self._wave_id)
            except pigpio.error:
                pass
            self._wave_id = None

        self.pi.wave_add_generic(pulses)
        wid = self.pi.wave_create()
        if wid < 0:
            raise RuntimeError("Failed to create pigpio waveform (out of wave resources?)")

        self._wave_id = wid
        self.pi.wave_send_repeat(wid)

# ----------------------------
running = True
def _sig(_a, _b):
    global running
    running = False

signal.signal(signal.SIGINT, _sig)
signal.signal(signal.SIGTERM, _sig)

def main():
    # --- Joystick init
    pygame.init()
    pygame.joystick.init()

    if pygame.joystick.get_count() == 0:
        raise RuntimeError("No Xbox 360 controller detected over USB.")

    joy = pygame.joystick.Joystick(0)
    joy.init()
    print(f"[OK] Controller: {joy.get_name()} axes={joy.get_numaxes()} buttons={joy.get_numbuttons()}")

    # --- pigpio init
    pi = pigpio.pi()
    if not pi.connected:
        raise RuntimeError("pigpio daemon not running. Try: sudo systemctl start pigpiod")

    ppm = PPMOut(pi, PPM_GPIO)
    print(f"[OK] PPM output on GPIO{PPM_GPIO} @ {FRAME_HZ} Hz, {CHANNELS} channels")

    # State
    last_event_t = time.monotonic()
    sending_enabled = True
    armed = False

    # Channels default:
    # CH1 roll, CH2 pitch, CH3 thr, CH4 yaw, CH5 aux1 arm, CH6..CH8 mid
    ch = [MID_US, MID_US, MIN_US, MID_US, MIN_US, MID_US, MID_US, MID_US]

    def apply_failsafe():
        nonlocal armed, ch
        armed = False
        ch[0] = MID_US
        ch[1] = MID_US
        ch[2] = MIN_US
        ch[3] = MID_US
        ch[4] = MIN_US  # AUX1 disarm

    # Start in failsafe-safe state
    apply_failsafe()
    ppm.send_frame_repeat(ch)

    print("[RUN] A=toggle ARM (CH5), START=pause/resume, BACK=exit. Ctrl+C to quit.")

    try:
        while running:
            # Events update joystick state
            for event in pygame.event.get():
                if event.type in (pygame.JOYAXISMOTION, pygame.JOYBUTTONDOWN, pygame.JOYBUTTONUP):
                    last_event_t = time.monotonic()

                if event.type == pygame.JOYBUTTONDOWN:
                    if event.button == BTN_A:
                        armed = not armed
                        ch[4] = MAX_US if armed else MIN_US
                        print(f"[ARM] {'ON' if armed else 'OFF'} (CH5={ch[4]})")

                    elif event.button == BTN_START:
                        sending_enabled = not sending_enabled
                        print(f"[PPM] sending_enabled={sending_enabled}")

                    elif event.button == BTN_BACK:
                        print("[EXIT] BACK pressed")
                        break

            now = time.monotonic()
            if (now - last_event_t) > FAILSAFE_S:
                apply_failsafe()

            if sending_enabled:
                # Read axes
                lx = apply_deadzone(joy.get_axis(AXIS_LX))
                ly = apply_deadzone(joy.get_axis(AXIS_LY))
                rx = apply_deadzone(joy.get_axis(AXIS_RX))

                # Map to channels
                ch[0] = axis_to_us(lx)      # Roll
                ch[1] = axis_to_us(-ly)     # Pitch (invert so stick up = forward)
                ch[3] = axis_to_us(rx)      # Yaw

                # Throttle from RT
                t = throttle_from_rt(joy)
                ch[2] = throttle_to_us(t)

                # AUX1 arm already set by A toggle (ch[4])
                # CH6..CH8 left at MID unless you add features

                # Update PPM waveform (rebuild + repeat)
                ppm.send_frame_repeat(ch)

            time.sleep(0.01)

    finally:
        print("\n[EXIT] Failsafe + stop PPM")
        try:
            apply_failsafe()
            ppm.send_frame_repeat(ch)
            time.sleep(0.2)
        except Exception:
            pass
        ppm.stop()
        pi.stop()
        pygame.quit()

if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        print("[ERROR]", e)
        sys.exit(1)
