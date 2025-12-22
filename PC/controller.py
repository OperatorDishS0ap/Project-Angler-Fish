import json
import os
import socket
import threading
import time
from dataclasses import dataclass
from typing import Dict, Optional, Tuple

import pygame


def clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


@dataclass
class MotorCommand:
    m1: float = 0.0  # Z axis port bow
    m2: float = 0.0  # Z axis starboard bow
    m3: float = 0.0  # Y axis port stern
    m4: float = 0.0  # Y axis starboard stern

    def as_dict(self) -> Dict[str, float]:
        return {"m1": self.m1, "m2": self.m2, "m3": self.m3, "m4": self.m4}


class TriggerCalibrator:
    """
    Fixes trigger behavior across drivers by calibrating the *neutral* axis value.
    Prevents the classic Windows bug where triggers rest at 0 and get interpreted as "half pressed".
    """
    def __init__(self, neutral: float):
        self.neutral = neutral

    def amount_one_sided(self, raw: float) -> float:
        # Press could be in + direction
        inc = 0.0
        if raw > self.neutral:
            inc = (raw - self.neutral) / max(1e-6, (1.0 - self.neutral))

        # Or press could be in - direction
        dec = 0.0
        if raw < self.neutral:
            dec = (self.neutral - raw) / max(1e-6, (self.neutral - (-1.0)))

        return clamp(max(inc, dec), 0.0, 1.0)

    def split_combined(self, raw: float) -> Tuple[float, float]:
        """
        For combined trigger axis:
          raw < neutral => LT
          raw > neutral => RT
        Returns (lt_amt, rt_amt), each 0..1
        """
        rt = 0.0
        if raw > self.neutral:
            rt = (raw - self.neutral) / max(1e-6, (1.0 - self.neutral))

        lt = 0.0
        if raw < self.neutral:
            lt = (self.neutral - raw) / max(1e-6, (self.neutral - (-1.0)))

        return clamp(lt, 0.0, 1.0), clamp(rt, 0.0, 1.0)


class XboxKinematics:
    """
    Corrected mapping:

      - RT drives M3/M4 forward: 0..+100
      - LT drives M3/M4 reverse: 0..-100   (your correction)

    Other controls (fine trim):
      - Left stick Y: adds heave +/-20 to M1/M2
      - Right stick X: adds roll +/-20 differential to M1/M2
      - D-pad left/right: yaw +/-20 differential on M3/M4
      - D-pad up/down: adds heave +/-20 to M1/M2

    All motors default to 0 unless inputs are present.
    """
    def __init__(self, deadzone: float = 0.08):
        self.deadzone = deadzone

    def _dz(self, v: float) -> float:
        return 0.0 if abs(v) < self.deadzone else v

    def compute(
        self,
        lt_amt: float,
        rt_amt: float,
        left_y: float,
        right_x: float,
        hat: Tuple[int, int]
    ) -> MotorCommand:
        lt_amt = clamp(lt_amt, 0.0, 1.0)
        rt_amt = clamp(rt_amt, 0.0, 1.0)

        # Base surge on stern motors:
        surge = 100.0 * rt_amt + (-100.0) * lt_amt

        # D-pad yaw on stern motors:
        hat_x, hat_y = hat
        yaw = 20.0 * hat_x

        # Vertical (bow) motors:
        left_y = self._dz(left_y)
        heave_from_stick = (-left_y) * 20.0  # stick up is -1 => +20
        heave_from_hat = (hat_y) * 20.0      # hat_y is +1 for UP
        heave = clamp(heave_from_stick + heave_from_hat, -20.0, 20.0)

        right_x = self._dz(right_x)
        roll = right_x * 20.0

        m1 = clamp(heave + roll, -20.0, 20.0)
        m2 = clamp(heave - roll, -20.0, 20.0)

        m3 = clamp(surge - yaw, -100.0, 100.0)
        m4 = clamp(surge + yaw, -100.0, 100.0)

        return MotorCommand(m1=m1, m2=m2, m3=m3, m4=m4)


class MotorUdpSender:
    def __init__(self, pi_ip: str, pi_port: int = 9000, rate_hz: float = 30.0):
        self.pi_ip = pi_ip
        self.pi_port = pi_port
        self.rate_hz = rate_hz

        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None

        self.latest_cmd = MotorCommand()

    def start(self) -> None:
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()

    def set_target(self, cmd: MotorCommand) -> None:
        self.latest_cmd = cmd

    def _run(self) -> None:
        dt = 1.0 / max(1.0, self.rate_hz)
        while not self._stop.is_set():
            payload = {"ts": time.time(), **self.latest_cmd.as_dict()}
            try:
                self._sock.sendto(json.dumps(payload).encode("utf-8"), (self.pi_ip, self.pi_port))
            except Exception:
                pass
            time.sleep(dt)


class XboxControllerReader:
    """
    Reads Xbox controller using pygame and outputs MotorCommand.

    Fixes:
      - Trigger neutral calibration (no input => 0, no "-50 at rest")
      - D-pad support when it shows up as BUTTONS on Windows instead of a HAT
    """
    def __init__(self, deadzone: float = 0.08):
        pygame.init()
        pygame.joystick.init()
        if pygame.joystick.get_count() == 0:
            raise RuntimeError("No controller detected (pygame.joystick.get_count()==0).")
        self.js = pygame.joystick.Joystick(0)
        self.js.init()

        self.kin = XboxKinematics(deadzone=deadzone)
        self.latest_cmd = MotorCommand()

        pygame.event.pump()
        self.axis_count = self.js.get_numaxes()
        self.button_count = self.js.get_numbuttons()

        # Axis mapping defaults (tunable)
        self.LEFT_Y_AXIS = 1
        self.RIGHT_X_CANDIDATES = [3, 4]  # Windows often uses 4; Linux often uses 3

        # Triggers:
        self.LT_AXIS = 2 if self.axis_count > 2 else None
        self.RT_AXIS = 5 if self.axis_count > 5 else None
        self.COMBINED_TRIGGER_AXIS = 2 if self.axis_count > 2 else None

        self._lt_cal = None
        self._rt_cal = None
        self._comb_cal = None

        if self.LT_AXIS is not None:
            self._lt_cal = TriggerCalibrator(self.js.get_axis(self.LT_AXIS))
        if self.RT_AXIS is not None:
            self._rt_cal = TriggerCalibrator(self.js.get_axis(self.RT_AXIS))
        if self.COMBINED_TRIGGER_AXIS is not None:
            self._comb_cal = TriggerCalibrator(self.js.get_axis(self.COMBINED_TRIGGER_AXIS))

        # D-pad as buttons (common for Xbox on Windows pygame):
        # 11=UP, 12=DOWN, 13=LEFT, 14=RIGHT
        self.DPAD_UP = int(os.environ.get("ANGLERFISH_DPAD_UP", "11"))
        self.DPAD_DOWN = int(os.environ.get("ANGLERFISH_DPAD_DOWN", "12"))
        self.DPAD_LEFT = int(os.environ.get("ANGLERFISH_DPAD_LEFT", "13"))
        self.DPAD_RIGHT = int(os.environ.get("ANGLERFISH_DPAD_RIGHT", "14"))

        self._last_debug = 0.0
        self.debug = os.environ.get("ANGLERFISH_DEBUG_CONTROLLER", "0") == "1"

    def _get_axis_safe(self, idx: int) -> float:
        if idx is None or idx < 0 or idx >= self.axis_count:
            return 0.0
        try:
            return float(self.js.get_axis(idx))
        except Exception:
            return 0.0

    def _get_button_safe(self, idx: int) -> int:
        if idx is None or idx < 0 or idx >= self.button_count:
            return 0
        try:
            return int(self.js.get_button(idx))
        except Exception:
            return 0

    def _get_right_x(self) -> float:
        best = 0.0
        for idx in self.RIGHT_X_CANDIDATES:
            v = self._get_axis_safe(idx)
            if abs(v) > abs(best):
                best = v
        return best

    def _get_hat_from_buttons(self) -> Tuple[int, int]:
        up = self._get_button_safe(self.DPAD_UP)
        down = self._get_button_safe(self.DPAD_DOWN)
        left = self._get_button_safe(self.DPAD_LEFT)
        right = self._get_button_safe(self.DPAD_RIGHT)
        return (right - left, up - down)

    def _get_hat(self) -> Tuple[int, int]:
        if self.js.get_numhats() > 0:
            hat = self.js.get_hat(0)
            if hat != (0, 0):
                return hat
        return self._get_hat_from_buttons()

    def _get_triggers(self) -> Tuple[float, float]:
        # Prefer separate axes if present
        if self.RT_AXIS is not None and self.LT_AXIS is not None:
            lt_raw = self._get_axis_safe(self.LT_AXIS)
            rt_raw = self._get_axis_safe(self.RT_AXIS)
            lt_amt = self._lt_cal.amount_one_sided(lt_raw) if self._lt_cal else 0.0
            rt_amt = self._rt_cal.amount_one_sided(rt_raw) if self._rt_cal else 0.0
            return lt_amt, rt_amt

        # Otherwise combined axis
        if self.COMBINED_TRIGGER_AXIS is not None and self._comb_cal is not None:
            comb_raw = self._get_axis_safe(self.COMBINED_TRIGGER_AXIS)
            return self._comb_cal.split_combined(comb_raw)

        return 0.0, 0.0

    def poll(self) -> MotorCommand:
        pygame.event.pump()

        left_y = self._get_axis_safe(self.LEFT_Y_AXIS)
        right_x = self._get_right_x()
        hat = self._get_hat()
        lt_amt, rt_amt = self._get_triggers()

        cmd = self.kin.compute(lt_amt=lt_amt, rt_amt=rt_amt, left_y=left_y, right_x=right_x, hat=hat)
        self.latest_cmd = cmd

        if self.debug and (time.time() - self._last_debug) > 1.0:
            self._last_debug = time.time()
            axes = [round(self._get_axis_safe(i), 3) for i in range(self.axis_count)]
            buttons = [self._get_button_safe(i) for i in range(min(self.button_count, 16))]
            print(f"[DEBUG] axes={axes} buttons0-15={buttons} hat={hat} lt={lt_amt:.2f} rt={rt_amt:.2f} -> m={cmd}")

        return cmd
