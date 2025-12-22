import json
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


class XboxKinematics:
    """
    Implements mapping from control_kinematics.pdf:
      - Triggers drive m3/m4 (surge) +/-100%
      - Left stick Y drives m1/m2 (heave) +/-20%
      - D-pad left/right adds yaw differential on m3/m4 +/-20%
      - Right stick X adds roll differential on m1/m2 +/-20%
      - D-pad up/down adds heave on m1/m2 +/-20%
    Notes:
      - Axis conventions: left stick up is -1, down is +1; right stick left is -1, right +1.
      - Triggers are -1 (released) .. +1 (fully pressed).
    """
    def __init__(self, deadzone: float = 0.08):
        self.deadzone = deadzone

    def _dz(self, v: float) -> float:
        return 0.0 if abs(v) < self.deadzone else v

    def compute(self, lt: float, rt: float, left_y: float, right_x: float, hat: Tuple[int, int]) -> MotorCommand:
        # Convert triggers (-1..+1) => (0..1)
        lt01 = clamp((lt + 1.0) * 0.5, 0.0, 1.0)
        rt01 = clamp((rt + 1.0) * 0.5, 0.0, 1.0)

        # Base surge (m3/m4)
        surge = 100.0 * rt01 + (-100.0) * lt01  # forward positive, reverse negative

        # Yaw via D-pad left/right
        hat_x, hat_y = hat
        yaw = 20.0 * hat_x  # left=-1 => -20 (rotate left), right=+1 => +20

        # Heave via left stick Y and D-pad up/down
        # Left stick up is -1 but should map to +20 (Up)
        left_y = self._dz(left_y)
        heave_from_stick = (-left_y) * 20.0
        heave_from_hat = (-hat_y) * 20.0  # hat_y=+1 is up in pygame; so -hat_y => down negative
        heave = clamp(heave_from_stick + heave_from_hat, -20.0, 20.0)

        # Roll via right stick X (left=-1 => roll left)
        right_x = self._dz(right_x)
        roll = right_x * 20.0  # left=-1 => -20

        # Compose motor outputs
        # m1/m2 are vertical: heave (same sign) + roll differential
        m1 = clamp(heave + roll, -20.0, 20.0)
        m2 = clamp(heave - roll, -20.0, 20.0)

        # m3/m4 are horizontal: surge (same sign) + yaw differential
        m3 = clamp(surge + (-yaw), -100.0, 100.0)  # yaw left (-20) => m3 +20? tune if needed
        m4 = clamp(surge + (yaw), -100.0, 100.0)

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
    Reads an Xbox controller using pygame and outputs MotorCommand continuously.
    """
    def __init__(self):
        pygame.init()
        pygame.joystick.init()
        if pygame.joystick.get_count() == 0:
            raise RuntimeError("No controller detected (pygame.joystick.get_count()==0).")
        self.js = pygame.joystick.Joystick(0)
        self.js.init()

        self.kin = XboxKinematics()
        self.latest_cmd = MotorCommand()

    def poll(self) -> MotorCommand:
        pygame.event.pump()

        # Axes indices can vary; these are common defaults for Xbox controllers in pygame.
        # You can print all axes to verify on your system.
        left_y = self.js.get_axis(1)     # left stick vertical
        right_x = self.js.get_axis(3)    # right stick horizontal (sometimes 2 or 4)

        # Triggers sometimes share one axis; on many systems they are axis 2 and 5 or a combined axis.
        # We'll try a few patterns:
        lt =  -1.0
        rt =  -1.0

        axis_count = self.js.get_numaxes()
        if axis_count >= 6:
            # common on Linux: LT axis 2, RT axis 5
            lt = self.js.get_axis(2)
            rt = self.js.get_axis(5)
        elif axis_count >= 3:
            # fallback: combined trigger axis
            comb = self.js.get_axis(2)  # -1..1 where one trigger moves negative and other positive on some drivers
            # We'll interpret negative as LT, positive as RT (heuristic)
            lt = clamp(-comb, -1.0, 1.0)
            rt = clamp(comb, -1.0, 1.0)

        hat = (0, 0)
        if self.js.get_numhats() > 0:
            hat = self.js.get_hat(0)  # (x,y) in {-1,0,1}

        cmd = self.kin.compute(lt=lt, rt=rt, left_y=left_y, right_x=right_x, hat=hat)
        self.latest_cmd = cmd
        return cmd
