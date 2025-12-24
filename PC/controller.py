import os
import socket
import struct
import threading
import time
from dataclasses import dataclass
from typing import Optional, Tuple

import pygame
import xbox360_controller  # local file in this folder (pygame-xbox360controller)

pygame.init()

# ==========================================================
# MOTOR PACKET (legacy, matches your known-good script)
# ==========================================================
CMD_FMT = "<4sI4h"
CMD_MAGIC = b"SUB1"

def clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))

def throttle_to_i16(v: float) -> int:
    """Map -1..+1 -> -1000..+1000 (int16)."""
    return int(clamp(v, -1.0, 1.0) * 1000)

# ==========================================================
# MOTOR MIXING (same behavior as your working example)
# ==========================================================
ROLL_MAX = 0.20
VERT_MAX = 0.20
ROT_MAX  = 0.20


def compute_motors(lt_y: float, rt_x: float, triggers: float, pad: Tuple[int, int, int, int]) -> Tuple[float, float, float, float]:

    pad_up, pad_right, pad_down, pad_left = pad
    roll_flag = False
    vert_flag = False

    # Control M3/M4
    if abs(triggers) > 0.05:
        surge = clamp(triggers, -1.0, 1.0)
        m3 = surge
        m4 = surge
    elif pad_left > 0 and abs(triggers) < 0.05:
        m3 = -ROT_MAX
        m4 = ROT_MAX
    elif pad_right > 0 and abs(triggers) < 0.05:
        m3 = ROT_MAX
        m4 = -ROT_MAX
    else:
        m3 = 0.0
        m4 = 0.0
    
    # Control M1/M2
    if roll_flag == False:
        if abs(lt_y) > 0.05:
            vert_flag = True
            roll_flag = False
            m1, m2 = -lt_y*VERT_MAX, -lt_y*VERT_MAX
        elif pad_up > 0 and abs(lt_y) < 0.05:
            vert_flag = True
            roll_flag = False
            m1 = VERT_MAX
            m2 = VERT_MAX
        elif pad_down > 0 and abs(lt_y) < 0.05:
            vert_flag = True
            roll_flag = False
            m1 = -VERT_MAX
            m2 = -VERT_MAX
        else:
            vert_flag = False
            roll_flag = False
            m1 = 0.0
            m2 = 0.0

    if vert_flag == False:
        if abs(rt_x) > 0.05:
            roll_flag = True
            vert_flag = False
            m1 = rt_x*ROLL_MAX
            m2 = -rt_x*ROLL_MAX
        else:
            roll_flag = False
            vert_flag = False
            m1 = 0.0
            m2 = 0.0

    debug = False
    if debug:
        print(f"[DEBUG] lt_y={lt_y:.2f} rt_x={rt_x:.2f} trig={triggers:.2f} pad={pad} -> m1={m1:.2f} m2={m2:.2f} m3={m3:.2f} m4={m4:.2f}")
        time.sleep(0.1)

    return tuple(clamp(v, -1.0, 1.0) for v in (m1, m2, m3, m4))

@dataclass
class MotorCommand:
    m1: float = 0.0  # -1..1
    m2: float = 0.0
    m3: float = 0.0
    m4: float = 0.0

    def pct(self):
        return (self.m1 * 100.0, self.m2 * 100.0, self.m3 * 100.0, self.m4 * 100.0)

class MotorUdpSender:
    """Sends the legacy binary packet: <4s I 4h."""
    def __init__(self, pi_ip: str, pi_port: int = 9000, rate_hz: float = 30.0):
        self.pi_ip = pi_ip
        self.pi_port = pi_port
        self.rate_hz = rate_hz
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._seq = 0
        self.latest_cmd = MotorCommand()

    def start(self) -> None:
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        try:
            pkt = struct.pack(CMD_FMT, CMD_MAGIC, self._seq, 0, 0, 0, 0)
            self._sock.sendto(pkt, (self.pi_ip, self.pi_port))
        except Exception:
            pass

    def set_target(self, cmd: MotorCommand) -> None:
        self.latest_cmd = cmd

    def _run(self) -> None:
        dt = 1.0 / max(1.0, self.rate_hz)
        while not self._stop.is_set():
            cmd = self.latest_cmd
            pkt = struct.pack(
                CMD_FMT,
                CMD_MAGIC,
                self._seq,
                throttle_to_i16(cmd.m1),
                throttle_to_i16(cmd.m2),
                throttle_to_i16(cmd.m3),
                throttle_to_i16(cmd.m4),
            )
            try:
                self._sock.sendto(pkt, (self.pi_ip, self.pi_port))
                self._seq = (self._seq + 1) & 0xFFFFFFFF
            except Exception:
                pass
            time.sleep(dt)

class XboxControllerReader:
    """
    Controller reader using pygame-xbox360controller (xbox360_controller.py).
    """
    def __init__(self):
        if pygame.joystick.get_count() == 0:
            raise RuntimeError("No Xbox controller detected.")

        self.controller = xbox360_controller.Controller()
        self.debug = os.environ.get("ANGLERFISH_DEBUG_CONTROLLER", "0") == "1"
        self._last_debug = 0.0
        self.latest_cmd = MotorCommand()

    def poll(self) -> MotorCommand:
        pygame.event.pump()

        lt_x, lt_y = self.controller.get_left_stick()
        rt_x, _rt_y = self.controller.get_right_stick()
        triggers = self.controller.get_triggers()
        pad = self.controller.get_pad()

        m1, m2, m3, m4 = compute_motors(lt_y, rt_x, triggers, pad)
        self.latest_cmd = MotorCommand(m1=m1, m2=m2, m3=m3, m4=m4)

        if self.debug and (time.time() - self._last_debug) > 1.0:
            self._last_debug = time.time()
            print(f"[DEBUG] lt=({lt_x:.2f},{lt_y:.2f}) rt_x={rt_x:.2f} trig={triggers:.2f} pad={pad} -> m={self.latest_cmd}")

        return self.latest_cmd
