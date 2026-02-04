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
PITCH_MAX = 0.20
YAW_MAX  = 0.20


def compute_motors(lt_x: float, lt_y: float, rt_x: float, triggers: float, a_btn: int, pad: Tuple[int, int, int, int]) -> Tuple[float, float, float, float, int, float]:

    pad_up, pad_right, pad_down, pad_left = pad

    if a_btn > 0:
        arm = 1
    else:
        arm = 0

    # Control throttle
    if abs(triggers) > 0.05:
        throttle = clamp(triggers, -1.0, 1.0)
    
    # Control yaw
    if abs(rt_x) > 0.05:
        yaw = clamp(rt_x, -1.0, 1.0)
    elif pad_left > 0 and abs(rt_x) < 0.05:
        yaw = YAW_MAX
    elif pad_right > 0 and abs(rt_x) < 0.05:
        yaw = -YAW_MAX
    else:
        yaw = 0.0

    #Control pitch
    if abs(lt_y) > 0.05:
        pitch = clamp(lt_y, -1.0, 1.0)
    elif pad_up > 0 and abs(lt_y) < 0.05:
        pitch = PITCH_MAX
    elif pad_down > 0 and abs(lt_y) < 0.05:
        pitch = -PITCH_MAX
    else:
        pitch = 0.0

    #Control roll
    if abs(lt_x) > 0.05:
        roll = clamp(lt_x, -1.0, 1.0)
    else:
        roll = 0.0

    debug = True
    if debug:
        print(f"[DEBUG] lt_y={lt_y:.2f} rt_x={rt_x:.2f} trig={triggers:.2f} pad={pad} -> throttle={throttle:.2f} yaw={yaw:.2f} pitch={pitch:.2f} roll={roll:.2f}")
        time.sleep(0.1)

    return tuple(clamp(v, -1.0, 1.0) for v in (throttle, yaw, pitch, roll, arm))

@dataclass
class MotorCommand:
    throttle: float = 0.0  # -1..1
    yaw: float = 0.0
    pitch: float = 0.0
    roll: float = 0.0

    def pct(self):
        return (self.throttle * 100.0, self.yaw * 100.0, self.pitch * 100.0, self.roll * 100.0)

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
                throttle_to_i16(cmd.throttle),
                throttle_to_i16(cmd.yaw),
                throttle_to_i16(cmd.pitch),
                throttle_to_i16(cmd.roll),
                throttle_to_i16(cmd.arm),
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

        pressed = self.controller.get_buttons()
        lt_x, lt_y = self.controller.get_left_stick()
        rt_x, _rt_y = self.controller.get_right_stick()
        triggers = self.controller.get_triggers()
        a_btn = pressed[xbox360_controller.A]
        pad = self.controller.get_pad()

        throttle, yaw, pitch, roll, arm = compute_motors(lt_x, lt_y, rt_x, triggers, a_btn, pad)
        self.latest_cmd = MotorCommand(throttle=throttle, yaw=yaw, pitch=pitch, roll=roll)

        if self.debug and (time.time() - self._last_debug) > 1.0:
            self._last_debug = time.time()
            print(f"[DEBUG] lt=({lt_x:.2f},{lt_y:.2f}) rt_x={rt_x:.2f} trig={triggers:.2f} pad={pad} arm={a_btn} -> m={self.latest_cmd}")

        return self.latest_cmd
