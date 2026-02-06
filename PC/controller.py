import os
import socket
import struct
import threading
import time
from dataclasses import dataclass
from typing import Optional, Tuple

import pygame
import xbox360_contm4er  # local file in this folder (pygame-xbox360contm4er)

pygame.init()

# ==========================================================
# MOTOR PACKET (legacy, matches your known-good script)
# ==========================================================
CMD_FMT = "<4sI5h"
CMD_MAGIC = b"SUB1"

def clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))

def m1_to_i16(v: float) -> int:
    """Map -1..+1 -> -1000..+1000 (int16)."""
    return int(clamp(v, -1.0, 1.0) * 1000)

# ==========================================================
# MOTOR MIXING (same behavior as your working example)
# ==========================================================
ROLL_MAX = 0.20
PITCH_MAX = 0.20
YAW_MAX  = 0.20
a_flag = False


def compute_motors(lt_x: float, lt_y: float, rt_x: float, triggers: float, pad: Tuple[int, int, int, int]) -> Tuple[float, float, float, float, float]:

    pad_up, pad_right, pad_down, pad_left = pad

    yaw_flag = 0
    pitch_flag = 0

    # For forward/backward, keep m1 and m2 the same
    if abs(triggers) > 0.05:
        m1 = clamp(triggers, -1.0, 1.0)
        m2 = m1
        yaw_flag = 1 
    else:
        m1, m2 = 0.0, 0.0
        yaw_flag = 0

    # Contorl Yaw 
    if yaw_flag == 0:
        if abs(rt_x) > 0.05:
            m2 = clamp(rt_x, -1.0, 1.0)
            m1 = -m2
        elif pad_left > 0 and abs(rt_x) < 0.05:
            m2 = YAW_MAX
            m1 = -m2
        elif pad_right > 0 and abs(rt_x) < 0.05:
            m2 = -YAW_MAX
            m1 = -m2
        else:
            m1, m2 = 0.0, 0.0

    #Control Pitch
    if abs(lt_y) > 0.05:
        m3 = clamp(-lt_y, -1.0, 1.0)
        m4 = m3
        pitch_flag = 1
    elif pad_up > 0 and abs(lt_y) < 0.05:
        m3 = PITCH_MAX
        m4 = m3
        pitch_flag = 1
    elif pad_down > 0 and abs(lt_y) < 0.05:
        m3 = -PITCH_MAX
        m4 = m3
        pitch_flag = 1
    else:
        m3, m4 = 0.0, 0.0
        pitch_flag = 0

    #Control Roll
    if pitch_flag == 0:
        if abs(lt_x) > 0.05:
            m4 = clamp(lt_x, -1.0, 1.0)
            m3 = -m4
        else:
            m3, m4 = 0.0, 0.0

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
    a_flag: bool = False

    def pct(self):
        a_flag_pct = 100.0 if self.a_flag else 0.0
        return (self.m1 * 100.0, self.m2 * 100.0, self.m3 * 100.0, self.m4 * 100.0, a_flag_pct)

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
            pkt = struct.pack(CMD_FMT, CMD_MAGIC, self._seq, 0, 0, 0, 0, 0)
            self._sock.sendto(pkt, (self.pi_ip, self.pi_port))
        except Exception:
            pass

    def set_target(self, cmd: MotorCommand) -> None:
        self.latest_cmd = cmd

    def _run(self) -> None:
        dt = 1.0 / max(1.0, self.rate_hz)
        while not self._stop.is_set():
            cmd = self.latest_cmd
            a_flag_val = 1000 if cmd.a_flag else 0
            pkt = struct.pack(
                CMD_FMT,
                CMD_MAGIC,
                self._seq,
                m1_to_i16(cmd.m1),
                m1_to_i16(cmd.m2),
                m1_to_i16(cmd.m3),
                m1_to_i16(cmd.m4),
                a_flag_val,
            )
            try:
                self._sock.sendto(pkt, (self.pi_ip, self.pi_port))
                self._seq = (self._seq + 1) & 0xFFFFFFFF
            except Exception:
                pass
            time.sleep(dt)

class XboxContm4erReader:
    """
    Contm4er reader using pygame-xbox360contm4er (xbox360_contm4er.py).
    """
    def __init__(self):
        if pygame.joystick.get_count() == 0:
            raise RuntimeError("No Xbox contm4er detected.")

        self.contm4er = xbox360_contm4er.Contm4er()
        self.debug = os.environ.get("ANGLERFISH_DEBUG_CONTm4ER", "0") == "1"
        self._last_debug = 0.0
        self._a_btn_last_press_time = 0.0
        self.latest_cmd = MotorCommand()

    def poll(self) -> MotorCommand:
        pygame.event.pump()

        pressed = self.contm4er.get_buttons()
        lt_x, lt_y = self.contm4er.get_left_stick()
        rt_x, _rt_y = self.contm4er.get_right_stick()
        triggers = self.contm4er.get_triggers()
        a_btn = pressed[xbox360_contm4er.A]
        pad = self.contm4er.get_pad()

        global a_flag
        # Debounce: only allow toggle if 500ms has passed since last press
        if a_btn:
            current_time = time.time()
            if current_time - self._a_btn_last_press_time > 0.5:
                a_flag = not a_flag
                self._a_btn_last_press_time = current_time

        m1, m2, m3, m4 = compute_motors(lt_x, lt_y, rt_x, triggers, pad)
        self.latest_cmd = MotorCommand(m1=m1, m2=m2, m3=m3, m4=m4, a_flag=a_flag)

        if self.debug and (time.time() - self._last_debug) > 1.0:
            self._last_debug = time.time()
            print(f"[DEBUG] lt=({lt_x:.2f},{lt_y:.2f}) rt_x={rt_x:.2f} trig={triggers:.2f} pad={pad} arm={a_flag} -> m={self.latest_cmd}")

        return self.latest_cmd
