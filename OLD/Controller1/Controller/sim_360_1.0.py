import cv2
import socket
import struct
import numpy as np
import time
import random
import pygame
import paramiko

# ==========================================================
# SSH SETTINGS (START LAUNCHER ON PI)
# ==========================================================
PI_SSH_HOST = "172.16.3.52" #"10.42.0.242"
PI_SSH_USER = "pi"
PI_SSH_PASSWORD = "raspberry"
PI_COMMAND = "nohup python3 /home/pi/Project-Angler-Fish/Pi/sub_launcher.py > launcher.log 2>&1 &"
PI_UPDATE = "cd ~/Project-Angler-Fish && git pull"

def start_launcher_on_pi():
    client = paramiko.SSHClient()
    client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    try:
        client.connect(PI_SSH_HOST, username=PI_SSH_USER, password=PI_SSH_PASSWORD, timeout=5)
        # print("Connected Updating")
        # client.exec_command(PI_UPDATE)
        # time.sleep(2)
        client.exec_command(PI_COMMAND)
        print("[SSH] Launcher started on Pi")
    except Exception as e:
        print("[SSH] Error:", e)
    finally:
        client.close()

# ==========================================================
# XBOX 360 CONTROLLER MAP (PYGAME)
# ==========================================================
AXIS_LX = 0    # Left Stick X
AXIS_LY = 1    # Left Stick Y
AXIS_LT = 2    # Left Trigger
AXIS_RX = 3    # Right Stick X
AXIS_RY = 4    # Right Stick Y
AXIS_RT = 5    # Right Trigger

BTN_A     = 0
BTN_B     = 1
BTN_X     = 2
BTN_Y     = 3
BTN_LB    = 4
BTN_RB    = 5
BTN_BACK  = 6
BTN_START = 7
BTN_LS    = 8
BTN_RS    = 9
BTN_GUIDE = 10

HAT_INDEX = 0

# ==========================================================
# NETWORK SETTINGS
# ==========================================================
HOST = "10.42.0.50"
CAMERA_PORT = 8000
MOTOR_PORT = 9000

CMD_FMT = "<4sI4h"
CMD_MAGIC = b"SUB1"

# ==========================================================
# UTILITIES
# ==========================================================
def clamp(v, lo, hi):
    return max(lo, min(hi, v))

def throttle_to_i16(v):
    return int(clamp(v, -1.0, 1.0) * 1000)

# ==========================================================
# MOTOR MIXING
# ==========================================================
ROLL_MAX = 0.20
VERT_MAX = 0.20
ROT_MAX  = 0.20

def compute_motors(lx, ly, rx, l2, r2, hat_x, hat_y):
    # Triggers -> now control M3/M4 (surge)
    fwd = (r2 + 1) * 0.5
    rev = (l2 + 1) * 0.5
    surge = fwd - rev

    m3 = surge
    m4 = surge

    # Right stick X (roll/yaw trim) -> applies to surge pair (M3/M4)
    if abs(rx) > 0.1:
        roll = rx * ROLL_MAX
        m3 += roll
        m4 -= roll

    # Left stick Y + D-pad -> now control M1/M2 (vertical + rotation)
    vert = 0
    if abs(ly) > 0.1:
        vert = -ly * VERT_MAX
    if abs(hat_y) > abs(vert):
        vert = hat_y * VERT_MAX

    m1 = vert + hat_x * ROT_MAX
    m2 = vert - hat_x * ROT_MAX

    return tuple(clamp(v, -1, 1) for v in (m1, m2, m3, m4))

# ==========================================================
# INIT PYGAME + CONTROLLER
# ==========================================================
pygame.init()
pygame.joystick.init()

if pygame.joystick.get_count() == 0:
    raise RuntimeError("No Xbox 360 controller detected")

js = pygame.joystick.Joystick(0)
js.init()
print("Connected Controller:", js.get_name())

# ==========================================================
# START PI SYSTEM
# ==========================================================
start_launcher_on_pi()
time.sleep(1)

# ==========================================================
# SOCKETS
# ==========================================================
cam_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
cam_sock.connect((HOST, CAMERA_PORT))

motor_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
seq = 0

# ==========================================================
# MAIN LOOP
# ==========================================================
data = b""
while True:
    # ---- CAMERA FRAME ----
    while len(data) < 4:
        data += cam_sock.recv(4096)
    size = struct.unpack(">I", data[:4])[0]
    data = data[4:]

    while len(data) < size:
        data += cam_sock.recv(4096)

    frame = cv2.imdecode(np.frombuffer(data[:size], np.uint8), cv2.IMREAD_COLOR)
    data = data[size:]

    pygame.event.pump()

    lx = js.get_axis(AXIS_LX)
    ly = js.get_axis(AXIS_LY)
    rx = js.get_axis(AXIS_RX)
    l2 = js.get_axis(AXIS_LT)
    r2 = js.get_axis(AXIS_RT)
    hat_x, hat_y = js.get_hat(HAT_INDEX)

    m1, m2, m3, m4 = compute_motors(lx, ly, rx, l2, r2, hat_x, hat_y)

    pkt = struct.pack(
        CMD_FMT,
        CMD_MAGIC,
        seq,
        throttle_to_i16(m1),
        throttle_to_i16(m2),
        throttle_to_i16(m3),
        throttle_to_i16(m4),
    )
    motor_sock.sendto(pkt, (HOST, MOTOR_PORT))
    seq += 1

    cv2.putText(frame, f"M1:{m1:+.2f} M2:{m2:+.2f}", (20, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,255,255), 2)
    cv2.putText(frame, f"M3:{m3:+.2f} M4:{m4:+.2f}", (20, 60),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,255,255), 2)

    cv2.imshow("Xbox 360 Controller Simulation", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# ==========================================================
# CLEANUP
# ==========================================================
motor_sock.sendto(struct.pack(CMD_FMT, CMD_MAGIC, seq, 0, 0, 0, 0), (HOST, MOTOR_PORT))
cam_sock.close()
motor_sock.close()
cv2.destroyAllWindows()
