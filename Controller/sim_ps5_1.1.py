import cv2
import socket
import struct
import numpy as np
import time
import random
import pygame
import subprocess  # for SSH to Pi (optional)
import paramiko

# ------------------------
# SSH SETTINGS TO START LAUNCHER ON PI
# ------------------------
PI_SSH_HOST = "10.42.0.50"      # Pi's Ethernet IP
PI_SSH_USER = "pi"              # Pi username
PI_SSH_PASSWORD = "raspberry"   # <--- CHANGE THIS
PI_COMMAND = "nohup python3 /home/pi/Project-Angler-Fish/Pi/sub_launcher.py > launcher.log 2>&1 &"

def start_launcher_on_pi():
    """Connect to Pi using SSH (with password) and run the launcher command."""
    print(f"[SSH] Connecting to {PI_SSH_USER}@{PI_SSH_HOST} ...")

    client = paramiko.SSHClient()
    client.set_missing_host_key_policy(paramiko.AutoAddPolicy())

    try:
        client.connect(
            hostname=PI_SSH_HOST,
            username=PI_SSH_USER,
            password=PI_SSH_PASSWORD,
            look_for_keys=False,
            allow_agent=False,
            timeout=5
        )

        print("[SSH] Connected! Running launcher...")
        stdin, stdout, stderr = client.exec_command(PI_COMMAND)

        out = stdout.read().decode()
        err = stderr.read().decode()

        if out.strip():
            print("[SSH STDOUT]\n" + out)
        if err.strip():
            print("[SSH STDERR]\n" + err)

        print("[SSH] Launcher started successfully!")

    except Exception as e:
        print(f"[SSH] Error: {e}")

    finally:
        client.close()


# ------------------------
# FONT SETTINGS
# ------------------------
font_large = cv2.FONT_HERSHEY_DUPLEX
font_medium = cv2.FONT_HERSHEY_DUPLEX
font_small = cv2.FONT_HERSHEY_DUPLEX

scale_large = 2
scale_medium = 1
scale_small = 0.7

thick_large = 3
thick_medium = 2
thick_small = 1

# ------------------------
# PI CAMERA TCP SETTINGS
# ------------------------
HOST = "10.42.0.50"  # Pi's Ethernet IP
PORT = 8000

# ------------------------
# MOTOR UDP SETTINGS (NEW)
# ------------------------
MOTOR_UDP_IP = HOST
MOTOR_UDP_PORT = 9000

# Packet format (binary, little-endian): '<4s I 4h'
CMD_FMT = "<4sI4h"
CMD_MAGIC = b"SUB1"

# Telemetry from Pi (optional): '<4s I H 4h B'
STAT_FMT = "<4sIH4hB"
STAT_MAGIC = b"STAT"

def clamp(x, lo, hi):
    return lo if x < lo else hi if x > hi else x

def throttle_to_i16(t: float) -> int:
    t = clamp(t, -1.0, 1.0)
    return int(round(t * 1000.0))

# ------------------------
# CONTROLLER -> MOTOR MIXING (per your PDF table)
# ------------------------
# Motors:
#   M1: Port Bow (Z)
#   M2: Starboard Bow (Z)
#   M3: Port Stern (Y)
#   M4: Starboard Stern (Y)
#
# Mapping implemented:
#   R2: forward  0..100%  -> M1=M2 0..+1.0
#   L2: reverse  0..-100% -> M1=M2 0..-1.0
#   Right stick X (rx): roll differential +/-20% on M1/M2
#   Vertical (choose stronger of D-pad up/down OR Left stick up/down): +/-20% on M3/M4
#   D-pad left/right: rotate differential +/-20% on M3/M4
#
# Notes:
# - pygame triggers: -1 base, +1 fully pressed
# - sticks: -1..+1
ROLL_MAX = 0.20
VERT_MAX = 0.20
ROT_MAX  = 0.20

def compute_motors(lx, ly, rx, ry, l2, r2, hat_x, hat_y):
    # Triggers to forward/reverse
    fwd = (r2 + 1.0) * 0.5   # 0..1
    rev = (l2 + 1.0) * 0.5   # 0..1
    surge = fwd - rev        # -1..+1

    m1 = surge
    m2 = surge

    # Roll differential using right stick X (rx): -1 left, +1 right
    if abs(rx) > 0.10:
        roll = rx * ROLL_MAX
    else:
        roll = 0.0
    m1 += roll
    m2 -= roll

    # Vertical command: choose the stronger of (left stick up/down) or (dpad up/down)
    stick_vert = 0.0
    if abs(ly) > 0.10:
        # ly: -1 up, +1 down  -> up should be +VERT_MAX
        stick_vert = (-ly) * VERT_MAX

    dpad_vert = hat_y * VERT_MAX  # hat_y: +1 up, -1 down

    vert = stick_vert if abs(stick_vert) >= abs(dpad_vert) else dpad_vert

    m3 = vert
    m4 = vert

    # Rotate with dpad left/right: differential on M3/M4
    rot = hat_x * ROT_MAX  # hat_x: -1 left, +1 right
    m3 += rot
    m4 -= rot

    # Clamp final
    m1 = clamp(m1, -1.0, 1.0)
    m2 = clamp(m2, -1.0, 1.0)
    m3 = clamp(m3, -1.0, 1.0)
    m4 = clamp(m4, -1.0, 1.0)
    return m1, m2, m3, m4


# ------------------------
# INIT CONTROLLER
# ------------------------
pygame.init()
pygame.joystick.init()

if pygame.joystick.get_count() == 0:
    print("No controller detected!")
    exit()

js = pygame.joystick.Joystick(0)
js.init()
print("Controller Connected:", js.get_name())

# ------------------------
# START LAUNCHER ON PI (optional)
# ------------------------
start_launcher_on_pi()

# ------------------------
# WAIT BEFORE CONNECTING TO CAMERA
# ------------------------
startup_delay = 5   # seconds — increase if needed
print(f"Waiting {startup_delay} seconds for Pi camera server to start...")
time.sleep(startup_delay)

# ------------------------
# CONNECT TO PI TCP CAMERA (with retry)
# ------------------------
client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
print(f"Connecting to Pi at {HOST}:{PORT} ...")

max_retries = 10
retry_delay = 1.0  # seconds

for attempt in range(1, max_retries + 1):
    try:
        client_socket.connect((HOST, PORT))
        print("Connected to Pi camera!")
        break
    except (ConnectionRefusedError, OSError) as e:
        print(f"Connection attempt {attempt}/{max_retries} failed: {e}")
        if attempt == max_retries:
            print("Giving up on connecting to camera server.")
            raise
        time.sleep(retry_delay)

data = b""
payload_size = 4  # 4 bytes for frame length

# ------------------------
# MOTOR UDP SOCKETS (NEW)
# ------------------------
motor_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
motor_sock.setblocking(False)  # send is blocking-safe; recv telemetry non-blocking
seq = 0

# Telemetry receive buffer/state
last_stat = None
last_stat_t = 0.0

# ------------------------
# SIMULATION STATE
# ------------------------
fake_sensor = 0
last_sensor_update = time.time()

# ------------------------
# WINDOW SETTINGS
# ------------------------
window_width = 1280
window_height = 720

center_x = window_width // 2
center_y = window_height // 2

# BLINK
blink_state = True
last_blink_time = time.time()
blink_interval = 1  # seconds

# Send motor commands at a steady rate (independent of camera frame rate)
MOTOR_SEND_HZ = 50.0
motor_send_dt = 1.0 / MOTOR_SEND_HZ
last_motor_send = 0.0

# ------------------------
# MAIN LOOP
# ------------------------
try:
    while True:
        # ---- RECEIVE FRAME FROM PI ----
        while len(data) < payload_size:
            packet = client_socket.recv(4096)
            if not packet:
                raise ConnectionError("Pi disconnected")
            data += packet
        packed_size = data[:payload_size]
        data = data[payload_size:]
        frame_size = struct.unpack(">I", packed_size)[0]

        while len(data) < frame_size:
            packet = client_socket.recv(4096)
            if not packet:
                raise ConnectionError("Pi disconnected")
            data += packet
        frame_data = data[:frame_size]
        data = data[frame_size:]

        frame = cv2.imdecode(np.frombuffer(frame_data, dtype=np.uint8), cv2.IMREAD_COLOR)
        if frame is None:
            frame = np.full((window_height, window_width, 3), (200, 100, 100), dtype=np.uint8)

        # Center video on canvas
        canvas = np.zeros((window_height, window_width, 3), dtype=np.uint8)
        vid_h, vid_w = frame.shape[:2]
        x_offset = (window_width - vid_w) // 2
        y_offset = (window_height - vid_h) // 2
        canvas[y_offset:y_offset+vid_h, x_offset:x_offset+vid_w] = frame
        frame = canvas

        # ---- FAKE SENSOR UPDATE ----
        if time.time() - last_sensor_update > 0.5:
            fake_sensor = round(random.uniform(0, 100), 2)
            last_sensor_update = time.time()

        # ---- READ CONTROLLER STATE ----
        pygame.event.pump()
        lx = js.get_axis(0)
        ly = js.get_axis(1)
        rx = js.get_axis(3)
        ry = js.get_axis(4)
        l2 = js.get_axis(2)
        r2 = js.get_axis(5)
        hat_x, hat_y = js.get_hat(0)
        x_btn = js.get_button(0)
        circle = js.get_button(1)
        square = js.get_button(3)
        triangle = js.get_button(2)

        # ---- COMPUTE + SEND MOTOR COMMANDS (NEW) ----
        now = time.time()
        if now - last_motor_send >= motor_send_dt:
            m1, m2, m3, m4 = compute_motors(lx, ly, rx, ry, l2, r2, hat_x, hat_y)
            pkt = struct.pack(
                CMD_FMT,
                CMD_MAGIC,
                seq & 0xFFFFFFFF,
                throttle_to_i16(m1),
                throttle_to_i16(m2),
                throttle_to_i16(m3),
                throttle_to_i16(m4),
            )
            try:
                motor_sock.sendto(pkt, (MOTOR_UDP_IP, MOTOR_UDP_PORT))
            except Exception:
                pass
            seq += 1
            last_motor_send = now

        # ---- READ TELEMETRY FROM PI (NEW, optional) ----
        try:
            stat_data, _addr = motor_sock.recvfrom(2048)
            if len(stat_data) >= struct.calcsize(STAT_FMT):
                magic, stat_seq, age_ms, sm1, sm2, sm3, sm4, armed = struct.unpack(
                    STAT_FMT, stat_data[:struct.calcsize(STAT_FMT)]
                )
                if magic == STAT_MAGIC:
                    last_stat = (stat_seq, age_ms, sm1, sm2, sm3, sm4, armed)
                    last_stat_t = now
        except BlockingIOError:
            pass
        except Exception:
            pass

        # ---- MAP CONTROLLER TO COMMANDS (for on-screen debug list) ----
        active_commands = []
        if x_btn: active_commands.append("X Button")
        if circle: active_commands.append("Circle Button")
        if square: active_commands.append("Square Button")
        if triangle: active_commands.append("Triangle Button")
        if hat_x != 0: active_commands.append("DPad: Right" if hat_x > 0 else "DPad: Left")
        if hat_y != 0: active_commands.append("DPad: Up" if hat_y > 0 else "DPad: Down")
        if l2 > -1: active_commands.append("Left Trigger 2")
        if r2 > -1: active_commands.append("Right Trigger 2")
        if abs(lx) > 0.1: active_commands.append(f"Left Stick: {'Right' if lx > 0 else 'Left'}")
        if abs(ly) > 0.1: active_commands.append(f"Left Stick: {'Up' if ly < 0 else 'Down'}")
        if abs(rx) > 0.1: active_commands.append(f"Right Stick: {'Right' if rx > 0 else 'Left'}")
        if abs(ry) > 0.1: active_commands.append(f"Right Stick: {'Up' if ry < 0 else 'Down'}")
        if not active_commands: active_commands.append("No Command")

        # ---- OPENCV TEXT OVERLAY ----
        cv2.putText(frame, f"Depth Sensor: {fake_sensor}", (950, 30), font_small, scale_small, (0,0,255), thick_small, cv2.LINE_AA)
        cv2.putText(frame, "Meters", (1200, 30), font_small, scale_small, (0,0,255), thick_small, cv2.LINE_AA)
        for i, cmd in enumerate(active_commands):
            cv2.putText(frame, cmd, (20, 30 + i*25), font_small, scale_small, (0,255,255), thick_small, cv2.LINE_AA)

        # ---- Controller sticks and triggers display ----
        cv2.putText(frame, f"LX:{lx:.2f}", (20, 500), font_small, scale_small, (255,0,0), thick_small, cv2.LINE_AA)
        cv2.putText(frame, f"LY:{ly:.2f}", (20, 520), font_small, scale_small, (255,0,0), thick_small, cv2.LINE_AA)
        cv2.putText(frame, f"RX:{rx:.2f}", (150, 500), font_small, scale_small, (0,0,255), thick_small, cv2.LINE_AA)
        cv2.putText(frame, f"RY:{ry:.2f}", (150, 520), font_small, scale_small, (0,0,255), thick_small, cv2.LINE_AA)
        cv2.putText(frame, f"L2:{l2:.2f}", (20, 450), font_small, scale_small, (0,255,255), thick_small, cv2.LINE_AA)
        cv2.putText(frame, f"R2:{r2:.2f}", (150, 450), font_small, scale_small, (0,255,255), thick_small, cv2.LINE_AA)

        # ---- Motor command display (NEW) ----
        m1, m2, m3, m4 = compute_motors(lx, ly, rx, ry, l2, r2, hat_x, hat_y)
        cv2.putText(frame, f"M1:{m1:+.2f}  M2:{m2:+.2f}", (900, 680), font_small, scale_small, (255,255,255), thick_small, cv2.LINE_AA)
        cv2.putText(frame, f"M3:{m3:+.2f}  M4:{m4:+.2f}", (900, 705), font_small, scale_small, (255,255,255), thick_small, cv2.LINE_AA)

        # ---- Telemetry display (NEW) ----
        if last_stat is not None and (now - last_stat_t) < 1.0:
            stat_seq, age_ms, sm1, sm2, sm3, sm4, armed = last_stat
            cv2.putText(frame, f"Pi STAT seq:{stat_seq} age:{age_ms}ms armed:{armed}", (20, 680),
                        font_small, scale_small, (255,255,255), thick_small, cv2.LINE_AA)
        else:
            cv2.putText(frame, "Pi STAT: (no recent telemetry)", (20, 680),
                        font_small, scale_small, (200,200,200), thick_small, cv2.LINE_AA)

        # ---- VISUALIZE CONTROLLER INPUT WITH ARROWS ----
        arrow_len = 50
        arrow_thick = 3

        # D-Pad
        if hat_x > 0:
            cv2.arrowedLine(frame, (center_x, center_y), (center_x + arrow_len, center_y), (0,255,0), arrow_thick)
        elif hat_x < 0:
            cv2.arrowedLine(frame, (center_x, center_y), (center_x - arrow_len, center_y), (0,255,0), arrow_thick)
        if hat_y > 0:
            cv2.arrowedLine(frame, (center_x, center_y), (center_x, center_y - arrow_len), (0,255,0), arrow_thick)
        elif hat_y < 0:
            cv2.arrowedLine(frame, (center_x, center_y), (center_x, center_y + arrow_len), (0,255,0), arrow_thick)

        # Left stick
        cv2.arrowedLine(frame, (center_x, center_y),
                        (int(center_x + lx*arrow_len), int(center_y + ly*arrow_len)), (255,0,0), arrow_thick)
        # Right stick
        cv2.arrowedLine(frame, (center_x, center_y),
                        (int(center_x + rx*arrow_len), int(center_y + ry*arrow_len)), (0,0,255), arrow_thick)

        # ---- TRIGGERS WITH BORDERS ----
        max_width = 50
        height_top = center_y + 70
        height_bottom = center_y + 80

        # LEFT TRIGGER
        l2_width = int((l2 + 1) / 2 * max_width)
        right_x = center_x - 70
        left_x = right_x - l2_width
        cv2.rectangle(frame, (left_x, height_top), (right_x, height_bottom), (0, 255, 255), -1)
        cv2.rectangle(frame, (right_x - max_width, height_top), (right_x, height_bottom), (0, 100, 255), 1)

        # RIGHT TRIGGER
        r2_width = int((r2 + 1) / 2 * max_width)
        left_x_r = center_x + 70
        right_x_r = left_x_r + r2_width
        cv2.rectangle(frame, (left_x_r, height_top), (right_x_r, height_bottom), (0, 255, 255), -1)
        cv2.rectangle(frame, (left_x_r, height_top), (left_x_r + max_width, height_bottom), (0, 100, 255), 1)

        # ---- SHOW FRAME ----
        cv2.imshow("PS5 Controller Simulation", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

except Exception as e:
    print("Error:", e)

finally:
    # Best-effort: send neutral a few times on exit
    try:
        for _ in range(3):
            pkt = struct.pack(CMD_FMT, CMD_MAGIC, seq & 0xFFFFFFFF, 0, 0, 0, 0)
            motor_sock.sendto(pkt, (MOTOR_UDP_IP, MOTOR_UDP_PORT))
            time.sleep(0.02)
    except Exception:
        pass

    try:
        motor_sock.close()
    except Exception:
        pass

    client_socket.close()
    cv2.destroyAllWindows()
