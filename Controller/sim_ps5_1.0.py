import cv2
import socket
import struct
import numpy as np
import time
import random
import pygame
import subprocess  # <-- NEW: for SSH to Pi
import paramiko

# ------------------------
# SSH SETTINGS TO START LAUNCHER ON PI
# ------------------------
PI_SSH_HOST = "10.42.0.50"      # Pi's Ethernet IP (same as HOST below)
PI_SSH_USER = "pi"              # Pi username
PI_SSH_PASSWORD = "raspberry"   # <--- CHANGE THIS
PI_COMMAND = "nohup python3 /home/pi/sub_launcher.py > launcher.log 2>&1 &"

def start_launcher_on_pi():
    """
    Connect to Pi using SSH (with password) and run the launcher command.
    """
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

        # Run launcher command on Pi
        stdin, stdout, stderr = client.exec_command(PI_COMMAND)

        # Collect logs
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

# If you want the launcher to keep running in background on the Pi and NOT block this script,
# you could instead use:
# PI_COMMAND = "nohup python3 /home/pi/projects/main_launcher.py > launcher.log 2>&1 &"

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
# START LAUNCHER ON PI (NEW)
# ------------------------
start_launcher_on_pi()

# ------------------------
# WAIT BEFORE CONNECTING TO CAMERA  (ADD THIS)
# ------------------------
startup_delay = 5   # seconds — increase if needed
print(f"Waiting {startup_delay} seconds for Pi camera server to start...")
time.sleep(startup_delay)

# ------------------------
# CONNECT TO PI TCP CAMERA (with retry)
# ------------------------
client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
print(f"Connecting to Pi at {HOST}:{PORT} ...")

# Try a few times in case the Pi is still starting the camera script
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
            # Fallback if frame decoding fails
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

        # ---- MAP CONTROLLER TO COMMANDS ----
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
    client_socket.close()
    cv2.destroyAllWindows()
