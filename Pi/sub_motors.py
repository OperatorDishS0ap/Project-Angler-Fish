import serial
import time
import socket
import json

# ---------------- SERIAL ----------------
PORT = "/dev/serial0"
BAUDRATE = 420000

# ---------------- UDP ----------------
UDP_IP = "127.0.0.1"
UDP_PORT = 5005

# ---------------- CRSF ----------------
CRSF_ADDR_FC = 0xC8
CRSF_TYPE_RC_CHANNELS = 0x16

CRSF_MIN = 172
CRSF_MID = 992
CRSF_MAX = 1811

DEADBAND = 0.05

# ---------------- CRC ----------------
def crc8_dvb_s2(data):
    crc = 0
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = ((crc << 1) ^ 0xD5) if (crc & 0x80) else (crc << 1)
            crc &= 0xFF
    return crc

# ---------------- Scaling ----------------
def scale_axis(x):
    if abs(x) <= DEADBAND:
        return CRSF_MID
    x = max(-1.0, min(1.0, x))
    if x > 0:
        return int(CRSF_MID + x * (CRSF_MAX - CRSF_MID))
    else:
        return int(CRSF_MID + x * (CRSF_MID - CRSF_MIN))

def scale_throttle(t):
    t = max(0.0, min(1.0, t))
    return int(CRSF_MIN + t * (CRSF_MAX - CRSF_MIN))

def scale_arm(a):
    return CRSF_MAX if a else CRSF_MIN

# ---------------- CRSF PACKING ----------------
def pack_channels(ch):
    payload = bytearray()
    bitbuf = 0
    bitcount = 0
    for c in ch:
        bitbuf |= (c & 0x7FF) << bitcount
        bitcount += 11
        while bitcount >= 8:
            payload.append(bitbuf & 0xFF)
            bitbuf >>= 8
            bitcount -= 8
    return payload

def build_rc_packet(channels):
    payload = pack_channels(channels)
    length = len(payload) + 2
    frame = bytearray([
        CRSF_ADDR_FC,
        length,
        CRSF_TYPE_RC_CHANNELS
    ])
    frame.extend(payload)
    frame.append(crc8_dvb_s2(frame[2:]))
    return frame

# ---------------- SETUP ----------------
ser = serial.Serial(PORT, BAUDRATE, timeout=0)

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))
sock.setblocking(False)

# Default failsafe values
yaw = pitch = roll = 0.0
throttle = 0.0
arm = 0
last_rx = time.time()

channels = [CRSF_MID] * 16
channels[2] = CRSF_MIN

print("CRSF sender running...")

# ---------------- MAIN LOOP ----------------
try:
    while True:
        # ----- Receive UDP -----
        try:
            data, _ = sock.recvfrom(1024)
            cmd = json.loads(data.decode())

            yaw = cmd["yaw"]
            pitch = cmd["pitch"]
            roll = cmd["roll"]
            throttle = cmd["throttle"]
            arm = cmd["arm"]

            last_rx = time.time()

        except BlockingIOError:
            pass

        # ----- FAILSAFE -----
        if time.time() - last_rx > 0.25:
            arm = 0
            throttle = 0.0

        # ----- Channel mapping (AETR1234) -----
        channels[0] = scale_axis(roll)
        channels[1] = scale_axis(pitch)
        channels[2] = scale_throttle(throttle)
        channels[3] = scale_axis(yaw)
        channels[4] = scale_arm(arm)

        packet = build_rc_packet(channels)
        ser.write(packet)

        time.sleep(0.01)  # 100 Hz

except KeyboardInterrupt:
    ser.close()
    print("CRSF sender stopped")
