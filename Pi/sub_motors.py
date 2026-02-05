import socket
import struct
import serial
import time

# =============================
# UDP CONFIG
# =============================
UDP_PORT = 9000
CMD_FMT = "<4sI5h"
CMD_MAGIC = b"SUB1"

# =============================
# CRSF CONFIG
# =============================
CRSF_ADDR_FLIGHT_CONTROLLER = 0xC8
CRSF_TYPE_RC_CHANNELS_PACKED = 0x16
CRSF_FRAME_SIZE = 24
CRSF_BAUD = 420000
CRSF_UART = "/dev/serial0"

# CRSF channel limits
CRSF_MIN = 172
CRSF_MID = 992
CRSF_MAX = 1811

# =============================
# HELPERS
# =============================
def crc8(data):
    crc = 0
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = (crc << 1) ^ 0xD5 if (crc & 0x80) else (crc << 1)
            crc &= 0xFF
    return crc

def map_channel(x, in_min, in_max):
    return int((x - in_min) * (CRSF_MAX - CRSF_MIN) / (in_max - in_min) + CRSF_MIN)

def norm_axis(v):
    return max(-1.0, min(1.0, v))

# =============================
# CRSF PACKET BUILDER
# =============================
def build_crsf_packet(throttle, yaw, pitch, roll, arm):
    channels = [CRSF_MID] * 16

    channels[0] = map_channel(norm_axis(roll),  -1, 1)
    channels[1] = map_channel(norm_axis(pitch), -1, 1)
    channels[2] = map_channel(norm_axis(throttle), 0, 1)
    channels[3] = map_channel(norm_axis(yaw),   -1, 1)
    channels[4] = CRSF_MAX if arm else CRSF_MIN

    payload = bytearray()
    bits = 0
    bitlen = 0

    for ch in channels:
        bits |= ch << bitlen
        bitlen += 11
        while bitlen >= 8:
            payload.append(bits & 0xFF)
            bits >>= 8
            bitlen -= 8

    frame = bytearray([
        CRSF_ADDR_FLIGHT_CONTROLLER,
        CRSF_FRAME_SIZE,
        CRSF_TYPE_RC_CHANNELS_PACKED,
    ]) + payload[:22]

    frame.append(crc8(frame[2:]))
    return frame

# =============================
# MAIN
# =============================
def main():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("", UDP_PORT))
    sock.setblocking(False)

    ser = serial.Serial(CRSF_UART, CRSF_BAUD, timeout=0)

    throttle = yaw = pitch = roll = 0.0
    arm = 0

    print("[sub_motors] Listening on UDP port", UDP_PORT)

    while True:
        try:
            data, _ = sock.recvfrom(1024)
            magic, seq, t_i, y_i, p_i, r_i, a_i = struct.unpack(CMD_FMT, data)

            if magic != CMD_MAGIC:
                continue

            throttle = max(0.0, min(1.0, t_i / 1000.0))
            yaw = max(-1.0, min(1.0, y_i / 1000.0))
            pitch = max(-1.0, min(1.0, p_i / 1000.0))
            roll = max(-1.0, min(1.0, r_i / 1000.0))
            arm = 1 if a_i > 0 else 0

        except BlockingIOError:
            pass

        pkt = build_crsf_packet(throttle, yaw, pitch, roll, arm)
        ser.write(pkt)
        time.sleep(0.01)  # ~100 Hz

if __name__ == "__main__":
    main()
