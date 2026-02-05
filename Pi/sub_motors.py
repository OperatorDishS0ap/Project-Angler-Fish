import serial
import time

PORT = "/dev/serial0"
BAUDRATE = 420000

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
    """Scale -1..1 with deadband to CRSF"""
    if abs(x) <= DEADBAND:
        return CRSF_MID

    x = max(-1.0, min(1.0, x))
    if x > 0:
        return int(CRSF_MID + x * (CRSF_MAX - CRSF_MID))
    else:
        return int(CRSF_MID + x * (CRSF_MID - CRSF_MIN))


def scale_throttle(t):
    """Scale 0..1 to CRSF"""
    t = max(0.0, min(1.0, t))
    return int(CRSF_MIN + t * (CRSF_MAX - CRSF_MIN))


def scale_arm(a):
    return CRSF_MAX if a else CRSF_MIN


# ---------------- CRSF packing ----------------
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
    length = len(payload) + 2  # type + CRC
    frame = bytearray([
        CRSF_ADDR_FC,
        length,
        CRSF_TYPE_RC_CHANNELS
    ])
    frame.extend(payload)
    frame.append(crc8_dvb_s2(frame[2:]))
    return frame


# ---------------- Main ----------------
ser = serial.Serial(PORT, BAUDRATE, timeout=0)

# Initialize channels
channels = [CRSF_MID] * 16
channels[2] = CRSF_MIN  # throttle low by default

try:
    while True:
        # ---- YOUR VARIABLES ----
        yaw = 0.0        # -1.0 .. 1.0
        pitch = 0.0      # -1.0 .. 1.0
        roll = 0.0       # -1.0 .. 1.0
        throttle = 0.0   # 0.0 .. 1.0
        arm = 0          # 0 or 1
        # ------------------------

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

