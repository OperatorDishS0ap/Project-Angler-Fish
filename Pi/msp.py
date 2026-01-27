import serial
import struct
import time

SERIAL_PORT = "/dev/serial0"   # or /dev/serial0
BAUD = 115200

ser = serial.Serial(SERIAL_PORT, BAUD, timeout=1)

def msp_motor_command(m1, m2, m3, m4):
    # MSP_SET_MOTOR = 214
    motors = [m1, m2, m3, m4]
    payload = b''.join(struct.pack('<H', m) for m in motors)
    
    size = len(payload)
    cmd = 214
    
    checksum = size ^ cmd
    for b in payload:
        checksum ^= b
    
    packet = b'$M<' + bytes([size, cmd]) + payload + bytes([checksum])
    ser.write(packet)

# Example: spin motors independently
while True:
    msp_motor_command(1460, 1460, 1460, 1460)
    time.sleep(1)
    msp_motor_command(1460, 1350, 1460, 1460)
    time.sleep(1)
    msp_motor_command(1460, 1460, 1350, 1460)
    time.sleep(1)
    msp_motor_command(1460, 1460, 1460, 1350)
    time.sleep(1)
    msp_motor_command(1350, 1460, 1460, 1460)
    time.sleep(1)
