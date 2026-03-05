#!/usr/bin/env python3
import json
import os
import random
import socket
import time
import math
import ms5837
from mpu6050 import mpu6050

sensor = ms5837.MS5837_30BA()
imu = mpu6050(0x68)  # MPU6050 I2C address

if not sensor.init():
    print("Sensor could not be initialized")
    exit(1)
if not sensor.read():
    print("Sensor read failed!")
    exit(1)

PC_IP = os.environ.get("ANGLERFISH_PC_IP", "192.168.137.1")  # set to your Windows ethernet IP
PC_PORT = 9100
RATE_HZ = 5.0

def read_pi_temp_c() -> float:
    try:
        with open("/sys/class/thermal/thermal_zone0/temp", "r", encoding="utf-8") as f:
            return float(f.read().strip()) / 1000.0
    except Exception:
        return 0.0

def bar30():
    pressure = sensor.pressure(ms5837.UNITS_psi)
    temperature = sensor.temperature(ms5837.UNITS_Centigrade)
    depth = sensor.depth()
    return pressure, temperature, depth



def main():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    dt = 1.0 / max(1.0, RATE_HZ)
    speed = 0.0  # Integrated speed from acceleration
    GRAVITY = 9.8  # m/s²
    
    while True:
        temp_pi = read_pi_temp_c()
        if sensor.read():
            pressure, temp_env, depth = bar30()

        # Read IMU data
        accel = imu.get_accel_data()
        imu_temp = imu.get_temp()
        
        # Calculate acceleration magnitude
        accel_magnitude = math.sqrt(accel['x']**2 + accel['y']**2 + accel['z']**2)
        
        # Account for gravity - subtract gravitational component
        # When stationary, accel_magnitude ~= 9.8, so motion_accel ~= 0
        motion_accel = max(0, accel_magnitude - GRAVITY)
        
        # Integrate acceleration to estimate speed
        speed += motion_accel * dt
        
        # Apply exponential decay to prevent drift
        # Time constant of ~5 seconds for decay
        decay_factor = 0.97
        speed *= decay_factor
        
        # Clamp speed to prevent unrealistic values
        speed = max(0, speed)

        Debug = True
        if Debug:
            print(f"Pressure: {pressure:.2f} psi, Temp (Pi): {temp_pi:.2f} C, Temp (Env): {temp_env:.2f} C, Depth: {depth:.3f} m")
            print(f"Speed: {speed:.2f} m/s, Accel: {motion_accel:.2f} m/s²")

        msg = {
            "ts": time.time(),
            "battery": 12.0 + 0.2 * random.random(),
            "depth": depth,
            "pressure": pressure,
            "temp_pi": temp_pi,
            "temp_env": temp_env,
            "temp_enclosure": imu_temp,
            "speed": speed,
            "acceleration": motion_accel,
        }
        try:
            sock.sendto(json.dumps(msg).encode("utf-8"), (PC_IP, PC_PORT))
        except Exception:
            pass
        time.sleep(dt)

if __name__ == "__main__":
    main()
