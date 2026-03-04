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

def calculate_pitch_roll_yaw(acc, gyro):
    """Calculate pitch and roll from accelerometer, yaw rate from gyroscope."""
    ax, ay, az = acc['x'], acc['y'], acc['z']
    gx, gy, gz = gyro['x'], gyro['y'], gyro['z']
    
    # Calculate pitch and roll from accelerometer
    # Pitch: rotation around Y axis
    pitch = math.atan2(ax, math.sqrt(ay**2 + az**2)) * 180 / math.pi
    # Roll: rotation around X axis
    roll = math.atan2(ay, math.sqrt(ax**2 + az**2)) * 180 / math.pi
    
    # Angular velocity (gyro readings in deg/s)
    angular_vel = math.sqrt(gx**2 + gy**2 + gz**2)
    
    # Yaw rate (rotation around Z axis)
    yaw_rate = gz
    
    return pitch, roll, angular_vel, yaw_rate

def main():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    while True:
        temp_pi = read_pi_temp_c()
        if sensor.read():
            pressure, temp_env, depth = bar30()

        # Read IMU data
        accel = imu.get_accel_data()
        gyro = imu.get_gyro_data()
        imu_temp = imu.get_temp()
        pitch, roll, angular_vel, yaw_rate = calculate_pitch_roll_yaw(accel, gyro)
        
        # Calculate acceleration magnitude
        acceleration = math.sqrt(accel['x']**2 + accel['y']**2 + accel['z']**2)

        Debug = True
        if Debug:
            print(f"Pressure: {pressure:.2f} psi, Temp (Pi): {temp_pi:.2f} C, Temp (Env): {temp_env:.2f} C, Depth: {depth:.3f} m")
            print(f"Pitch: {pitch:.2f}°, Roll: {roll:.2f}°, Yaw Rate: {yaw_rate:.2f}°/s, Angular Vel: {angular_vel:.2f}°/s, Accel: {acceleration:.2f} m/s²")

        msg = {
            "ts": time.time(),
            "battery": 12.0 + 0.2 * random.random(),
            "depth": depth,
            "pressure": pressure,
            "temp_pi": temp_pi,
            "temp_env": temp_env,
            "temp_enclosure": imu_temp,
            "pitch": pitch,
            "roll": roll,
            "yaw_rate": yaw_rate,
            "angular_vel": angular_vel,
            "acceleration": acceleration,
        }
        try:
            sock.sendto(json.dumps(msg).encode("utf-8"), (PC_IP, PC_PORT))
        except Exception:
            pass
        time.sleep(1.0 / max(1.0, RATE_HZ))

if __name__ == "__main__":
    main()
