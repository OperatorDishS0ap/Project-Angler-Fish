import math
import os
import time
from mpu6050 import mpu6050


IMU_I2C_ADDR = int(os.environ.get("ANGLERFISH_IMU_I2C_ADDR", "0x68"), 16)
IMU_INIT_RETRIES = int(os.environ.get("ANGLERFISH_IMU_INIT_RETRIES", "5"))
IMU_RETRY_DELAY_S = float(os.environ.get("ANGLERFISH_IMU_RETRY_DELAY_S", "0.5"))
RAD_TO_DEG = 57.2957795


def init_imu_with_retry(address: int = IMU_I2C_ADDR, retries: int = IMU_INIT_RETRIES, delay_s: float = IMU_RETRY_DELAY_S):
    for attempt in range(1, retries + 1):
        try:
            imu_dev = mpu6050(address)
            print(f"[mpu6050] initialized at 0x{address:02X}")
            return imu_dev
        except OSError as exc:
            print(f"[mpu6050] init failed (attempt {attempt}/{retries}): {exc}")
            if attempt < retries:
                time.sleep(delay_s)
    print("[mpu6050] continuing without MPU6050")
    return None


def calibrate_gyro(imu_dev, samples: int = 200, delay_s: float = 0.005):
    if imu_dev is None:
        return {"x": 0.0, "y": 0.0, "z": 0.0}

    sum_x = 0.0
    sum_y = 0.0
    sum_z = 0.0

    for _ in range(max(1, int(samples))):
        gyro = imu_dev.get_gyro_data()
        sum_x += float(gyro.get("x", 0.0))
        sum_y += float(gyro.get("y", 0.0))
        sum_z += float(gyro.get("z", 0.0))
        if delay_s > 0.0:
            time.sleep(delay_s)

    sample_count = float(max(1, int(samples)))
    return {
        "x": sum_x / sample_count,
        "y": sum_y / sample_count,
        "z": sum_z / sample_count,
    }


def accel_to_pitch_roll(accel):
    ax = float(accel.get("x", 0.0))
    ay = float(accel.get("y", 0.0))
    az = float(accel.get("z", 0.0))
    pitch_deg = math.atan2(az, math.sqrt((ax * ax) + (ay * ay))) * RAD_TO_DEG
    roll_deg = math.atan2(ax, ay) * RAD_TO_DEG
    return pitch_deg, roll_deg


def read_imu(imu_dev):
    if imu_dev is None:
        return {"x": 0.0, "y": 0.0, "z": 9.8}, {"x": 0.0, "y": 0.0, "z": 0.0}, 0.0
    accel = imu_dev.get_accel_data()
    gyro = imu_dev.get_gyro_data()
    imu_temp = imu_dev.get_temp()
    return accel, gyro, imu_temp
