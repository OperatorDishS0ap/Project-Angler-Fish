import os
import time
from mpu6050 import mpu6050


IMU_I2C_ADDR = int(os.environ.get("ANGLERFISH_IMU_I2C_ADDR", "0x68"), 16)
IMU_INIT_RETRIES = int(os.environ.get("ANGLERFISH_IMU_INIT_RETRIES", "5"))
IMU_RETRY_DELAY_S = float(os.environ.get("ANGLERFISH_IMU_RETRY_DELAY_S", "0.5"))


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


def read_imu(imu_dev):
    if imu_dev is None:
        return {"x": 0.0, "y": 0.0, "z": 9.8}, 0.0
    accel = imu_dev.get_accel_data()
    imu_temp = imu_dev.get_temp()
    return accel, imu_temp
