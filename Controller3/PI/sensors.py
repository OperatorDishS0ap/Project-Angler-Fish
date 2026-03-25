#!/usr/bin/env python3
import json
import os
import random
import socket
import time
import math
import ms5837
from mpu6050 import mpu6050

IMU_I2C_ADDR = int(os.environ.get("ANGLERFISH_IMU_I2C_ADDR", "0x68"), 16)
IMU_INIT_RETRIES = int(os.environ.get("ANGLERFISH_IMU_INIT_RETRIES", "5"))
IMU_RETRY_DELAY_S = float(os.environ.get("ANGLERFISH_IMU_RETRY_DELAY_S", "0.5"))

PC_IP = os.environ.get("ANGLERFISH_PC_IP", "192.168.137.1")  # set to your Windows ethernet IP
PC_PORT = int(os.environ.get("ANGLERFISH_PC_PORT", "9001"))
RATE_HZ = 30.0

# Minimum deltas required before publishing a new averaged telemetry packet.
TEMP_DELTA_C = float(os.environ.get("ANGLERFISH_TEMP_DELTA_C", "0.1"))
VOLT_DELTA_V = float(os.environ.get("ANGLERFISH_VOLT_DELTA_V", "0.1"))
PRESSURE_DELTA_PSI = float(os.environ.get("ANGLERFISH_PRESSURE_DELTA_PSI", "0.05"))
DEPTH_DELTA_M = float(os.environ.get("ANGLERFISH_DEPTH_DELTA_M", "0.05"))

def read_pi_temp_c() -> float:
    try:
        with open("/sys/class/thermal/thermal_zone0/temp", "r", encoding="utf-8") as f:
            return float(f.read().strip()) / 1000.0
    except Exception:
        return 0.0

def bar30(sensor_dev):
    pressure = sensor_dev.pressure(ms5837.UNITS_psi)
    temperature = sensor_dev.temperature(ms5837.UNITS_Centigrade)
    depth = sensor_dev.depth()
    return pressure, temperature, depth


def init_bar30():
    sensor_dev = ms5837.MS5837_30BA()
    if not sensor_dev.init():
        print("[sensors] BAR30 init failed; continuing without pressure/depth")
        return None
    if not sensor_dev.read():
        print("[sensors] BAR30 first read failed; continuing without pressure/depth")
        return None
    return sensor_dev


def init_imu_with_retry(address: int, retries: int, delay_s: float):
    for attempt in range(1, retries + 1):
        try:
            imu_dev = mpu6050(address)
            print(f"[sensors] MPU6050 initialized at 0x{address:02X}")
            return imu_dev
        except OSError as exc:
            print(f"[sensors] MPU6050 init failed (attempt {attempt}/{retries}): {exc}")
            if attempt < retries:
                time.sleep(delay_s)
    print("[sensors] Continuing without MPU6050; acceleration/imu temp will be zeroed")
    return None


def _avg(values):
    return sum(values) / len(values) if values else 0.0


def _changed_enough(current, previous):
    if previous is None:
        return True

    return (
        abs(current["temp_pi"] - previous["temp_pi"]) >= TEMP_DELTA_C
        or abs(current["temp_env"] - previous["temp_env"]) >= TEMP_DELTA_C
        or abs(current["battery"] - previous["battery"]) >= VOLT_DELTA_V
        or abs(current["pressure"] - previous["pressure"]) >= PRESSURE_DELTA_PSI
        or abs(current["depth"] - previous["depth"]) >= DEPTH_DELTA_M
    )



def main():
    sensor = init_bar30()
    imu = init_imu_with_retry(IMU_I2C_ADDR, IMU_INIT_RETRIES, IMU_RETRY_DELAY_S)
    if sensor is None and imu is None:
        raise SystemExit("No sensors initialized (BAR30 and MPU6050 unavailable)")

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    dt = 1.0 / max(1.0, RATE_HZ)
    speed = 0.0  # Integrated speed from acceleration
    GRAVITY = 9.8  # m/s²

    pressure = 0.0
    temp_env = 0.0
    depth = 0.0

    # 1-second averaging buckets for outbound telemetry values.
    samples = {
        "battery": [],
        "pressure": [],
        "depth": [],
        "temp_pi": [],
        "temp_env": [],
        "temp_enclosure": [],
        "speed": [],
        "acceleration": [],
    }
    window_start = time.time()
    last_sent_values = None
    last_imu_reconnect_try = 0.0
    
    while True:
        temp_pi = read_pi_temp_c()
        if sensor is not None and sensor.read():
            pressure, temp_env, depth = bar30(sensor)

        # Read IMU data
        accel = {"x": 0.0, "y": 0.0, "z": GRAVITY}
        imu_temp = 0.0
        if imu is not None:
            try:
                accel = imu.get_accel_data()
                imu_temp = imu.get_temp()
            except OSError as exc:
                print(f"[sensors] MPU6050 read failed: {exc}")
                imu = None

        if imu is None and (time.time() - last_imu_reconnect_try) >= 2.0:
            last_imu_reconnect_try = time.time()
            imu = init_imu_with_retry(IMU_I2C_ADDR, 1, 0.0)
        
        # Calculate acceleration magnitude
        accel_magnitude = math.sqrt(accel['x']**2 + accel['y']**2 + accel['z']**2)
        
        # Account for gravity - subtract gravitational component
        # When stationary, accel_magnitude ~= 9.8, so motion_accel ~= 0
        motion_accel = max(0, accel_magnitude - GRAVITY)
        
        # Integrate acceleration to estimate speed
        speed += motion_accel * dt
        
        # Apply exponential decay to prevent drift
        # Time constant of ~5 seconds for decay
        decay_factor = 0.96
        speed *= decay_factor
        
        # Clamp speed to prevent unrealistic values
        speed = max(0, speed)

        Debug = True
        if Debug:
            print(f"Pressure: {pressure:.2f} psi, Temp (Pi): {temp_pi:.2f} C, Temp (Env): {temp_env:.2f} C, Depth: {depth:.3f} m")
            print(f"Speed: {speed:.2f} m/s, Accel: {motion_accel:.2f} m/s²")

        # Collect samples at RATE_HZ and publish averaged values once per second.
        samples["battery"].append(12.0 + 0.2 * random.random())
        samples["pressure"].append(pressure)
        samples["depth"].append(depth)
        samples["temp_pi"].append(temp_pi)
        samples["temp_env"].append(temp_env)
        samples["temp_enclosure"].append(imu_temp)
        samples["speed"].append(speed)
        samples["acceleration"].append(motion_accel)

        now = time.time()
        if now - window_start >= 1.0:
            avg_msg = {
                "ts": now,
                "type": "telemetry",
                "telemetry": {
                    "battery_v": round(_avg(samples["battery"]), 3),
                    "depth_m": round(_avg(samples["depth"]), 4),
                    "pressure_bar": round(_avg(samples["pressure"]), 3),
                    "pi_temp_c": round(_avg(samples["temp_pi"]), 3),
                    "water_temp_c": round(_avg(samples["temp_env"]), 3),
                    "enclosure_temp_c": round(_avg(samples["temp_enclosure"]), 3),
                    "speed_mps": round(_avg(samples["speed"]), 4),
                    "accel_mps2": round(_avg(samples["acceleration"]), 4),
                },
            }

            compare_values = {
                "temp_pi": avg_msg["telemetry"]["pi_temp_c"],
                "temp_env": avg_msg["telemetry"]["water_temp_c"],
                "battery": avg_msg["telemetry"]["battery_v"],
                "pressure": avg_msg["telemetry"]["pressure_bar"],
                "depth": avg_msg["telemetry"]["depth_m"],
            }

            # Only publish when monitored averaged values exceed threshold deltas.
            if _changed_enough(compare_values, last_sent_values):
                try:
                    sock.sendto(json.dumps(avg_msg).encode("utf-8"), (PC_IP, PC_PORT))
                    last_sent_values = compare_values
                except Exception:
                    pass

            for values in samples.values():
                values.clear()
            window_start = now

        time.sleep(dt)

if __name__ == "__main__":
    main()
