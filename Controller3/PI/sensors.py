#!/usr/bin/env python3
import json
import os
import random
import socket
import time
import math
from bar30_sensor import init_bar30, read_bar30
from mpu6050_sensor import init_imu_with_retry, read_imu

USE_BROADCAST = os.environ.get("ANGLERFISH_USE_BROADCAST", "1") == "1"
BROADCAST_IP = os.environ.get("ANGLERFISH_BROADCAST_IP", "255.255.255.255")
PC_IP = os.environ.get("ANGLERFISH_PC_IP", "192.168.137.1")  # used when ANGLERFISH_USE_BROADCAST=0
PC_PORT = int(os.environ.get("ANGLERFISH_PC_PORT", "9001"))
RATE_HZ = 30.0
ENABLE_BAR30 = os.environ.get("ANGLERFISH_ENABLE_BAR30", "1") == "0"
ENABLE_MPU6050 = os.environ.get("ANGLERFISH_ENABLE_MPU6050", "1") == "0"
DEBUG = os.environ.get("ANGLERFISH_SENSOR_DEBUG", "1") == "1"

# Minimum deltas required before publishing a new averaged telemetry packet.
TEMP_DELTA_C = float(os.environ.get("ANGLERFISH_TEMP_DELTA_C", "0.1"))
VOLT_DELTA_V = float(os.environ.get("ANGLERFISH_VOLT_DELTA_V", "0.1"))
PRESSURE_DELTA_PSI = float(os.environ.get("ANGLERFISH_PRESSURE_DELTA_PSI", "0.05"))
DEPTH_DELTA_M = float(os.environ.get("ANGLERFISH_DEPTH_DELTA_M", "0.05"))

def _avg(values):
    return sum(values) / len(values) if values else 0.0


def _changed_enough(current, previous):
    if previous is None:
        return True

    return (
        abs(current["temp_env"] - previous["temp_env"]) >= TEMP_DELTA_C
        or abs(current["battery"] - previous["battery"]) >= VOLT_DELTA_V
        or abs(current["pressure"] - previous["pressure"]) >= PRESSURE_DELTA_PSI
        or abs(current["depth"] - previous["depth"]) >= DEPTH_DELTA_M
    )



def main():
    sensor = init_bar30() if ENABLE_BAR30 else None
    imu = init_imu_with_retry() if ENABLE_MPU6050 else None

    print(
        f"[sensors] Sensor toggles: BAR30={'ON' if ENABLE_BAR30 else 'OFF'} "
        f"MPU6050={'ON' if ENABLE_MPU6050 else 'OFF'}"
    )

    if ENABLE_BAR30 and sensor is None:
        print("[sensors] BAR30 enabled but unavailable; publishing zeros for BAR30 fields")
    if ENABLE_MPU6050 and imu is None:
        print("[sensors] MPU6050 enabled but unavailable; publishing zeros for MPU fields")

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    if USE_BROADCAST:
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

    target_ip = BROADCAST_IP if USE_BROADCAST else PC_IP
    print(f"[sensors] Telemetry target: {target_ip}:{PC_PORT} (broadcast={USE_BROADCAST})")

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
        "temp_env": [],
        "temp_enclosure": [],
        "speed": [],
        "acceleration": [],
    }
    window_start = time.time()
    last_sent_values = None
    last_imu_reconnect_try = 0.0
    
    while True:
        if ENABLE_BAR30:
            pressure, temp_env, depth = read_bar30(sensor)
        else:
            pressure, temp_env, depth = 0.0, 0.0, 0.0

        # Read IMU data
        accel = {"x": 0.0, "y": 0.0, "z": GRAVITY}
        imu_temp = 0.0
        if imu is not None:
            try:
                accel, imu_temp = read_imu(imu)
            except OSError as exc:
                print(f"[sensors] MPU6050 read failed: {exc}")
                imu = None

        if ENABLE_MPU6050 and imu is None and (time.time() - last_imu_reconnect_try) >= 2.0:
            last_imu_reconnect_try = time.time()
            imu = init_imu_with_retry(retries=1, delay_s=0.0)
        
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

        if DEBUG:
            print(f"Pressure: {pressure:.2f} psi, Temp (Env): {temp_env:.2f} C, Depth: {depth:.3f} m")
            print(f"Speed: {speed:.2f} m/s, Accel: {motion_accel:.2f} m/s²")

        # Collect samples at RATE_HZ and publish averaged values once per second.
        samples["battery"].append(12.0 + 0.2 * random.random())
        samples["pressure"].append(pressure)
        samples["depth"].append(depth)
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
                    "water_temp_c": round(_avg(samples["temp_env"]), 3),
                    "enclosure_temp_c": round(_avg(samples["temp_enclosure"]), 3),
                    "speed_mps": round(_avg(samples["speed"]), 4),
                    "accel_mps2": round(_avg(samples["acceleration"]), 4),
                },
            }

            compare_values = {
                "temp_env": avg_msg["telemetry"]["water_temp_c"],
                "battery": avg_msg["telemetry"]["battery_v"],
                "pressure": avg_msg["telemetry"]["pressure_bar"],
                "depth": avg_msg["telemetry"]["depth_m"],
            }

            # Only publish when monitored averaged values exceed threshold deltas.
            if _changed_enough(compare_values, last_sent_values):
                try:
                    sock.sendto(json.dumps(avg_msg).encode("utf-8"), (target_ip, PC_PORT))
                    last_sent_values = compare_values
                except Exception:
                    pass

            for values in samples.values():
                values.clear()
            window_start = now

        time.sleep(dt)

if __name__ == "__main__":
    main()
