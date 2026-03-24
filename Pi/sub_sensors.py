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
imu = mpu6050(0x68)  #MPU6050 I2C address

if not sensor.init():
    print("Sensor could not be initialized")
    exit(1)
if not sensor.read():
    print("Sensor read failed!")
    exit(1)

PC_HOST = os.environ.get("ANGLERFISH_PC_HOST", "anglerfish.local")  # PC hostname or IP (IPv6/IPv4)
PC_PORT = 9100
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

def bar30():
    pressure = sensor.pressure(ms5837.UNITS_psi)
    temperature = sensor.temperature(ms5837.UNITS_Centigrade)
    depth = sensor.depth()
    return pressure, temperature, depth


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


def _resolve_udp_endpoint(host: str, port: int):
    infos = socket.getaddrinfo(host, port, type=socket.SOCK_DGRAM)
    if not infos:
        raise RuntimeError(f"Unable to resolve host '{host}'")
    infos.sort(key=lambda i: 0 if i[0] == socket.AF_INET6 else 1)
    family, _socktype, _proto, _canonname, sockaddr = infos[0]
    return family, sockaddr



def main():
    family, pc_addr = _resolve_udp_endpoint(PC_HOST, PC_PORT)
    sock = socket.socket(family, socket.SOCK_DGRAM)
    dt = 1.0 / max(1.0, RATE_HZ)
    speed = 0.0  # Integrated speed from acceleration
    GRAVITY = 9.8  # m/s²

    print(f"[sub_sensors] Sending telemetry to {PC_HOST}:{PC_PORT}")

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
                "battery": round(_avg(samples["battery"]), 3),
                "depth": round(_avg(samples["depth"]), 4),
                "pressure": round(_avg(samples["pressure"]), 3),
                "temp_pi": round(_avg(samples["temp_pi"]), 3),
                "temp_env": round(_avg(samples["temp_env"]), 3),
                "temp_enclosure": round(_avg(samples["temp_enclosure"]), 3),
                "speed": round(_avg(samples["speed"]), 4),
                "acceleration": round(_avg(samples["acceleration"]), 4),
            }

            compare_values = {
                "temp_pi": avg_msg["temp_pi"],
                "temp_env": avg_msg["temp_env"],
                "battery": avg_msg["battery"],
                "pressure": avg_msg["pressure"],
                "depth": avg_msg["depth"],
            }

            # Only publish when monitored averaged values exceed threshold deltas.
            if _changed_enough(compare_values, last_sent_values):
                try:
                    sock.sendto(json.dumps(avg_msg).encode("utf-8"), pc_addr)
                    last_sent_values = compare_values
                except Exception:
                    pass

            for values in samples.values():
                values.clear()
            window_start = now

        time.sleep(dt)

if __name__ == "__main__":
    main()
