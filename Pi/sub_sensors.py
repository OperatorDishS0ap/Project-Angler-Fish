#!/usr/bin/env python3
import json
import os
import random
import socket
import time
import ms5837

sensor = ms5837.MS5837_30BA()

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
    while True:
        temp_pi = read_pi_temp_c()
        pressure, temp_env, depth = bar30()

        Debug = True
        if Debug:
            print(f"Pressure: {pressure:.2f} psi, Temp (Pi): {temp_pi:.2f} C, Temp (Env): {temp_env:.2f} C, Depth: {depth:.3f} m")

        msg = {
            "ts": time.time(),
            "battery": 12.0 + 0.2 * random.random(),
            "depth": depth,
            "pressure": pressure,
            "temp_pi": temp_pi,
            "temp_env": temp_env,
        }
        try:
            sock.sendto(json.dumps(msg).encode("utf-8"), (PC_IP, PC_PORT))
        except Exception:
            pass
        time.sleep(1.0 / max(1.0, RATE_HZ))

if __name__ == "__main__":
    main()
