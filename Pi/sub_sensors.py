#!/usr/bin/env python3
import json
import os
import random
import socket
import time

PC_IP = os.environ.get("ANGLERFISH_PC_IP", "192.168.137.1")  # set to your Windows ethernet IP
PC_PORT = 9100
RATE_HZ = 5.0

def read_pi_temp_c() -> float:
    try:
        with open("/sys/class/thermal/thermal_zone0/temp", "r", encoding="utf-8") as f:
            return float(f.read().strip()) / 1000.0
    except Exception:
        return 0.0

def main():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    while True:
        temp_pi = read_pi_temp_c()
        msg = {
            "ts": time.time(),
            "battery": 12.0 + 0.2 * random.random(),
            "depth": 0.0 + 0.1 * random.random(),
            "pressure": 1.0 + 0.05 * random.random(),
            "temp_pi": temp_pi,
            "temp_env": temp_pi - 3.0,
        }
        try:
            sock.sendto(json.dumps(msg).encode("utf-8"), (PC_IP, PC_PORT))
        except Exception:
            pass
        time.sleep(1.0 / max(1.0, RATE_HZ))

if __name__ == "__main__":
    main()
