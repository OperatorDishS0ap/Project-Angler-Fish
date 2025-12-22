#!/usr/bin/env python3
import json
import os
import random
import socket
import time


# Send telemetry to the PC (fill in PC_IP or auto-detect from first control packet if desired)
PC_IP = os.environ.get("ANGLERFISH_PC_IP", "192.168.1.78")  # <-- set to your PC ethernet IP
PC_PORT = 9100

RATE_HZ = 5.0


def read_pi_temp_c() -> float:
    # Works on Raspberry Pi OS
    try:
        with open("/sys/class/thermal/thermal_zone0/temp", "r", encoding="utf-8") as f:
            return float(f.read().strip()) / 1000.0
    except Exception:
        return 0.0


def main():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    t0 = time.time()
    while True:
        # TODO: Replace these with real sensors
        temp_pi = read_pi_temp_c()
        temp_env = temp_pi - 3.0  # placeholder
        battery = 12.0 + 0.2 * random.random()  # placeholder
        depth = 0.0 + 0.1 * random.random()     # placeholder
        pressure = 1.0 + 0.05 * random.random() # placeholder

        msg = {
            "ts": time.time(),
            "battery": battery,
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
