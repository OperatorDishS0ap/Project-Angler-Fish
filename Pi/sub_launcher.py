## Main Program to run and launch the necissary programs for the submarine
import subprocess
import time
from pathlib import Path

BASE_DIR = Path(__file__).resolve().parent

scripts = [
    BASE_DIR / "camera_controller.py",
    BASE_DIR / "motor_controller.py",
    BASE_DIR / "sensor_controller.py",
]

def main():
    procs = []
    for s in scripts:
        print(f"Starting {s}...")
        p = subprocess.Popen(["python3", str(s)])
        procs.append(p)

    # Optionally wait for them (or just exit and let systemd manage them)
    for p in procs:
        p.wait()

if __name__ == "__main__":
    main()
