#!/usr/bin/env python3
import os
import signal
import subprocess
import sys
import time

SCRIPTS = [
    ("camera", "sub_camera.py"),
    ("motors", "sub_motors.py"),
    ("sensors", "sub_sensors.py"),
]
PROCS = {}

def start_all():
    base = os.path.dirname(os.path.abspath(__file__))
    for name, script in SCRIPTS:
        path = os.path.join(base, script)
        if not os.path.exists(path):
            print(f"[launcher] Missing: {path}")
            continue
        p = subprocess.Popen([sys.executable, path], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                             preexec_fn=os.setsid)
        PROCS[name] = p
        print(f"[launcher] started {name} pid={p.pid}")
        time.sleep(0.2)

def stop_all():
    for name, p in PROCS.items():
        try:
            os.killpg(os.getpgid(p.pid), signal.SIGTERM)
        except Exception:
            pass
    time.sleep(0.5)
    for name, p in PROCS.items():
        try:
            if p.poll() is None:
                os.killpg(os.getpgid(p.pid), signal.SIGKILL)
        except Exception:
            pass

def _handle(sig, frame):
    stop_all()
    sys.exit(0)

def main():
    signal.signal(signal.SIGTERM, _handle)
    signal.signal(signal.SIGINT, _handle)
    start_all()
    while True:
        time.sleep(1.0)

if __name__ == "__main__":
    main()
