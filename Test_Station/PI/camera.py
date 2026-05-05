#!/usr/bin/env python3
import os
import shlex
import signal
import subprocess
import sys
import time

# ----------------------------
# Settings
# ----------------------------
WIDTH = 1280
HEIGHT = 720
FPS = 60
BITRATE = 12_000_000
RTP_HOST = "192.168.1.30"
RTP_PORT = int(os.environ.get("ANGLERFISH_RTP_PORT", "5600"))

running = True


def handle_stop(signum, frame):
    global running
    running = False


def main():
    global running

    signal.signal(signal.SIGINT, handle_stop)
    signal.signal(signal.SIGTERM, handle_stop)

    libcamera_cmd = [
        "rpicam-vid",
        "--nopreview",
        "--inline",
        "--width",
        str(WIDTH),
        "--height",
        str(HEIGHT),
        "--framerate",
        str(FPS),
        "--bitrate",
        str(BITRATE),
        "--codec",
        "h264",
        "--hflip",
        "--vflip",
        "--timeout",
        "0",
        "--output",
        "-",
    ]
    gst_cmd = [
        "gst-launch-1.0",
        "-e",
        "fdsrc",
        "!",
        "h264parse",
        "config-interval=1",
        "!",
        "rtph264pay",
        "pt=96",
        "config-interval=1",
        "!",
        "udpsink",
        f"host={RTP_HOST}",
        f"port={RTP_PORT}",
        "sync=false",
        "async=false",
    ]

    print(f"Starting camera RTP stream to {RTP_HOST}:{RTP_PORT}")
    print(f"camera command: {shlex.join(libcamera_cmd)}")
    print(f"gstreamer command: {shlex.join(gst_cmd)}")

    libcamera_proc = subprocess.Popen(
        libcamera_cmd,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        bufsize=0,
    )
    gst_proc = subprocess.Popen(
        gst_cmd,
        stdin=libcamera_proc.stdout,
        stderr=subprocess.PIPE,
        bufsize=0,
    )
    if libcamera_proc.stdout is not None:
        libcamera_proc.stdout.close()

    try:
        while running and libcamera_proc.poll() is None and gst_proc.poll() is None:
            time.sleep(1)
    finally:
        print("Stopping...")
        for proc in (libcamera_proc, gst_proc):
            if proc.poll() is None:
                proc.terminate()

        try:
            gst_proc.wait(timeout=3)
        except subprocess.TimeoutExpired:
            gst_proc.kill()

        try:
            libcamera_proc.wait(timeout=3)
        except subprocess.TimeoutExpired:
            libcamera_proc.kill()

        if gst_proc.returncode not in (None, 0):
            gst_err = ""
            if gst_proc.stderr is not None:
                gst_err = gst_proc.stderr.read().decode("utf-8", errors="replace").strip()
            if gst_err:
                print(f"GStreamer error: {gst_err}")

        if libcamera_proc.returncode not in (None, 0):
            cam_err = ""
            if libcamera_proc.stderr is not None:
                cam_err = libcamera_proc.stderr.read().decode("utf-8", errors="replace").strip()
            if cam_err:
                print(f"libcamera error: {cam_err}")

        return_code = 0
        if libcamera_proc.returncode not in (None, 0):
            return_code = libcamera_proc.returncode
        elif gst_proc.returncode not in (None, 0):
            return_code = gst_proc.returncode
        if return_code:
            raise SystemExit(return_code)


if __name__ == "__main__":
    sys.exit(main())