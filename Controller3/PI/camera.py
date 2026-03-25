#!/usr/bin/env python3
import signal
import time

from picamera2 import Picamera2
from picamera2.encoders import H264Encoder
from picamera2.outputs import PyavOutput
from libcamera import Transform

# ----------------------------
# Settings
# ----------------------------
WIDTH = 960
HEIGHT = 540
FPS = 30
BITRATE = 1_500_000
RTSP_URL = "rtsp://anglerfish.local:8554/cam"

running = True


def handle_stop(signum, frame):
    global running
    running = False


def main():
    global running

    signal.signal(signal.SIGINT, handle_stop)
    signal.signal(signal.SIGTERM, handle_stop)

    picam2 = Picamera2()

    # Use YUV420 for video encoding to keep the pipeline efficient.
    config = picam2.create_video_configuration(
        main={"size": (WIDTH, HEIGHT), "format": "YUV420"},
        controls={"FrameRate": FPS},
        transform=Transform(hflip=1, vflip=1),
        buffer_count=3,
    )
    picam2.configure(config)

    # repeat=True inserts SPS/PPS periodically, which is helpful for streaming.
    # iperiod=15 means an I-frame roughly every 15 frames.
    encoder = H264Encoder(
        bitrate=BITRATE,
        repeat=True,
        iperiod=15,
    )

    # Publish directly to MediaMTX over RTSP.
    output = PyavOutput(RTSP_URL, format="rtsp")

    print("Starting camera...")

    picam2.start_recording(encoder, output)

    try:
        while running:
            time.sleep(1)
    finally:
        print("Stopping...")
        try:
            picam2.stop_recording()
        except Exception as e:
            print(f"Warning while stopping recording: {e}")
        picam2.close()


if __name__ == "__main__":
    main()