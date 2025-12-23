#!/usr/bin/env python3
"""
CSI Camera (Camera Module 3) streamer for Raspberry Pi (libcamera stack).

Stream protocol (to PC):
  [4-byte big-endian length][JPEG bytes]...

Preferred backend: Picamera2 (libcamera) -> numpy -> cv2.imencode -> TCP
Fallback: OpenCV VideoCapture (USB/V4L2) if Picamera2 is not available.
"""
import socket
import struct
import time

import cv2


HOST = "0.0.0.0"
PORT = 8000

# Streaming settings
WIDTH = 640
HEIGHT = 480
FPS = 30
JPEG_QUALITY = 75

# Fallback (USB cam) if picamera2 isn't installed
USB_CAM_INDEX = 0


def _run_picamera2():
    from picamera2 import Picamera2

    picam2 = Picamera2()
    config = picam2.create_video_configuration(
        main={"size": (WIDTH, HEIGHT), "format": "RGB888"},
        controls={"FrameRate": FPS},
    )
    picam2.configure(config)
    picam2.start()
    time.sleep(0.2)

    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind((HOST, PORT))
    srv.listen(1)
    print(f"[sub_camera] (CSI/Picamera2) Listening on {HOST}:{PORT}")

    while True:
        conn, addr = srv.accept()
        print(f"[sub_camera] Client connected: {addr}")
        try:
            while True:
                frame = picam2.capture_array()  # RGB888
                frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
                ok, jpg = cv2.imencode(".jpg", frame_bgr, [int(cv2.IMWRITE_JPEG_QUALITY), JPEG_QUALITY])
                if not ok:
                    continue
                data = jpg.tobytes()
                conn.sendall(struct.pack(">I", len(data)))
                conn.sendall(data)
                time.sleep(1.0 / max(1, FPS))
        except Exception as e:
            print(f"[sub_camera] client disconnected ({e})")
        finally:
            try:
                conn.close()
            except Exception:
                pass


def _run_opencv_fallback():
    cap = cv2.VideoCapture(USB_CAM_INDEX, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, HEIGHT)
    cap.set(cv2.CAP_PROP_FPS, FPS)

    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind((HOST, PORT))
    srv.listen(1)
    print(f"[sub_camera] (USB/OpenCV fallback) Listening on {HOST}:{PORT}")

    while True:
        conn, addr = srv.accept()
        print(f"[sub_camera] Client connected: {addr}")
        try:
            while True:
                ok, frame = cap.read()
                if not ok:
                    time.sleep(0.05)
                    continue
                ok, jpg = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), JPEG_QUALITY])
                if not ok:
                    continue
                data = jpg.tobytes()
                conn.sendall(struct.pack(">I", len(data)))
                conn.sendall(data)
                time.sleep(1.0 / max(1, FPS))
        except Exception as e:
            print(f"[sub_camera] client disconnected ({e})")
        finally:
            try:
                conn.close()
            except Exception:
                pass


def main():
    try:
        import picamera2  # noqa: F401
        _run_picamera2()
    except Exception as e:
        print(f"[sub_camera] Picamera2 not available or failed ({e}). Falling back to OpenCV V4L2.")
        _run_opencv_fallback()


if __name__ == "__main__":
    main()
