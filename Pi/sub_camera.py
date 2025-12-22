#!/usr/bin/env python3
import socket
import struct
import time

import cv2

HOST = "0.0.0.0"
PORT = 8000
CAM_INDEX = 0  # USB camera
WIDTH = 640
HEIGHT = 480
FPS = 30
JPEG_QUALITY = 75

def main():
    cap = cv2.VideoCapture(CAM_INDEX, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, HEIGHT)
    cap.set(cv2.CAP_PROP_FPS, FPS)

    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind((HOST, PORT))
    srv.listen(1)
    print(f"[sub_camera] Listening on {HOST}:{PORT}")

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

if __name__ == "__main__":
    main()
