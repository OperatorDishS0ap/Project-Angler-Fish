import socket
import struct
import time
import cv2


HOST = "0.0.0.0"
PORT = 8000

# Streaming settings
WIDTH = 1280
HEIGHT = 720
FPS = 100
JPEG_QUALITY = 75

def _run_picamera2():
    from picamera2 import Picamera2

    picam2 = Picamera2()
    config = picam2.create_video_configuration(
        main={"size": (WIDTH, HEIGHT), "format": "BGR888"},
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
            frame_count = 0
            start_time = time.time()
            actual_fps = 0
            while True:
                frame = picam2.capture_array()  # RGB888
                frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
                frame_bgr = cv2.flip(frame_bgr, 0)  # Flip vertically
                frame_count += 1
                elapsed = time.time() - start_time
                if elapsed >= 1:  # Update FPS every 1 second
                    actual_fps = frame_count / elapsed
                    frame_count = 0
                    start_time = time.time()
                # Draw FPS on frame
                cv2.putText(frame_bgr, f"FPS: {actual_fps:.1f}", (10, 30), 
                           cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
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




def main():
    _run_picamera2()


if __name__ == "__main__":
    main()
