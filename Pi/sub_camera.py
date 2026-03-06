import socket
import struct
import time
from fractions import Fraction
import cv2
import av


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
        main={"size": (WIDTH, HEIGHT), "format": "RGB888"},
        controls={"FrameRate": FPS},
    )
    picam2.configure(config)
    picam2.start()
    time.sleep(0.2)

    srv = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    srv.bind((HOST, PORT))
    print(f"[sub_camera] (CSI/Picamera2) Listening on {HOST}:{PORT}")

    # Wait for HELLO from client
    client_addr = None
    while True:
        data, addr = srv.recvfrom(1024)
        if data == b'HELLO':
            client_addr = addr
            print(f"[sub_camera] Client connected: {addr}")
            break

    # Setup H.264 encoder
    codec = av.CodecContext.create('libx264', 'w')
    codec.width = WIDTH
    codec.height = HEIGHT
    codec.pix_fmt = 'yuv420p'
    codec.time_base = Fraction(1, FPS)
    codec.bit_rate = 2000000  # Adjust bitrate as needed
    codec.open()

    frame_count = 0
    start_time = time.time()
    actual_fps = 0
    try:
        while True:
            frame = picam2.capture_array()  # RGB888
            frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            frame_bgr = cv2.flip(frame_bgr, 0)  # Flip vertically

            # Encode to H.264
            av_frame = av.VideoFrame.from_ndarray(frame_bgr, format='bgr24')
            av_frame.pts = frame_count
            packets = codec.encode(av_frame)
            for packet in packets:
                srv.sendto(bytes(packet), client_addr)

            frame_count += 1
            elapsed = time.time() - start_time
            if elapsed >= 1:  # Update FPS every 1 second
                actual_fps = frame_count / elapsed
                frame_count = 0
                start_time = time.time()
            # Note: No sleep needed as picamera2 controls frame rate
    except Exception as e:
        print(f"[sub_camera] Error: {e}")
    finally:
        srv.close()




def main():
    _run_picamera2()


if __name__ == "__main__":
    main()
