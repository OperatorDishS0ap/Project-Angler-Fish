import socket
import time
import io
from picamera2 import Picamera2
from picamera2.encoders import H264Encoder
from picamera2.outputs import Output


HOST = "0.0.0.0"
PORT = 8000

# Streaming settings
WIDTH = 640
HEIGHT = 480
FPS = 30
BITRATE = 500000  # Reduced from 2Mbps to 500kbps to fit in UDP packets

class UdpOutput(Output):
    def __init__(self, socket, addr):
        super().__init__()
        self.socket = socket
        self.addr = addr
        self.packets_sent = 0
        self.max_udp_size = 65000  # Safe UDP packet size limit

    def outputframe(self, frame, keyframe=None, timestamp=None, packet=None, audio=None):
        if frame:
            # Debug: print frame size
            print(f"[sub_camera] Frame size: {len(frame)} bytes")
            
            # If frame fits in single UDP packet, send as-is
            if len(frame) <= self.max_udp_size:
                self.socket.sendto(frame, self.addr)
                self.packets_sent += 1
            else:
                # Split large frames (this may break NAL units - not ideal)
                print(f"[sub_camera] Splitting large frame: {len(frame)} bytes")
                for i in range(0, len(frame), self.max_udp_size):
                    chunk = frame[i:i + self.max_udp_size]
                    self.socket.sendto(chunk, self.addr)
                    self.packets_sent += 1

def _run_picamera2():
    picam2 = Picamera2()
    config = picam2.create_video_configuration(
        main={"size": (WIDTH, HEIGHT), "format": "RGB888"},
        controls={"FrameRate": FPS},
    )
    picam2.configure(config)

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

    # Setup hardware H.264 encoder with UDP output
    output = UdpOutput(srv, client_addr)
    encoder = H264Encoder(bitrate=BITRATE)
    picam2.start_recording(encoder, output)
    
    print(f"[sub_camera] Started hardware H.264 encoding at {WIDTH}x{HEIGHT}@{FPS}fps")

    start_time = time.time()
    packets_sent = 0
    try:
        while True:
            time.sleep(1.0)
            packets_sent = output.packets_sent
            output.packets_sent = 0
            elapsed = time.time() - start_time
            if elapsed >= 1:
                print(f"[sub_camera] Packets sent: {packets_sent}")
                start_time = time.time()
    except Exception as e:
        print(f"[sub_camera] Error: {e}")
    finally:
        picam2.stop_recording()
        srv.close()




def main():
    _run_picamera2()


if __name__ == "__main__":
    main()
