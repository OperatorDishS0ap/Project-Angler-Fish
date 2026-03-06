import socket
import struct
import threading
import time
from typing import Optional

import cv2
import numpy as np
import av


class CameraClient:
    """
    Receives a H.264 stream over UDP:
    Sends HELLO to server, then receives H.264 packets and decodes to frames.
    """
    def __init__(self, host: str, port: int = 8000, reconnect: bool = True):
        self.host = host
        self.port = port
        self.reconnect = reconnect

        self._sock: Optional[socket.socket] = None
        self._thread: Optional[threading.Thread] = None
        self._stop = threading.Event()

        self._lock = threading.Lock()
        self._latest_frame: Optional[np.ndarray] = None
        self._connected = False

        self._codec = None

    @property
    def connected(self) -> bool:
        return self._connected

    def start(self) -> None:
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        try:
            if self._sock:
                self._sock.close()
        except Exception:
            pass
        self._sock = None
        self._codec = None

    def get_frame(self) -> Optional[np.ndarray]:
        with self._lock:
            return None if self._latest_frame is None else self._latest_frame.copy()

    def _run(self) -> None:
        while not self._stop.is_set():
            try:
                self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                self._sock.settimeout(5)
                self._sock.sendto(b'HELLO', (self.host, self.port))
                self._sock.settimeout(None)
                self._connected = True
                print(f"[camera] Connected to {self.host}:{self.port}")

                # Setup H.264 decoder
                self._codec = av.CodecContext.create('h264', 'r')
                self._codec.open()
                
                packets_received = 0
                last_log_time = time.time()

                while not self._stop.is_set():
                    data, addr = self._sock.recvfrom(65536)
                    if data:
                        packets_received += 1
                        packet = av.Packet(data)
                        frames = self._codec.decode(packet)
                        for frame in frames:
                            img = frame.to_ndarray(format='bgr24')
                            with self._lock:
                                self._latest_frame = img
                        
                        # Log every second
                        if time.time() - last_log_time >= 1:
                            print(f"[camera] Packets received: {packets_received}")
                            packets_received = 0
                            last_log_time = time.time()
            except socket.timeout:
                print(f"[camera] Socket timeout - no data received")
                self._connected = False
                self._codec = None
                try:
                    if self._sock:
                        self._sock.close()
                except Exception:
                    pass
                self._sock = None
                if self.reconnect and not self._stop.is_set():
                    time.sleep(1.0)
                else:
                    return
            except Exception as e:
                print(f"[camera] Error: {e}")
                self._connected = False
                self._codec = None
                try:
                    if self._sock:
                        self._sock.close()
                except Exception:
                    pass
                self._sock = None
                if self.reconnect and not self._stop.is_set():
                    time.sleep(1.0)
                else:
                    return
