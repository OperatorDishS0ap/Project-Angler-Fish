import socket
import struct
import threading
import time
from typing import Optional, Tuple

import cv2
import numpy as np


class CameraClient:
    """
    Receives a JPEG stream over TCP:
      [4-byte big-endian length][JPEG bytes]...
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

    def get_frame(self) -> Optional[np.ndarray]:
        with self._lock:
            if self._latest_frame is None:
                return None
            return self._latest_frame.copy()

    def _recv_exact(self, n: int) -> Optional[bytes]:
        buf = b""
        while len(buf) < n and not self._stop.is_set():
            chunk = self._sock.recv(n - len(buf))
            if not chunk:
                return None
            buf += chunk
        return buf

    def _connect(self) -> bool:
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.settimeout(5)
            s.connect((self.host, self.port))
            s.settimeout(None)
            self._sock = s
            self._connected = True
            return True
        except Exception:
            self._connected = False
            self._sock = None
            return False

    def _run(self) -> None:
        while not self._stop.is_set():
            if not self._connect():
                if not self.reconnect:
                    return
                time.sleep(1.0)
                continue

            try:
                while not self._stop.is_set():
                    header = self._recv_exact(4)
                    if header is None:
                        break
                    (size,) = struct.unpack(">I", header)
                    data = self._recv_exact(size)
                    if data is None:
                        break
                    jpg = np.frombuffer(data, dtype=np.uint8)
                    frame = cv2.imdecode(jpg, cv2.IMREAD_COLOR)
                    if frame is not None:
                        with self._lock:
                            self._latest_frame = frame
            except Exception:
                pass
            finally:
                self._connected = False
                try:
                    if self._sock:
                        self._sock.close()
                except Exception:
                    pass
                self._sock = None
                if self.reconnect:
                    time.sleep(0.5)
                else:
                    return
