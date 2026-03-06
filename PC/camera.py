import threading
import time
from typing import Optional

import cv2
import numpy as np


class CameraClient:
    """
    Simple RTSP client using OpenCV. Connects to an RTSP stream and keeps
    the latest frame available for retrieval.
    """
    def __init__(self, host: str, port: int = 8554, path: str = "stream", reconnect: bool = True):
        self.host = host
        self.port = port
        self.path = path
        self.reconnect = reconnect

        self._thread: Optional[threading.Thread] = None
        self._stop = threading.Event()

        self._lock = threading.Lock()
        self._latest_frame: Optional[np.ndarray] = None
        self._connected = False

        self._capture: Optional[cv2.VideoCapture] = None

    @property
    def connected(self) -> bool:
        return self._connected

    def start(self) -> None:
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._capture is not None:
            self._capture.release()
        self._capture = None

    def get_frame(self) -> Optional[np.ndarray]:
        with self._lock:
            return None if self._latest_frame is None else self._latest_frame.copy()

    def _run(self) -> None:
        url = f"rtsp://{self.host}:{self.port}/{self.path}"
        while not self._stop.is_set():
            try:
                self._capture = cv2.VideoCapture(url)
                if not self._capture.isOpened():
                    raise RuntimeError("Unable to open RTSP stream")
                self._connected = True
                print(f"[camera] Connected to {url}")
                while not self._stop.is_set():
                    ret, frame = self._capture.read()
                    if not ret:
                        raise RuntimeError("Frame read failed")
                    with self._lock:
                        self._latest_frame = frame
            except Exception as e:
                print(f"[camera] Error: {e}")
                self._connected = False
                if self._capture is not None:
                    self._capture.release()
                self._capture = None
                if self.reconnect and not self._stop.is_set():
                    time.sleep(1.0)
                else:
                    return
