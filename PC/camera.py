import threading
import time
from typing import Optional
import os

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
        self._fps = 0.0
        self._fps_count = 0
        self._fps_last_ts = time.time()

    @property
    def connected(self) -> bool:
        return self._connected

    @property
    def fps(self) -> float:
        return self._fps

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
        host = self.host.strip()
        if ":" in host and not (host.startswith("[") and host.endswith("]")):
            host = f"[{host}]"
        url = f"rtsp://{host}:{self.port}/{self.path}"
        while not self._stop.is_set():
            try:
                # Hint FFmpeg backend to keep RTSP buffering minimal.
                os.environ["OPENCV_FFMPEG_CAPTURE_OPTIONS"] = (
                    "rtsp_transport;udp|fflags;nobuffer|flags;low_delay|max_delay;50000"
                )
                self._capture = cv2.VideoCapture(url, cv2.CAP_FFMPEG)
                self._capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)
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

                    self._fps_count += 1
                    now = time.time()
                    if now - self._fps_last_ts >= 1.0:
                        self._fps = self._fps_count / (now - self._fps_last_ts)
                        self._fps_count = 0
                        self._fps_last_ts = now
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
