import json
import socket
import threading
from dataclasses import dataclass
from typing import Optional


@dataclass
class Telemetry:
    battery: float = 0.0
    depth: float = 0.0
    pressure: float = 0.0
    temp_pi: float = 0.0
    temp_env: float = 0.0


class SensorUdpReceiver:
    def __init__(self, listen_port: int = 9100):
        self.listen_port = listen_port
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.bind(("0.0.0.0", self.listen_port))
        self._sock.settimeout(1.0)

        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None

        self.latest = Telemetry()

    def start(self) -> None:
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()

    def _run(self) -> None:
        while not self._stop.is_set():
            try:
                data, _addr = self._sock.recvfrom(4096)
                msg = json.loads(data.decode("utf-8", errors="ignore"))
                self.latest = Telemetry(
                    battery=float(msg.get("battery", self.latest.battery)),
                    depth=float(msg.get("depth", self.latest.depth)),
                    pressure=float(msg.get("pressure", self.latest.pressure)),
                    temp_pi=float(msg.get("temp_pi", self.latest.temp_pi)),
                    temp_env=float(msg.get("temp_env", self.latest.temp_env)),
                )
            except socket.timeout:
                continue
            except Exception:
                continue
