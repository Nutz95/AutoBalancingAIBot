from __future__ import annotations

import socket
import threading

from ..state import DashboardState
from ..telemetry import TelemetryPacketParser


class TelemetryReceiver(threading.Thread):
    def __init__(self, state: DashboardState, parser: TelemetryPacketParser, bind_host: str, udp_port: int) -> None:
        super().__init__(daemon=True)
        self._state = state
        self._parser = parser
        self._bind_host = bind_host
        self._udp_port = udp_port
        self._stop_event = threading.Event()

    def stop(self) -> None:
        self._stop_event.set()

    def run(self) -> None:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind((self._bind_host, self._udp_port))
        sock.settimeout(0.5)
        self._state.set_status(f"Listening on UDP {self._bind_host or '0.0.0.0'}:{self._udp_port}")
        while not self._stop_event.is_set():
            try:
                data, _ = sock.recvfrom(2048)
            except socket.timeout:
                continue
            except OSError:
                break
            packet = self._parser.parse(data)
            if packet is not None:
                self._state.add(packet)
        sock.close()
