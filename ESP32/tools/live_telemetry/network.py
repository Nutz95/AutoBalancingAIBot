from __future__ import annotations

import socket
import threading
import time

from .state import DashboardState
from .telemetry import TelemetryPacketParser


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


class RobotConsoleClient:
    def __init__(self, host: str, port: int, timeout: float = 5.0) -> None:
        self._host = host
        self._port = port
        self._timeout = timeout

    def send(self, *commands: str) -> list[str]:
        lines: list[str] = []
        with socket.create_connection((self._host, self._port), timeout=self._timeout) as sock:
            sock.setblocking(False)
            time.sleep(0.2)
            self._drain(sock, lines, 0.3)
            for command in commands:
                if not command.endswith("\n"):
                    command += "\n"
                sock.sendall(command.encode("utf-8"))
                self._drain(sock, lines, 0.35)
        return lines

    @staticmethod
    def _drain(sock: socket.socket, lines: list[str], quiet_s: float) -> None:
        deadline = time.monotonic() + quiet_s
        while time.monotonic() < deadline:
            try:
                chunk = sock.recv(8192)
            except BlockingIOError:
                time.sleep(0.02)
                continue
            if not chunk:
                break
            text = chunk.decode("utf-8", errors="ignore")
            for line in text.splitlines():
                if line.strip():
                    lines.append(line.rstrip())
            deadline = time.monotonic() + quiet_s
