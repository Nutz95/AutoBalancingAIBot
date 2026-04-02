from __future__ import annotations

import socket
import time


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
