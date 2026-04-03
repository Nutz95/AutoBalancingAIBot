from __future__ import annotations

import json
import socket
import threading
import time
from typing import Any

from .log_buffer import LogBuffer


class ConsoleSession:
    DEFAULT_CONNECT_TIMEOUT_S = 5.0
    DEFAULT_RECONNECT_DELAY_S = 1.0
    DEFAULT_READ_TIMEOUT_S = 0.2
    DEFAULT_REQUEST_TIMEOUT_S = 2.0

    def __init__(
        self,
        host: str,
        port: int,
        log_buffer: LogBuffer,
        connect_timeout_s: float = DEFAULT_CONNECT_TIMEOUT_S,
        reconnect_delay_s: float = DEFAULT_RECONNECT_DELAY_S,
        read_timeout_s: float = DEFAULT_READ_TIMEOUT_S,
    ) -> None:
        self._host = host
        self._port = port
        self._log_buffer = log_buffer
        self._connect_timeout_s = connect_timeout_s
        self._reconnect_delay_s = reconnect_delay_s
        self._read_timeout_s = read_timeout_s
        self._stop_event = threading.Event()
        self._connected_event = threading.Event()
        self._socket_lock = threading.Lock()
        self._waiter_lock = threading.Lock()
        self._socket: socket.socket | None = None
        self._thread: threading.Thread | None = None
        self._pending_responses: dict[str, dict[str, Any]] = {}
        self._line_buffer = ""
        self._last_connect_failure_log_monotonic = 0.0

    def start(self) -> None:
        if self._thread and self._thread.is_alive():
            return
        self._thread = threading.Thread(target=self._run, name="live-telemetry-console", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop_event.set()
        self._disconnect()
        if self._thread:
            self._thread.join(timeout=1.0)

    def is_connected(self) -> bool:
        return self._connected_event.is_set()

    def wait_until_connected(self, timeout_s: float) -> bool:
        return self._connected_event.wait(timeout_s)

    def send_command(self, command: str) -> None:
        command_text = command.strip()
        if not command_text:
            return
        encoded_command = f"{command_text}\n".encode("utf-8")
        with self._socket_lock:
            if not self._socket:
                raise OSError("Robot console is not connected")
            self._socket.sendall(encoded_command)
        self._log_buffer.append("command", f"> {command_text}")

    def send_commands(self, commands: list[str]) -> None:
        for command in commands:
            self.send_command(command)
            time.sleep(0.05)

    def request_json(self, command: str, response_prefix: str, timeout_s: float = DEFAULT_REQUEST_TIMEOUT_S) -> dict[str, Any]:
        event = threading.Event()
        waiter = {"event": event, "payload": None}
        with self._waiter_lock:
            if response_prefix in self._pending_responses:
                raise RuntimeError(f"A request is already pending for prefix {response_prefix!r}")
            self._pending_responses[response_prefix] = waiter

        try:
            self.send_command(command)
            if not event.wait(timeout_s):
                raise TimeoutError(f"Timed out waiting for {response_prefix}")
            payload_text = waiter["payload"]
            if payload_text is None:
                raise TimeoutError(f"No payload received for {response_prefix}")
            return json.loads(payload_text)
        finally:
            with self._waiter_lock:
                self._pending_responses.pop(response_prefix, None)

    def _run(self) -> None:
        while not self._stop_event.is_set():
            if not self._connect_socket():
                if not self._stop_event.wait(self._reconnect_delay_s):
                    continue
                break

            try:
                self._read_loop()
            finally:
                self._disconnect(log_disconnect=self._stop_event.is_set() is False)

    def _connect_socket(self) -> bool:
        try:
            sock = socket.create_connection((self._host, self._port), timeout=self._connect_timeout_s)
            sock.settimeout(self._read_timeout_s)
        except OSError as exc:
            now = time.monotonic()
            if (now - self._last_connect_failure_log_monotonic) >= 5.0:
                self._log_buffer.append("system", f"Console reconnect failed: {exc}")
                self._last_connect_failure_log_monotonic = now
            return False

        with self._socket_lock:
            self._socket = sock
        self._connected_event.set()
        self._line_buffer = ""
        self._last_connect_failure_log_monotonic = 0.0
        self._log_buffer.append("system", f"Console connected to {self._host}:{self._port}")
        return True

    def _disconnect(self, log_disconnect: bool = False) -> None:
        with self._socket_lock:
            sock = self._socket
            self._socket = None
        if sock is not None:
            try:
                sock.close()
            except OSError:
                pass
        self._connected_event.clear()
        if log_disconnect:
            self._log_buffer.append("system", "Console disconnected")

    def _read_loop(self) -> None:
        while not self._stop_event.is_set():
            with self._socket_lock:
                sock = self._socket
            if sock is None:
                return

            try:
                chunk = sock.recv(4096)
            except socket.timeout:
                continue
            except OSError as exc:
                self._log_buffer.append("system", f"Console read error: {exc}")
                return

            if not chunk:
                return

            self._consume_text(chunk.decode("utf-8", errors="ignore"))

    def _consume_text(self, text: str) -> None:
        self._line_buffer += text.replace("\r", "")
        while "\n" in self._line_buffer:
            line, self._line_buffer = self._line_buffer.split("\n", 1)
            stripped_line = line.strip()
            if stripped_line:
                self._handle_line(stripped_line)

    def _handle_line(self, line: str) -> None:
        with self._waiter_lock:
            for response_prefix, waiter in self._pending_responses.items():
                if line.startswith(response_prefix):
                    waiter["payload"] = line[len(response_prefix):].strip()
                    waiter["event"].set()
                    return
        self._log_buffer.append("robot", line)
