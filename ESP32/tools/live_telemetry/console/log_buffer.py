from __future__ import annotations

import threading
import time
from collections import deque
from collections.abc import Callable
from typing import Any


class LogBuffer:
    def __init__(self, max_entries: int = 800) -> None:
        self._entries: deque[dict[str, Any]] = deque(maxlen=max_entries)
        self._lock = threading.Lock()
        self._listeners: list[Callable[[dict[str, Any]], None]] = []
        self._next_sequence = 1

    def append(self, level: str, message: str) -> None:
        entry = {
            "seq": self._next_sequence,
            "unix_ms": time.time_ns() // 1_000_000,
            "level": level,
            "message": message,
        }
        with self._lock:
            self._entries.append(entry)
            self._next_sequence += 1
            listeners = tuple(self._listeners)
        for listener in listeners:
            listener(entry)

    def add_listener(self, listener: Callable[[dict[str, Any]], None]) -> None:
        with self._lock:
            self._listeners.append(listener)

    def snapshot(self, after_sequence: int = 0, limit: int = 200) -> dict[str, Any]:
        with self._lock:
            entries = [entry for entry in self._entries if entry["seq"] > after_sequence]
            if limit > 0:
                entries = entries[-limit:]
            latest_sequence = self._entries[-1]["seq"] if self._entries else 0
            return {
                "entries": entries,
                "latest_sequence": latest_sequence,
            }
