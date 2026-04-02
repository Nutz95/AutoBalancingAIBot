from __future__ import annotations

import threading
import time
from collections import deque
from typing import Any

from .telemetry import TelemetryPacket, TelemetrySignalProcessor


class DashboardState:
    def __init__(self, window_sec: float, processor: TelemetrySignalProcessor) -> None:
        self.window_sec = window_sec
        self._processor = processor
        self._samples: deque[TelemetryPacket] = deque()
        self._lock = threading.Lock()
        self._packet_count = 0
        self._last_rx_monotonic = 0.0
        self._status = "Waiting for telemetry…"

    def add(self, packet: TelemetryPacket) -> None:
        with self._lock:
            self._samples.append(packet)
            self._packet_count += 1
            self._last_rx_monotonic = time.monotonic()
            self._status = "Receiving telemetry"
            cutoff_ms = packet.timestamp_ms - int(self.window_sec * 1000.0)
            while self._samples and self._samples[0].timestamp_ms < cutoff_ms:
                self._samples.popleft()

    def set_status(self, status: str) -> None:
        with self._lock:
            self._status = status

    def snapshot(self) -> dict[str, Any]:
        with self._lock:
            packets = list(self._samples)
            latest = packets[-1] if packets else None
            status = self._status
            timed_out = self._last_rx_monotonic and (time.monotonic() - self._last_rx_monotonic) > 2.0
            if timed_out:
                status = "Telemetry timeout"

            if latest is None:
                return {
                    "window_sec": self.window_sec,
                    "samples": [],
                    "fft": [],
                    "fft_meta": {"x_max_hz": 25.0, "sample_rate_hz": None},
                    "latest": None,
                    "status": status,
                }

            latest_ts = latest.timestamp_ms
            stride = max(1, len(packets) // 900)
            sliced = packets[::stride]
            if sliced[-1] is not latest:
                sliced.append(latest)

            samples = [packet.to_dict(latest_ts) for packet in sliced]
            samples = self._processor.enrich_samples(samples)
            fft_payload = self._processor.compute_fft(samples)
            latest_dict = samples[-1].copy()
            latest_dict["packet_count"] = self._packet_count
            return {
                "window_sec": self.window_sec,
                "samples": samples,
                "fft": fft_payload["samples"],
                "fft_meta": {
                    "x_max_hz": fft_payload["x_max_hz"],
                    "sample_rate_hz": fft_payload["sample_rate_hz"],
                },
                "latest": latest_dict,
                "status": status,
            }
