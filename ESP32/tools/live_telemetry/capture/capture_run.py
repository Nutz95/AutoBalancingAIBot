from __future__ import annotations

import csv
import json
from dataclasses import fields
from pathlib import Path
from typing import Any

from ..telemetry import TelemetryPacket, TelemetrySignalProcessor


class CaptureRun:
    MAX_PREVIEW_SAMPLES = 900

    def __init__(
        self,
        capture_id: str,
        label: str,
        capture_root: Path,
        armed_at_unix_ms: int,
        mode: str = "manual",
    ) -> None:
        self.capture_id = capture_id
        self.label = label
        self.mode = mode
        self.capture_dir = capture_root / capture_id
        self.csv_path = self.capture_dir / f"{capture_id}.csv"
        self.json_path = self.capture_dir / f"{capture_id}.json"
        self.log_path = self.capture_dir / f"{capture_id}.log"
        self.armed_at_unix_ms = armed_at_unix_ms
        self.started_at_unix_ms: int | None = None
        self.ended_at_unix_ms: int | None = None
        self.start_trigger = "manual_arm"
        self.start_trigger_message: str | None = None
        self.stop_reason: str | None = None
        self.status = "armed"
        self.packet_count = 0
        self.first_packet_timestamp_ms: int | None = None
        self.last_packet_timestamp_ms: int | None = None
        self.summary: dict[str, Any] | None = None
        self.preview_payload: dict[str, Any] = {"samples": [], "fft": [], "fft_meta": {"x_max_hz": 25.0, "sample_rate_hz": None}}
        self._config_snapshot: dict[str, Any] | None = None
        self._telemetry_packets: list[TelemetryPacket] = []
        self._log_entries: list[dict[str, Any]] = []
        self._csv_file: Any | None = None
        self._csv_writer: csv.DictWriter[str] | None = None
        self._packet_field_names = [field.name for field in fields(TelemetryPacket)]

    def start(self, started_at_unix_ms: int, trigger: str, trigger_message: str | None, config_snapshot: dict[str, Any] | None) -> None:
        if self.started_at_unix_ms is not None:
            return
        self.capture_dir.mkdir(parents=True, exist_ok=True)
        self.started_at_unix_ms = started_at_unix_ms
        self.start_trigger = trigger
        self.start_trigger_message = trigger_message
        self._config_snapshot = config_snapshot
        self.status = "capturing"
        self._csv_file = self.csv_path.open("w", newline="", encoding="utf-8")
        self._csv_writer = csv.DictWriter(self._csv_file, fieldnames=["received_unix_ms", *self._packet_field_names])
        self._csv_writer.writeheader()
        self._write_log_line({"unix_ms": started_at_unix_ms, "level": "capture", "message": f"Capture started via {trigger}"})

    def append_packet(self, packet: TelemetryPacket, received_unix_ms: int) -> None:
        if self.started_at_unix_ms is None or self._csv_writer is None:
            return
        if self.first_packet_timestamp_ms is None:
            self.first_packet_timestamp_ms = packet.timestamp_ms
        self.last_packet_timestamp_ms = packet.timestamp_ms
        self.packet_count += 1
        self._telemetry_packets.append(packet)
        self._csv_writer.writerow({"received_unix_ms": received_unix_ms, **packet.__dict__})
        if self._csv_file is not None:
            self._csv_file.flush()

    def append_log_entry(self, entry: dict[str, Any]) -> None:
        if self.started_at_unix_ms is None:
            return
        log_entry = {
            "unix_ms": int(entry.get("unix_ms", 0) or 0),
            "level": str(entry.get("level", "robot")),
            "message": str(entry.get("message", "")),
        }
        self._log_entries.append(log_entry)
        self._write_log_line(log_entry)

    def update_preview(self, processor: TelemetrySignalProcessor) -> None:
        preview_packets = self._downsample_packets(self._telemetry_packets, self.MAX_PREVIEW_SAMPLES)
        self.preview_payload = self._build_preview_payload(preview_packets, processor)

    def finalize(self, ended_at_unix_ms: int, stop_reason: str, summary: dict[str, Any], processor: TelemetrySignalProcessor) -> None:
        self.ended_at_unix_ms = ended_at_unix_ms
        self.stop_reason = stop_reason
        self.status = "completed"
        self.summary = summary
        self.update_preview(processor)
        self._write_log_line({"unix_ms": ended_at_unix_ms, "level": "capture", "message": f"Capture stopped: {stop_reason}"})
        if self._csv_file is not None:
            self._csv_file.close()
            self._csv_file = None
            self._csv_writer = None
        self.json_path.write_text(json.dumps(self.to_manifest(include_preview=True), indent=2), encoding="utf-8")

    def to_manifest(self, include_preview: bool = False) -> dict[str, Any]:
        manifest = {
            "capture_id": self.capture_id,
            "label": self.label,
            "mode": self.mode,
            "directory": self.capture_dir.name,
            "status": self.status,
            "armed_at_unix_ms": self.armed_at_unix_ms,
            "started_at_unix_ms": self.started_at_unix_ms,
            "ended_at_unix_ms": self.ended_at_unix_ms,
            "start_trigger": self.start_trigger,
            "start_trigger_message": self.start_trigger_message,
            "stop_reason": self.stop_reason,
            "packet_count": self.packet_count,
            "first_packet_timestamp_ms": self.first_packet_timestamp_ms,
            "last_packet_timestamp_ms": self.last_packet_timestamp_ms,
            "summary": self.summary,
            "config_snapshot": self._config_snapshot,
            "files": {
                "csv": f"/api/capture/download?capture_id={self.capture_id}&kind=csv",
                "json": f"/api/capture/download?capture_id={self.capture_id}&kind=json",
                "log": f"/api/capture/download?capture_id={self.capture_id}&kind=log",
            },
        }
        if include_preview:
            manifest["preview"] = self.preview_payload
        return manifest

    @classmethod
    def load_manifest(cls, manifest_path: Path) -> dict[str, Any]:
        return json.loads(manifest_path.read_text(encoding="utf-8"))

    def _write_log_line(self, entry: dict[str, Any]) -> None:
        self.capture_dir.mkdir(parents=True, exist_ok=True)
        line = f"[{entry['unix_ms']}] {entry['level']}: {entry['message']}\n"
        with self.log_path.open("a", encoding="utf-8") as log_file:
            log_file.write(line)

    @staticmethod
    def _downsample_packets(packets: list[TelemetryPacket], max_samples: int) -> list[TelemetryPacket]:
        if len(packets) <= max_samples:
            return packets
        if max_samples <= 1:
            return [packets[-1]]
        sampled_packets: list[TelemetryPacket] = []
        last_index = len(packets) - 1
        for index in range(max_samples):
            packet_index = round(index * last_index / (max_samples - 1))
            sampled_packets.append(packets[packet_index])
        return sampled_packets

    @staticmethod
    def _build_preview_payload(packets: list[TelemetryPacket], processor: TelemetrySignalProcessor) -> dict[str, Any]:
        if not packets:
            return {"samples": [], "fft": [], "fft_meta": {"x_max_hz": 25.0, "sample_rate_hz": None}}
        latest_timestamp_ms = packets[-1].timestamp_ms
        samples = [packet.to_dict(latest_timestamp_ms) for packet in packets]
        samples = processor.enrich_samples(samples)
        fft_payload = processor.compute_fft(samples)
        return {
            "samples": samples,
            "fft": fft_payload["samples"],
            "fft_meta": {
                "x_max_hz": fft_payload["x_max_hz"],
                "sample_rate_hz": fft_payload["sample_rate_hz"],
            },
        }