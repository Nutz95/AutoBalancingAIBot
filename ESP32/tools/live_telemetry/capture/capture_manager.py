from __future__ import annotations

import json
import shutil
import threading
import time
from pathlib import Path
from typing import Any

from ..console import ConfigSnapshotStore
from ..telemetry import TelemetryPacket, TelemetrySignalProcessor
from .capture_run import CaptureRun


class CaptureManager:
    MANUAL_MODE = "manual"
    AUTO_MODE = "auto"
    PREVIEW_UPDATE_INTERVAL_MS = 400
    MAX_RECENT_CAPTURES = 20
    FALL_STOP_ANGLE_DEG = 45.0
    FALL_STOP_SETTLE_CMD_ABS = 0.05
    FALL_STOP_HOLD_MS = 300

    def __init__(
        self,
        capture_root: Path,
        processor: TelemetrySignalProcessor,
        config_store: ConfigSnapshotStore,
    ) -> None:
        self._capture_root = capture_root
        self._processor = processor
        self._config_store = config_store
        self._capture_root.mkdir(parents=True, exist_ok=True)
        self._lock = threading.Lock()
        self._current_run: CaptureRun | None = None
        self._last_preview_update_unix_ms = 0
        self._fall_auto_stop_state = self._build_fall_auto_stop_state()
        self._recent_manifests = self._load_recent_manifests()

    def arm_capture(self, label: str | None = None, capture_mode: str = MANUAL_MODE) -> dict[str, Any]:
        normalized_mode = self._normalize_capture_mode(capture_mode)
        with self._lock:
            if self._current_run and self._current_run.status in {"armed", "capturing"}:
                raise RuntimeError("A capture is already armed")
            armed_at_unix_ms = self._now_unix_ms()
            capture_id = time.strftime("capture_%Y%m%dT%H%M%S", time.localtime(armed_at_unix_ms / 1000.0)) + f"_{armed_at_unix_ms % 1000:03d}"
            capture_label = (label or "").strip() or capture_id
            self._fall_auto_stop_state = self._build_fall_auto_stop_state()
            self._current_run = CaptureRun(capture_id, capture_label, self._capture_root, armed_at_unix_ms, mode=normalized_mode)
            return self._build_status_payload_locked()

    def stop_capture(self, stop_reason: str = "manual_stop") -> dict[str, Any]:
        capture_run_to_finalize: CaptureRun | None = None
        with self._lock:
            if self._current_run is None:
                raise RuntimeError("No capture is armed")
            if self._current_run.status == "armed":
                self._current_run = None
                return self._build_status_payload_locked()
            capture_run_to_finalize = self._current_run
            self._current_run = None

        completed_manifest = self._finalize_capture_run(capture_run_to_finalize, stop_reason)
        with self._lock:
            self._recent_manifests = [
                completed_manifest,
                *[manifest for manifest in self._recent_manifests if manifest.get("capture_id") != completed_manifest["capture_id"]],
            ]
            self._recent_manifests = self._recent_manifests[: self.MAX_RECENT_CAPTURES]
            payload = self._build_status_payload_locked()
        payload["current_capture"] = completed_manifest
        return payload

    def handle_packet(self, packet: TelemetryPacket) -> None:
        should_start_capture = False
        capture_run_to_finalize: CaptureRun | None = None
        stop_reason: str | None = None
        with self._lock:
            if self._current_run is None:
                return
            if self._current_run.status == "armed" and self._should_start_from_packet(packet, self._current_run.mode):
                should_start_capture = True

        if should_start_capture:
            self._start_current_run(trigger="motion_trigger", trigger_message="Motion detected from telemetry")

        with self._lock:
            if self._current_run is None or self._current_run.status != "capturing":
                return
            received_unix_ms = self._now_unix_ms()
            self._current_run.append_packet(packet, received_unix_ms)
            if (received_unix_ms - self._last_preview_update_unix_ms) >= self.PREVIEW_UPDATE_INTERVAL_MS:
                self._current_run.update_preview(self._processor)
                self._last_preview_update_unix_ms = received_unix_ms
            if self._should_auto_stop_from_packet(packet, self._current_run.mode):
                capture_run_to_finalize = self._current_run
                stop_reason = "fall_detected"
                self._current_run = None

        if capture_run_to_finalize is not None and stop_reason is not None:
            completed_manifest = self._finalize_capture_run(capture_run_to_finalize, stop_reason)
            with self._lock:
                self._recent_manifests = [
                    completed_manifest,
                    *[manifest for manifest in self._recent_manifests if manifest.get("capture_id") != completed_manifest["capture_id"]],
                ]
                self._recent_manifests = self._recent_manifests[: self.MAX_RECENT_CAPTURES]

    def handle_log_entry(self, entry: dict[str, Any]) -> None:
        should_start_capture = False
        capture_run_to_finalize: CaptureRun | None = None
        completed_manifest: dict[str, Any] | None = None
        stop_reason: str | None = None
        message_text = str(entry.get("message", ""))
        with self._lock:
            if self._current_run is None:
                return
            if self._current_run.status == "armed" and self._should_start_from_log(message_text, self._current_run.mode):
                should_start_capture = True

        if should_start_capture:
            self._start_current_run(trigger="log_trigger", trigger_message=message_text)

        with self._lock:
            if self._current_run is None or self._current_run.status != "capturing":
                return
            self._current_run.append_log_entry(entry)
            stop_reason = self._stop_reason_from_log(message_text, self._current_run.mode)
            if stop_reason is not None:
                capture_run_to_finalize = self._current_run
                self._current_run = None

        if capture_run_to_finalize is not None:
            completed_manifest = self._finalize_capture_run(capture_run_to_finalize, stop_reason)
            with self._lock:
                self._recent_manifests = [
                    completed_manifest,
                    *[manifest for manifest in self._recent_manifests if manifest.get("capture_id") != completed_manifest["capture_id"]],
                ]
                self._recent_manifests = self._recent_manifests[: self.MAX_RECENT_CAPTURES]

    def get_status_payload(self) -> dict[str, Any]:
        with self._lock:
            return self._build_status_payload_locked()

    def get_capture_payload(self, capture_id: str) -> dict[str, Any]:
        with self._lock:
            if self._current_run is not None and self._current_run.capture_id == capture_id:
                return self._current_run.to_manifest(include_preview=True)
        manifest_path = self._capture_root / capture_id / f"{capture_id}.json"
        if not manifest_path.is_file():
            raise FileNotFoundError(capture_id)
        return CaptureRun.load_manifest(manifest_path)

    def get_download_info(self, capture_id: str, kind: str) -> tuple[Path, str]:
        if kind not in {"csv", "json", "log"}:
            raise ValueError(kind)
        target_path = self._capture_root / capture_id / f"{capture_id}.{kind}"
        if not target_path.is_file():
            raise FileNotFoundError(capture_id)
        mime_type = {
            "csv": "text/csv; charset=utf-8",
            "json": "application/json; charset=utf-8",
            "log": "text/plain; charset=utf-8",
        }[kind]
        return target_path, mime_type

    def delete_capture(self, capture_id: str) -> dict[str, Any]:
        capture_dir = self._capture_root / capture_id
        with self._lock:
            if self._current_run is not None and self._current_run.capture_id == capture_id:
                raise RuntimeError("Cannot delete a capture that is currently armed or running")
        if not capture_dir.is_dir():
            raise FileNotFoundError(capture_id)
        shutil.rmtree(capture_dir)
        with self._lock:
            self._recent_manifests = [manifest for manifest in self._recent_manifests if manifest.get("capture_id") != capture_id]
            return self._build_status_payload_locked()

    def _build_status_payload_locked(self) -> dict[str, Any]:
        return {
            "state": self._current_run.status if self._current_run is not None else "idle",
            "capture_root": self._capture_root.as_posix(),
            "current_capture": None if self._current_run is None else self._current_run.to_manifest(include_preview=True),
            "recent_captures": self._recent_manifests[: self.MAX_RECENT_CAPTURES],
        }

    def _start_current_run(self, trigger: str, trigger_message: str | None) -> None:
        config_snapshot = self._config_store.get_cached_snapshot()
        with self._lock:
            if self._current_run is None or self._current_run.status != "armed":
                return
            self._fall_auto_stop_state = self._build_fall_auto_stop_state()
            self._current_run.start(
                started_at_unix_ms=self._now_unix_ms(),
                trigger=trigger,
                trigger_message=trigger_message,
                config_snapshot=config_snapshot,
            )
            self._last_preview_update_unix_ms = 0

    def _finalize_capture_run(self, capture_run: CaptureRun, stop_reason: str) -> dict[str, Any]:
        summary = self._build_summary(capture_run)
        capture_run.finalize(self._now_unix_ms(), stop_reason, summary, self._processor)
        return capture_run.to_manifest(include_preview=True)

    def _build_summary(self, capture_run: CaptureRun) -> dict[str, Any]:
        packets = capture_run._telemetry_packets
        if not packets:
            return {"duration_s": 0.0, "packet_count": 0, "dominant_pitch_fft_hz": None, "dominant_cmd_fft_hz": None, "saturation_ratio": 0.0}

        duration_s = max(0.0, (packets[-1].timestamp_ms - packets[0].timestamp_ms) / 1000.0)
        max_abs_pitch_deg = max(abs(packet.pitch_deg) for packet in packets)
        max_abs_cmd_final = max(abs(packet.cmd_final) for packet in packets)
        max_abs_steer = max(abs(packet.steer) for packet in packets)
        max_encoder_age_ms = max(packet.last_encoder_age_ms for packet in packets)
        average_loop_hz = sum(packet.loop_freq_hz for packet in packets) / max(1, len(packets))
        saturation_ratio = sum(1 for packet in packets if int(packet.sat_flags) != 0) / max(1, len(packets))

        preview_packets = CaptureRun._downsample_packets(packets, CaptureRun.MAX_PREVIEW_SAMPLES)
        preview_payload = CaptureRun._build_preview_payload(preview_packets, self._processor)
        dominant_pitch_fft_hz = self._dominant_frequency(preview_payload.get("fft", []), "pitch_mag")
        dominant_cmd_fft_hz = self._dominant_frequency(preview_payload.get("fft", []), "cmd_mag")

        return {
            "duration_s": round(duration_s, 3),
            "packet_count": capture_run.packet_count,
            "max_abs_pitch_deg": round(max_abs_pitch_deg, 3),
            "max_abs_cmd_final": round(max_abs_cmd_final, 4),
            "max_abs_steer": round(max_abs_steer, 4),
            "max_encoder_age_ms": int(max_encoder_age_ms),
            "average_loop_hz": round(average_loop_hz, 2),
            "saturation_ratio": round(saturation_ratio, 4),
            "dominant_pitch_fft_hz": dominant_pitch_fft_hz,
            "dominant_cmd_fft_hz": dominant_cmd_fft_hz,
        }

    def _load_recent_manifests(self) -> list[dict[str, Any]]:
        manifests: list[dict[str, Any]] = []
        for manifest_path in sorted(self._capture_root.glob("capture_*/*.json"), reverse=True):
            try:
                manifests.append(CaptureRun.load_manifest(manifest_path))
            except (OSError, ValueError, TypeError, json.JSONDecodeError):
                continue
        return manifests[: self.MAX_RECENT_CAPTURES]

    @staticmethod
    def _dominant_frequency(rows: list[dict[str, Any]], magnitude_key: str) -> float | None:
        if not rows:
            return None
        dominant_row = max(rows, key=lambda row: float(row.get(magnitude_key, 0.0) or 0.0))
        dominant_frequency = dominant_row.get("freq_hz")
        return None if dominant_frequency is None else round(float(dominant_frequency), 3)

    @staticmethod
    def _should_start_from_packet(packet: TelemetryPacket, capture_mode: str) -> bool:
        if capture_mode == CaptureManager.AUTO_MODE:
            return False
        return (
            max(abs(packet.step_hz_l), abs(packet.step_hz_r)) > 50
            or max(abs(packet.motor_rpm_l), abs(packet.motor_rpm_r)) > 8.0
            or max(abs(packet.cmd_final), abs(packet.left_postclip), abs(packet.right_postclip)) > 0.03
        )

    @staticmethod
    def _should_start_from_log(message_text: str, capture_mode: str) -> bool:
        lowered = message_text.lower()
        if capture_mode == CaptureManager.AUTO_MODE:
            return "balancer: started" in lowered or "balancing started" in lowered
        return any(token in lowered for token in ["balance", "balancing", "motors enabled", "controller started", "control started"]) and any(
            token in lowered for token in ["start", "started", "running", "enabled", "active"]
        )

    @staticmethod
    def _stop_reason_from_log(message_text: str, capture_mode: str) -> str | None:
        lowered = message_text.lower()
        if any(token in lowered for token in ["fall", "fallen", "tip", "tipped"]):
            return "fall_detected"
        if any(token in lowered for token in ["fault", "panic", "abort", "error"]):
            return "error_detected"
        if capture_mode != CaptureManager.AUTO_MODE:
            return None
        if any(token in lowered for token in ["balancer: stopped", "stop balancing", "balancing stopped", "controller stopped", "motors disabled", "disabling motors"]):
            return "robot_stopped"
        return None

    def _should_auto_stop_from_packet(self, packet: TelemetryPacket, capture_mode: str) -> bool:
        if capture_mode != self.AUTO_MODE:
            return False
        fallen = abs(packet.pitch_deg) >= self.FALL_STOP_ANGLE_DEG
        inactive_cmd = max(abs(packet.cmd_final), abs(packet.pid_out)) <= self.FALL_STOP_SETTLE_CMD_ABS
        if fallen and inactive_cmd:
            candidate_t_ms = self._fall_auto_stop_state.get("candidate_t_ms")
            if candidate_t_ms is None:
                self._fall_auto_stop_state["candidate_t_ms"] = packet.timestamp_ms
                return False
            if packet.timestamp_ms - candidate_t_ms >= self.FALL_STOP_HOLD_MS:
                self._fall_auto_stop_state["triggered"] = True
                self._fall_auto_stop_state["trigger_t_ms"] = packet.timestamp_ms
                return True
        else:
            self._fall_auto_stop_state["candidate_t_ms"] = None
        return False

    @classmethod
    def _normalize_capture_mode(cls, capture_mode: str | None) -> str:
        normalized_mode = (capture_mode or cls.MANUAL_MODE).strip().lower()
        if normalized_mode not in {cls.MANUAL_MODE, cls.AUTO_MODE}:
            raise ValueError(f"Unsupported capture mode: {capture_mode}")
        return normalized_mode

    @staticmethod
    def _build_fall_auto_stop_state() -> dict[str, Any]:
        return {"candidate_t_ms": None, "triggered": False, "trigger_t_ms": None}

    @staticmethod
    def _now_unix_ms() -> int:
        return time.time_ns() // 1_000_000