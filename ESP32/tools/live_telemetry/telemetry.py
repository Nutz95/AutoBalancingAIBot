from __future__ import annotations

import cmath
import math
import struct
from dataclasses import dataclass
from typing import Any

TELEMETRY_FMT_V6 = "<2I6f3f3ff2i8I6f4I6fI2f2I"
TELEMETRY_FMT_V5 = "<2I6f3f3ff2i8I6f4I6fI2f2I2i"
TELEMETRY_FMT_V3 = "<2I6f3f3ff2i8I6f4I6fI"
TELEMETRY_SIZE_V6 = struct.calcsize(TELEMETRY_FMT_V6)
TELEMETRY_SIZE_V5 = struct.calcsize(TELEMETRY_FMT_V5)
TELEMETRY_SIZE_V3 = struct.calcsize(TELEMETRY_FMT_V3)
PACKET_MAGIC = 0xABBA0001


@dataclass
class TelemetryPacket:
    timestamp_ms: int
    pitch_deg: float
    pid_in_deg: float
    pid_out: float
    iterm: float
    cmd: float
    steer: float
    ax: float
    ay: float
    az: float
    gx: float
    gy: float
    gz: float
    loop_freq_hz: float
    enc_l: int
    enc_r: int
    last_encoder_age_ms: int
    bus_latency_us: int
    ack_pending_left_us: int
    ack_pending_right_us: int
    bus_latency_left_us: int
    bus_latency_right_us: int
    bus_latency_left_age_ms: int
    bus_latency_right_age_ms: int
    lqr_angle: float
    lqr_gyro: float
    lqr_dist: float
    lqr_speed: float
    cpu0_pct: float
    cpu1_pct: float
    prof_f: int
    prof_l: int
    prof_t: int
    prof_log: int
    cmd_raw: float
    cmd_final: float
    left_preclip: float
    right_preclip: float
    left_postclip: float
    right_postclip: float
    sat_flags: int
    motor_rpm_l: float
    motor_rpm_r: float
    step_hz_l: int
    step_hz_r: int

    def to_dict(self, latest_ts_ms: int) -> dict[str, Any]:
        return {
            **self.__dict__,
            "t_s": (self.timestamp_ms - latest_ts_ms) / 1000.0,
        }


class TelemetryPacketParser:
    def parse(self, data: bytes) -> TelemetryPacket | None:
        if len(data) == TELEMETRY_SIZE_V6:
            values = struct.unpack(TELEMETRY_FMT_V6, data)
            if values[0] != PACKET_MAGIC:
                return None
            return self._packet_from_v6(values)

        if len(data) == TELEMETRY_SIZE_V5:
            values = struct.unpack(TELEMETRY_FMT_V5, data)
            if values[0] != PACKET_MAGIC:
                return None
            return self._packet_from_v6(values)

        if len(data) == TELEMETRY_SIZE_V3:
            values = struct.unpack(TELEMETRY_FMT_V3, data)
            if values[0] != PACKET_MAGIC:
                return None
            return self._packet_from_v3(values)

        return None

    @staticmethod
    def _packet_from_v6(values: tuple[Any, ...]) -> TelemetryPacket:
        return TelemetryPacket(
            timestamp_ms=values[1],
            pitch_deg=values[2],
            pid_in_deg=values[3],
            pid_out=values[4],
            iterm=values[5],
            cmd=values[6],
            steer=values[7],
            ax=values[8], ay=values[9], az=values[10],
            gx=values[11], gy=values[12], gz=values[13],
            loop_freq_hz=values[14],
            enc_l=values[15], enc_r=values[16],
            last_encoder_age_ms=values[17],
            bus_latency_us=values[18],
            ack_pending_left_us=values[19],
            ack_pending_right_us=values[20],
            bus_latency_left_us=values[21],
            bus_latency_right_us=values[22],
            bus_latency_left_age_ms=values[23],
            bus_latency_right_age_ms=values[24],
            lqr_angle=values[25],
            lqr_gyro=values[26],
            lqr_dist=values[27],
            lqr_speed=values[28],
            cpu0_pct=values[29],
            cpu1_pct=values[30],
            prof_f=values[31],
            prof_l=values[32],
            prof_t=values[33],
            prof_log=values[34],
            cmd_raw=values[35],
            cmd_final=values[36],
            left_preclip=values[37],
            right_preclip=values[38],
            left_postclip=values[39],
            right_postclip=values[40],
            sat_flags=values[41],
            motor_rpm_l=values[42],
            motor_rpm_r=values[43],
            step_hz_l=values[44],
            step_hz_r=values[45],
        )

    @staticmethod
    def _packet_from_v3(values: tuple[Any, ...]) -> TelemetryPacket:
        return TelemetryPacket(
            timestamp_ms=values[1],
            pitch_deg=values[2],
            pid_in_deg=values[3],
            pid_out=values[4],
            iterm=values[5],
            cmd=values[6],
            steer=values[7],
            ax=values[8], ay=values[9], az=values[10],
            gx=values[11], gy=values[12], gz=values[13],
            loop_freq_hz=values[14],
            enc_l=values[15], enc_r=values[16],
            last_encoder_age_ms=values[17],
            bus_latency_us=values[18],
            ack_pending_left_us=values[19],
            ack_pending_right_us=values[20],
            bus_latency_left_us=values[21],
            bus_latency_right_us=values[22],
            bus_latency_left_age_ms=values[23],
            bus_latency_right_age_ms=values[24],
            lqr_angle=values[25],
            lqr_gyro=values[26],
            lqr_dist=values[27],
            lqr_speed=values[28],
            cpu0_pct=values[29],
            cpu1_pct=values[30],
            prof_f=values[31],
            prof_l=values[32],
            prof_t=values[33],
            prof_log=values[34],
            cmd_raw=values[35],
            cmd_final=values[36],
            left_preclip=values[37],
            right_preclip=values[38],
            left_postclip=values[39],
            right_postclip=values[40],
            sat_flags=values[41],
            motor_rpm_l=0.0,
            motor_rpm_r=0.0,
            step_hz_l=0,
            step_hz_r=0,
        )


class TelemetrySignalProcessor:
    def enrich_samples(self, samples: list[dict[str, Any]]) -> list[dict[str, Any]]:
        if not samples:
            return samples

        first_avg = (samples[0]["enc_l"] + samples[0]["enc_r"]) / 2.0
        first_diff = (samples[0]["enc_l"] - samples[0]["enc_r"]) / 2.0
        loop_ema = float(samples[0].get("loop_freq_hz", 0.0) or 0.0)
        prev: dict[str, Any] | None = None

        for sample in samples:
            avg_enc = (sample["enc_l"] + sample["enc_r"]) / 2.0
            diff_enc = (sample["enc_l"] - sample["enc_r"]) / 2.0
            sample["enc_avg_rel"] = avg_enc - first_avg
            sample["enc_diff_rel"] = diff_enc - first_diff
            sample["rpm_avg"] = (sample["motor_rpm_l"] + sample["motor_rpm_r"]) / 2.0
            sample["rpm_diff"] = sample["motor_rpm_l"] - sample["motor_rpm_r"]
            sample["step_avg"] = (sample["step_hz_l"] + sample["step_hz_r"]) / 2.0
            sample["step_diff"] = sample["step_hz_l"] - sample["step_hz_r"]

            loop_hz = float(sample.get("loop_freq_hz", 0.0) or 0.0)
            loop_ema = (0.85 * loop_ema) + (0.15 * loop_hz)
            sample["loop_freq_hz_ema"] = loop_ema

            motion_active = (
                max(abs(sample["step_hz_l"]), abs(sample["step_hz_r"])) > 50
                or max(abs(sample["motor_rpm_l"]), abs(sample["motor_rpm_r"])) > 8.0
                or max(abs(sample["cmd_final"]), abs(sample["left_postclip"]), abs(sample["right_postclip"])) > 0.03
            )
            sample["motion_active"] = motion_active
            sample["encoder_age_plot_ms"] = (
                min(float(sample["last_encoder_age_ms"]), 300.0)
                if motion_active and sample["last_encoder_age_ms"] < 5000
                else None
            )

            if prev is None:
                sample["enc_vel_proxy"] = 0.0
                sample["pitch_rate_est_dps"] = 0.0
            else:
                dt_s = max(1e-3, (sample["timestamp_ms"] - prev["timestamp_ms"]) / 1000.0)
                sample["enc_vel_proxy"] = (avg_enc - prev["_avg_enc"]) / dt_s
                sample["pitch_rate_est_dps"] = (sample["pitch_deg"] - prev["pitch_deg"]) / dt_s

            sample["_avg_enc"] = avg_enc
            prev = sample

        for sample in samples:
            sample.pop("_avg_enc", None)
        return samples

    def compute_fft(self, samples: list[dict[str, Any]], max_freq_hz: float = 25.0) -> dict[str, Any]:
        points = self._resample_points(samples, target=128)
        if len(points) < 32:
            return {"samples": [], "x_max_hz": max_freq_hz, "sample_rate_hz": None}

        dt_values = [
            (points[i]["timestamp_ms"] - points[i - 1]["timestamp_ms"]) / 1000.0
            for i in range(1, len(points))
            if points[i]["timestamp_ms"] > points[i - 1]["timestamp_ms"]
        ]
        if not dt_values:
            return {"samples": [], "x_max_hz": max_freq_hz, "sample_rate_hz": None}

        dt_avg = sum(dt_values) / len(dt_values)
        if dt_avg <= 0.0:
            return {"samples": [], "x_max_hz": max_freq_hz, "sample_rate_hz": None}

        fs = 1.0 / dt_avg
        display_max_hz = max(6.0, min(max_freq_hz, fs * 0.25))
        n = len(points)
        max_k = min(n // 2, int(display_max_hz * n / fs))
        if max_k <= 1:
            return {"samples": [], "x_max_hz": display_max_hz, "sample_rate_hz": fs}

        window = [0.5 - 0.5 * math.cos((2.0 * math.pi * i) / (n - 1)) for i in range(n)]
        pitch = [points[i]["pitch_deg"] for i in range(n)]
        cmd = [points[i]["cmd_final"] for i in range(n)]
        rpm = [points[i]["rpm_avg"] for i in range(n)]
        pitch_w = [v * w for v, w in zip(self._detrend(pitch), window)]
        cmd_w = [v * w for v, w in zip(self._detrend(cmd), window)]
        rpm_w = [v * w for v, w in zip(self._detrend(rpm), window)]

        rows: list[dict[str, float]] = []
        for k in range(1, max_k + 1):
            freq_hz = (k * fs) / n
            if freq_hz > display_max_hz:
                break
            rot = [cmath.exp((-2j * math.pi * k * i) / n) for i in range(n)]
            pitch_sum = sum(pitch_w[i] * rot[i] for i in range(n))
            cmd_sum = sum(cmd_w[i] * rot[i] for i in range(n))
            rpm_sum = sum(rpm_w[i] * rot[i] for i in range(n))
            rows.append(
                {
                    "freq_hz": freq_hz,
                    "pitch_mag": abs(pitch_sum) / n,
                    "cmd_mag": abs(cmd_sum) / n,
                    "rpm_mag": abs(rpm_sum) / n,
                }
            )

        return {
            "samples": rows,
            "x_max_hz": display_max_hz,
            "sample_rate_hz": fs,
        }

    @staticmethod
    def _resample_points(samples: list[dict[str, Any]], target: int = 128) -> list[dict[str, Any]]:
        if len(samples) <= target:
            return samples
        if target <= 1:
            return [samples[-1]]
        out: list[dict[str, Any]] = []
        last_index = len(samples) - 1
        for i in range(target):
            idx = round(i * last_index / (target - 1))
            out.append(samples[idx])
        return out

    @staticmethod
    def _detrend(values: list[float]) -> list[float]:
        n = len(values)
        if n <= 2:
            mean = sum(values) / max(1, n)
            return [v - mean for v in values]
        start = values[0]
        end = values[-1]
        slope = (end - start) / (n - 1)
        return [values[i] - (start + slope * i) for i in range(n)]
