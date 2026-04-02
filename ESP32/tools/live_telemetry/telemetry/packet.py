from __future__ import annotations

from dataclasses import dataclass
from typing import Any


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
