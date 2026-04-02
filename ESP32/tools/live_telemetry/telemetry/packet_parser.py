from __future__ import annotations

from typing import Any
import struct

from .constants import PACKET_MAGIC, TELEMETRY_FMT_V3, TELEMETRY_FMT_V5, TELEMETRY_FMT_V6, TELEMETRY_SIZE_V3, TELEMETRY_SIZE_V5, TELEMETRY_SIZE_V6
from .packet import TelemetryPacket


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
