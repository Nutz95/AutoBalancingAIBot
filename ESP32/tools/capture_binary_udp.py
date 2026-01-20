import socket
import struct
import csv
import time
import signal
import sys
import argparse

# Match TelemetryPacket in include/binary_telemetry.h
# struct TelemetryPacket {
#     uint32_t timestamp_ms;
#     float pitch, pitch_rate, speed_l, speed_r, pos_l, pos_r;
#     float u_pitch, u_pos, u_speed, u_yaw;
#     float pwm_l, pwm_r;
#     float t_fusion, t_lqr, t_logging;
#     float cpu_load_0, cpu_load_1;
#     float target_pos, target_speed, target_yaw;
#     float batt_v;
#     float reserved[4];
# };
# Pack format: < (little endian) I (uint32) 25f (25 floats)
TELEMETRY_FMT = "<I25f"
TELEMETRY_SIZE = struct.calcsize(TELEMETRY_FMT)

COLUMN_NAMES = [
    "timestamp_ms", "pitch", "pitch_rate", "speed_l", "speed_r", "pos_l", "pos_r",
    "u_pitch", "u_pos", "u_speed", "u_yaw", "pwm_l", "pwm_r",
    "t_fusion", "t_lqr", "t_logging", "cpu_load_0", "cpu_load_1",
    "target_pos", "target_speed", "target_yaw", "batt_v",
    "r1", "r2", "r3", "r4"
]

def main():
    parser = argparse.ArgumentParser(description="Capture binary UDP telemetry from ESP32")
    parser.add_argument("--port", type=int, default=8888, help="UDP port to listen on")
    parser.add_argument("--out", type=str, default="artifacts/binary_telemetry.csv", help="Output CSV file")
    args = parser.parse_args()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("0.0.0.0", args.port))
    sock.settimeout(1.0)

    print(f"Listening for UDP telemetry on port {args.port}...")
    print(f"Saving to {args.out}")

    with open(args.out, "w", newline='') as f:
        writer = csv.writer(f)
        writer.writerow(COLUMN_NAMES)

        count = 0
        start_time = time.time()
        
        try:
            while True:
                try:
                    data, addr = sock.recvfrom(512)
                    if len(data) == TELEMETRY_SIZE:
                        values = struct.unpack(TELEMETRY_FMT, data)
                        writer.writerow(values)
                        count += 1
                        
                        if count % 100 == 0:
                            elapsed = time.time() - start_time
                            rate = count / elapsed if elapsed > 0 else 0
                            # Print summary: Pitch, CPU0, CPU1, T_Fusion, T_LQR, T_Log
                            # values[1] is pitch, [16] is cpu0, [17] is cpu1, [13] fusion, [14] lqr, [15] log
                            print(f"\rCaptured {count} packets ({rate:.1f} Hz) | Pitch: {values[1]:6.2f} | CPU: {values[16]:4.1f}%/{values[17]:4.1f}% | Latency: F={values[13]:.0f}us L={values[14]:.0f}us Log={values[15]:.0f}us", end="")
                            f.flush()
                except socket.timeout:
                    continue
        except KeyboardInterrupt:
            print("\nStopped.")

if __name__ == "__main__":
    main()
