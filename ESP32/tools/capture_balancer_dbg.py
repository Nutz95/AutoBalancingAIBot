#!/usr/bin/env python3
"""
tools/capture_balancer_dbg.py

Connect to the Wi‑Fi console, wait for the balancer to start, capture DRIVE DBG
and BALANCER_DBG lines until the balancer stops, then save a CSV and a plot
showing pitch, accel/gyro (when available) and motor command traces.

Usage:
  python tools/capture_balancer_dbg.py --host 192.168.1.29 --port 2333 -o balancer_log.txt

"""
import argparse
import re
import socket
import select
import struct
import sys
import time
import math
from collections import defaultdict

import numpy as np
import pandas as pd

# TelemetryPacket in include/binary_telemetry.h
# struct TelemetryPacket {
#     uint32_t magic = 0xABBA0001; // 4
#     uint32_t timestamp_ms;       // 4
#     float pitch_deg;             // 4
#     float pid_in_deg;            // 4
#     float pid_out;               // 4
#     float iterm;                 // 4
#     float cmd;                   // 4
#     float steer;                 // 4
#     float ax, ay, az;            // 3*4=12
#     float gx, gy, gz;            // 3*4=12
#     float loop_freq_hz;          // 4
#     int32_t enc_l, enc_r;        // 2*4=8
#     uint32_t bus_latency_us;     // 4
#     uint32_t ack_pending_left_us;// 4
#     uint32_t ack_pending_right_us;// 4
#     float lqr_angle;             // 4
#     float lqr_gyro;              // 4
#     float lqr_dist;              // 4
#     float lqr_speed;             // 4
#     float cpu0_pct;              // 4
#     float cpu1_pct;              // 4
#     uint32_t prof_f;             // 4
#     uint32_t prof_l;             // 4
#     uint32_t prof_t;             // 4
#     uint32_t prof_log;           // 4
# };
# v2 adds last_encoder_age_ms after enc_l/enc_r
TELEMETRY_FMT = "<2I6f3f3ff2i8I6f4I"
TELEMETRY_FMT_V1 = "<2I6f3f3ff2i7I6f4I"
TELEMETRY_FMT_ACK_LR = "<2I6f3f3ff2i3I6f4I"
TELEMETRY_FMT_LEGACY_ACK = "<2I6f3f3ff2i2I6f4I"
TELEMETRY_SIZE = struct.calcsize(TELEMETRY_FMT)
TELEMETRY_SIZE_V1 = struct.calcsize(TELEMETRY_FMT_V1)
TELEMETRY_SIZE_ACK_LR = struct.calcsize(TELEMETRY_FMT_ACK_LR)
TELEMETRY_SIZE_LEGACY_ACK = struct.calcsize(TELEMETRY_FMT_LEGACY_ACK)
UDP_PORT = 8888


DRIVE_RE = re.compile(r"DRIVE DBG t=(?P<t>\d+)ms.*?tgtV=(?P<tgtV>[-0-9.eE]+).*?filtV=(?P<filtV>[-0-9.eE]+).*?pitch_setpoint=(?P<pitch_setpoint>[-0-9.eE]+)deg.*?pitch_setpoint_rate=(?P<pitch_setpoint_rate>[-0-9.eE]+)deg/s.*?pid_in=(?P<pid_in_drive>[-0-9.eE]+)deg.*?pid_rate=(?P<pid_rate_drive>[-0-9.eE]+)deg/s.*?pid_out=(?P<pid_out_drive>[-0-9.eE]+)")
SETDRIVE_RE = re.compile(r"SETDRIVE: t=(?P<t>\d+)ms v_req=(?P<v_req>[-0-9.eE]+) w_req=(?P<w_req>[-0-9.eE]+)")
BAL_RE = re.compile(
    r"BALANCER_DBG t=(?P<t>\d+)ms.*?pitch=(?P<pitch>[-0-9.eE]+)deg.*?pid_in=(?P<pid_in>[-0-9.eE]+)deg.*?pid_out=(?P<pid_out>[-0-9.eE]+).*?iterm=(?P<iterm>[-0-9.eE]+).*?cmd=(?P<cmd>[-0-9.eE]+)(?:.*?steer=(?P<steer>[-0-9.eE]+))?.*?lat=(?P<lat>\d+)us.*?ax=(?P<ax>[-0-9.eE]+).*?ay=(?P<ay>[-0-9.eE]+).*?az=(?P<az>[-0-9.eE]+).*?gx=(?P<gx>[-0-9.eE]+).*?gy=(?P<gy>[-0-9.eE]+).*?gz=(?P<gz>[-0-9.eE]+)(?:.*?lp_hz=(?P<lp_hz>[-0-9.eE]+))?(?:.*?encL=(?P<encL>[-0-9.eE]+))?(?:.*?encR=(?P<encR>[-0-9.eE]+))?(?:.*?termA=(?P<lqr_angle>[-0-9.eE]+))?(?:.*?termG=(?P<lqr_gyro>[-0-9.eE]+))?(?:.*?termD=(?P<lqr_dist>[-0-9.eE]+))?(?:.*?termS=(?P<lqr_speed>[-0-9.eE]+))?(?:.*?prof_f=(?P<prof_f>\d+))?(?:.*?prof_l=(?P<prof_l>\d+))?(?:.*?prof_t=(?P<prof_t>\d+))?"
)

# Motor telemetry (produced by src/motor_telemetry_manager.cpp)
MOTOR_TLM_BOTH_RE = re.compile(
    r"MOTOR: telemetry\s+drv=(?P<drv>\S+)\s+ts_us=(?P<ts_us>\d+)\s+interval=(?P<interval_ms>\d+)ms\s+bus_lat_us=(?P<bus_lat_us>\d+)(?:\s+accel=(?P<accel>\d+))?\s+"
    r"L\(id=(?P<lid>-?\d+)\)\s+enc=(?P<lenc>[-0-9.eE]+)\s+sp=(?P<lsp>[-0-9.eE]+)\s+cmd_age_ms=(?P<lage>-?\d+)\s+"
    r"R\(id=(?P<rid>-?\d+)\)\s+enc=(?P<renc>[-0-9.eE]+)\s+sp=(?P<rsp>[-0-9.eE]+)\s+cmd_age_ms=(?P<rage>-?\d+)"
)
MOTOR_TLM_ONE_RE = re.compile(
    r"MOTOR: telemetry\s+drv=(?P<drv>\S+)\s+ts_us=(?P<ts_us>\d+)\s+interval=(?P<interval_ms>\d+)ms\s+bus_lat_us=(?P<bus_lat_us>\d+)(?:\s+accel=(?P<accel>\d+))?\s+"
    r"id=(?P<id>-?\d+)\s+enc=(?P<enc>[-0-9.eE]+)\s+sp=(?P<sp>[-0-9.eE]+)\s+cmd_age_ms=(?P<age>-?\d+)"
)
CPU_RE = re.compile(r"CPU:\s+t=(?P<t>\d+)ms\s+core0=(?P<cpu0>[-0-9.eE]+)%\s+core1=(?P<cpu1>[-0-9.eE]+)%")
GAIN_RE_PID = re.compile(r"BALANCER: started \(PID\) \(Kp=(?P<kp>[-0-9.eE]+) Ki=(?P<ki>[-0-9.eE]+) Kd=(?P<kd>[-0-9.eE]+)\)")
GAIN_RE_LQR = re.compile(r"BALANCER: started \(LQR\) \(Kp=(?P<kp>[-0-9.eE]+) Kg=(?P<kg>[-0-9.eE]+) Kd=(?P<kd>[-0-9.eE]+) Ks=(?P<ks>[-0-9.eE]+)\)")
FILTER_RE = re.compile(r"FUSION: active filter=(?P<filter>[A-Za-z0-9_]+)|\[Filter: (?P<filter2>[A-Za-z0-9_]+)\]")


def safe_float(v):
    if v is None:
        return 0.0
    try:
        return float(v)
    except ValueError:
        return 0.0

def _maybe_update_motor_sync(motor_sync, bal_t_ms=None, motor_ts_us=None):
    if bal_t_ms is not None and motor_sync.get('first_bal_t_ms') is None:
        motor_sync['first_bal_t_ms'] = int(bal_t_ms)
    if motor_ts_us is not None and motor_sync.get('first_motor_ts_us') is None:
        motor_sync['first_motor_ts_us'] = int(motor_ts_us)

    if motor_sync.get('offset_ms') is None:
        fb = motor_sync.get('first_bal_t_ms')
        fm = motor_sync.get('first_motor_ts_us')
        if fb is not None and fm is not None:
            motor_sync['offset_ms'] = int(fb - (fm / 1000.0))


def parse_line(line, rows, motor_sync):
    m = DRIVE_RE.search(line)
    if m:
        d = {k: safe_float(v) for k, v in m.groupdict().items()}
        t = int(float(d.pop('t')))
        rows['drive'][t].update(d)
        return
    m = SETDRIVE_RE.search(line)
    if m:
        d = {k: safe_float(v) for k, v in m.groupdict().items()}
        t = int(float(d.pop('t')))
        rows['setdrive'][t].update(d)
        return
    m = BAL_RE.search(line)
    if m:
        d = {k: safe_float(v) for k, v in m.groupdict().items()}
        t = int(float(d.pop('t')))
        _maybe_update_motor_sync(motor_sync, bal_t_ms=t)
        rows['bal'][t].update(d)
        return

    # Motor telemetry (driver status/health)
    m = MOTOR_TLM_BOTH_RE.search(line)
    if m:
        gd = m.groupdict()
        ts_us = int(gd['ts_us'])
        _maybe_update_motor_sync(motor_sync, motor_ts_us=ts_us)
        off = motor_sync.get('offset_ms')
        t = int(ts_us / 1000.0 + (off if off is not None else 0))
        rows['motor'][t].update({
            'motor_bus_lat_us': safe_float(gd['bus_lat_us']),
            'motor_l_sp': safe_float(gd['lsp']),
            'motor_r_sp': safe_float(gd['rsp']),
            'motor_l_enc': safe_float(gd['lenc']),
            'motor_r_enc': safe_float(gd['renc']),
            'motor_l_cmd_age_ms': safe_float(gd['lage']),
            'motor_r_cmd_age_ms': safe_float(gd['rage']),
        })
        if gd.get('accel') is not None:
            rows['motor'][t].update({'motor_accel': safe_float(gd['accel'])})
        return

    m = MOTOR_TLM_ONE_RE.search(line)
    if m:
        gd = m.groupdict()
        ts_us = int(gd['ts_us'])
        _maybe_update_motor_sync(motor_sync, motor_ts_us=ts_us)
        off = motor_sync.get('offset_ms')
        t = int(ts_us / 1000.0 + (off if off is not None else 0))
        mid = int(gd['id'])
        # Keep generic single-side fields; plots mainly use BOTH mode.
        rows['motor'][t].update({
            'motor_bus_lat_us': safe_float(gd['bus_lat_us']),
            f'motor_{mid}_sp': safe_float(gd['sp']),
            f'motor_{mid}_enc': safe_float(gd['enc']),
            f'motor_{mid}_cmd_age_ms': safe_float(gd['age']),
        })
        if gd.get('accel') is not None:
            rows['motor'][t].update({'motor_accel': safe_float(gd['accel'])})
        return

    # CPU load telemetry
    m = CPU_RE.search(line)
    if m:
        gd = m.groupdict()
        t = int(safe_float(gd['t']))
        rows['cpu'][t].update({
            'cpu_core0_pct': safe_float(gd['cpu0']),
            'cpu_core1_pct': safe_float(gd['cpu1']),
        })
        return
    # try simple CSV-style imu line (legacy TUNING), detect by commas and numeric start
    if ',' in line:
        parts = [p.strip() for p in line.split(',')]
        try:
            t0 = safe_float(parts[0])
            # guess layout similar to TUNING CSV: timestamp_ms, pitch_deg, pitch_rad, pitch_rate_deg, pitch_rate_rad, ax,ay,az,gx,gy,gz,...
            if len(parts) >= 11:
                t = int(t0)
                ax = safe_float(parts[5])
                ay = safe_float(parts[6])
                az = safe_float(parts[7])
                gx = safe_float(parts[8])
                gy = safe_float(parts[9])
                gz = safe_float(parts[10])
                rows['imu'][t] = {'ax': ax, 'ay': ay, 'az': az, 'gx': gx, 'gy': gy, 'gz': gz}
        except Exception:
            pass


def parse_binary(data, rows, motor_sync):
    last_encoder_age_ms = 0
    lat_index = 17
    if len(data) == TELEMETRY_SIZE:
        v = struct.unpack(TELEMETRY_FMT, data)
        last_encoder_age_ms = v[17]
        lat_index = 18
        ack_left = v[19]
        ack_right = v[20]
        bus_lat_left = v[21]
        bus_lat_right = v[22]
        bus_lat_left_age = v[23]
        bus_lat_right_age = v[24]
        lqr_index = 25
    elif len(data) == TELEMETRY_SIZE_V1:
        v = struct.unpack(TELEMETRY_FMT_V1, data)
        ack_left = v[18]
        ack_right = v[19]
        bus_lat_left = v[20]
        bus_lat_right = v[21]
        bus_lat_left_age = v[22]
        bus_lat_right_age = v[23]
        lqr_index = 24
    elif len(data) == TELEMETRY_SIZE_ACK_LR:
        v = struct.unpack(TELEMETRY_FMT_ACK_LR, data)
        ack_left = v[18]
        ack_right = v[19]
        bus_lat_left = 0
        bus_lat_right = 0
        bus_lat_left_age = 0
        bus_lat_right_age = 0
        lqr_index = 20
    elif len(data) == TELEMETRY_SIZE_LEGACY_ACK:
        v = struct.unpack(TELEMETRY_FMT_LEGACY_ACK, data)
        ack_left = v[18]
        ack_right = 0
        bus_lat_left = 0
        bus_lat_right = 0
        bus_lat_left_age = 0
        bus_lat_right_age = 0
        lqr_index = 19
    else:
        return False
    if v[0] != 0xABBA0001:
        return False
    
    t = int(v[1])
    _maybe_update_motor_sync(motor_sync, bal_t_ms=t)
    
    d = {
        'pitch': v[2],
        'pid_in': v[3],
        'pid_out': v[4],
        'iterm': v[5],
        'cmd': v[6],
        'steer': v[7],
        'ax': v[8], 'ay': v[9], 'az': v[10],
        'gx': v[11], 'gy': v[12], 'gz': v[13],
        'lp_hz': v[14],
        'encL': v[15], 'encR': v[16],
        'last_encoder_age_ms': last_encoder_age_ms,
        'lat': v[lat_index],
        'ack_pending_l_us': ack_left,
        'ack_pending_r_us': ack_right,
        'bus_lat_l_us': bus_lat_left,
        'bus_lat_r_us': bus_lat_right,
        'bus_lat_l_age_ms': bus_lat_left_age,
        'bus_lat_r_age_ms': bus_lat_right_age,
        'lqr_angle': v[lqr_index],
        'lqr_gyro': v[lqr_index + 1],
        'lqr_dist': v[lqr_index + 2],
        'lqr_speed': v[lqr_index + 3],
        'cpu_core0_pct': v[lqr_index + 4],
        'cpu_core1_pct': v[lqr_index + 5],
        'prof_f': v[lqr_index + 6],
        'prof_l': v[lqr_index + 7],
        'prof_t': v[lqr_index + 8],
        'prof_log': v[lqr_index + 9]
    }
    rows['bal'][t].update(d)
    return True


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--host')
    p.add_argument('--port', type=int, default=2333)
    p.add_argument('--name', help='Base name for output files (e.g. "capture10" will create .txt, .csv, .png)')
    p.add_argument('-o', '--out', default='balancer_capture.txt')
    p.add_argument('--csv', default='balancer_capture.csv')
    p.add_argument('--png', default='balancer_capture.png')
    p.add_argument('--timeout', type=int, default=10, help='seconds to wait for connection')
    p.add_argument('--from-log', help='Parse an existing log file')
    p.add_argument('--motor-telemetry-ms', type=int, default=0,
                   help='If >0, send "MOTOR TELEMETRY ALL <ms>" at capture start and stop it at capture end.')
    p.add_argument('--motor-accel', type=int, default=None,
                   help='If set, send "MOTOR ACCEL <0-255>" at capture start (requires new firmware).')
    p.add_argument('--mks-baud', type=int, default=256000,
                   help='MKS RS485 baud rate (used to estimate transfer time).')
    p.add_argument('--cpu-telemetry-ms', type=int, default=0,
                   help='If >0, send "SYS CPU STREAM <ms>" at capture start and stop it at capture end.')
    p.add_argument('--no-ffill', dest='ffill', action='store_false', help='Disable forward-fill')
    p.set_defaults(ffill=True)
    args = p.parse_args()

    if args.name:
        args.out = f"{args.name}.txt"
        args.csv = f"{args.name}.csv"
        args.png = f"{args.name}.png"

    rows = {'drive': defaultdict(dict), 'bal': defaultdict(dict), 'setdrive': defaultdict(dict), 'imu': dict(), 'motor': defaultdict(dict), 'cpu': defaultdict(dict)}
    motor_sync = {'first_bal_t_ms': None, 'first_motor_ts_us': None, 'offset_ms': None}
    gain_info = "Unknown Gains"
    gains = {'kp': 0.0, 'ki': 0.0, 'kd': 0.0, 'kg': 0.0, 'ks': 0.0}
    strategy_mode = 'PID' # Default
    filter_info = "Unknown Filter"
    trim_info = {"val": 0.0, "type": "dynamic", "detected": False}
    
    if args.from_log:
        print(f"Parsing log file: {args.from_log}")
        with open(args.from_log, 'r', encoding='utf8', errors='ignore') as f:
            for line in f:
                line = line.strip()
                m_gain_pid = GAIN_RE_PID.search(line)
                m_gain_lqr = GAIN_RE_LQR.search(line)
                if m_gain_pid:
                    gains = {k: float(m_gain_pid.group(k)) for k in ['kp', 'ki', 'kd']}
                    gain_info = f"PID: Kp={gains['kp']} Ki={gains['ki']} Kd={gains['kd']}"
                    strategy_mode = 'PID'
                elif m_gain_lqr:
                    gains = {k: float(m_gain_lqr.group(k)) for k in ['kp', 'kg', 'kd', 'ks']}
                    gain_info = f"LQR: Kp={gains['kp']} Kg={gains['kg']} Kd={gains['kd']} Ks={gains['ks']}"
                    strategy_mode = 'LQR'
                m_filter = FILTER_RE.search(line)
                if m_filter:
                    filter_info = m_filter.group('filter') or m_filter.group('filter2')
                
                # Detect trim info
                if "using calibrated trim =" in line:
                    m = re.search(r"using calibrated trim = ([-0-9.]+) deg", line)
                    if m:
                        trim_info = {"val": float(m.group(1)), "type": "calibrated", "detected": True}
                elif "dynamic trim captured =" in line:
                    m = re.search(r"dynamic trim captured = ([-0-9.]+) deg", line)
                    if m:
                        trim_info = {"val": float(m.group(1)), "type": "dynamic", "detected": True}

                parse_line(line, rows, motor_sync)
    else:
        if not args.host:
            print("Error: --host required for live capture.")
            return

        sock = None
        udp_sock = None
        try:
            print(f"Connecting to {args.host}:{args.port}...")
            sock = socket.create_connection((args.host, args.port), timeout=args.timeout)
            sock.settimeout(0.5) # Small timeout to allow KeyboardInterrupt to fire

            # Setup UDP socket for binary telemetry
            try:
                udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                # Use SO_REUSEADDR to avoid "Port already in use" if we restart quickly
                udp_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                udp_sock.bind(("0.0.0.0", UDP_PORT))
                udp_sock.setblocking(False)
                print(f">>> Prepared UDP listener on port {UDP_PORT}")
            except Exception as e:
                print(f">>> ERROR: Could not bind UDP port {UDP_PORT}: {e}")
                print(">>> Continuing with TCP text logs only.")
                udp_sock = None

            # Automatically enable UDP telemetry first to reduce CPU load from text logs
            sock.sendall(b"SYS TELEM UDP AUTO\n")
            time.sleep(0.1)
            # We no longer enable text logs by default as binary telemetry is much faster.
            # sock.sendall(b"LOG ENABLE BALANCER\n")
            print(">>> Sent SYS TELEM UDP AUTO (Binary telemetry)")

            if args.motor_telemetry_ms and args.motor_telemetry_ms > 0:
                try:
                    # Motor telemetry logs are gated by the MOTOR channel.
                    sock.sendall(b"LOG ENABLE MOTOR\n")
                    time.sleep(0.02)

                    if args.motor_accel is not None:
                        cmd = f"MOTOR ACCEL {int(args.motor_accel)}\n"
                        sock.sendall(cmd.encode('utf8'))
                        time.sleep(0.02)
                        sock.sendall(b"MOTOR STATUS\n")
                        time.sleep(0.02)

                    cmd = f"MOTOR TELEMETRY ALL {args.motor_telemetry_ms}\n"
                    sock.sendall(cmd.encode('utf8'))
                    print(f">>> Enabled MOTOR TELEMETRY ALL {args.motor_telemetry_ms}ms")
                except Exception as e:
                    print(f"WARN: failed to enable motor telemetry: {e}")

            if args.cpu_telemetry_ms and args.cpu_telemetry_ms > 0:
                try:
                    cmd = f"SYS CPU STREAM {int(args.cpu_telemetry_ms)}\n"
                    sock.sendall(cmd.encode('utf8'))
                    print(f">>> Enabled SYS CPU STREAM {args.cpu_telemetry_ms}ms")
                except Exception as e:
                    print(f"WARN: failed to enable CPU telemetry: {e}")
            
            print("Listening for Telemetry... (Waiting for 'BALANCER: started' or first packet)")
            
            start_time = time.time()
            started = False
            line_buf = ""
            recent_lines = [] 
            udp_packet_count = 0
            
            # Use specific list for select, filter out None
            select_list = [sock]
            if udp_sock: select_list.append(udp_sock)

            with open(args.out, 'w', encoding='utf8') as out_fh:
                while True:
                    # Wait for data from either socket
                    r, _, _ = select.select(select_list, [], [], 0.05)
                    
                    if udp_sock and udp_sock in r:
                        try:
                            # Use a slightly larger buffer for safety
                            data, addr = udp_sock.recvfrom(1024)
                            if parse_binary(data, rows, motor_sync):
                                udp_packet_count += 1
                                if not started:
                                    print(f">>> Binary UDP stream detected from {addr}!")
                                    started = True
                                
                                if udp_packet_count % 500 == 0:
                                    # Print status without flooding
                                    ts = int(struct.unpack("<I", data[4:8])[0])
                                    print(f"\r>>> UDP packets: {udp_packet_count} (last t={ts}ms)  ", end="", flush=True)
                            else:
                                if udp_packet_count == 0:
                                    print(f">>> Received invalid UDP packet (size={len(data)})")
                        except Exception as e:
                            # On Windows, recvfrom can raise ConnectionResetError if the target port isn't reachable
                            pass

                    if sock in r:
                        try:
                            raw_data = sock.recv(4096)
                            if not raw_data:
                                if started:
                                    print("\n>>> TCP connection closed by peer. Continuing with UDP...")
                                    select_list.remove(sock)
                                    sock.close()
                                    sock = None
                                    continue
                                else:
                                    print("\n>>> TCP connection closed before telemetry started.")
                                    break
                            data = raw_data.decode('utf8', errors='ignore')
                            line_buf += data
                        except (socket.timeout, ConnectionResetError):
                            if started:
                                print("\n>>> TCP connection reset. Continuing with UDP...")
                                if sock in select_list: select_list.remove(sock)
                                sock.close()
                                sock = None
                                continue
                            else:
                                print("\n>>> TCP connection reset by ESP32 (possibly rebooting?).")
                                break
                    while '\n' in line_buf:
                        line, line_buf = line_buf.split('\n', 1)
                        line = line.strip()
                        if not line:
                            continue
                        
                        out_fh.write(line + '\n')
                        out_fh.flush()

                        recent_lines.append(line)
                        if len(recent_lines) > 100: recent_lines.pop(0)

                        # Look for gains and filter in any line received
                        m_gain_pid = GAIN_RE_PID.search(line)
                        m_gain_lqr = GAIN_RE_LQR.search(line)
                        if m_gain_pid:
                            gains = {k: float(m_gain_pid.group(k)) for k in ['kp', 'ki', 'kd']}
                            gain_info = f"PID: Kp={gains['kp']} Ki={gains['ki']} Kd={gains['kd']}"
                            strategy_mode = 'PID'
                            print(f">>> Detected Strategy: {gain_info}")
                        elif m_gain_lqr:
                            gains = {k: float(m_gain_lqr.group(k)) for k in ['kp', 'kg', 'kd', 'ks']}
                            gain_info = f"LQR: Kp={gains['kp']} Kg={gains['kg']} Kd={gains['kd']} Ks={gains['ks']}"
                            strategy_mode = 'LQR'
                            print(f">>> Detected Strategy: {gain_info}")

                        m_filter = FILTER_RE.search(line)
                        if m_filter:
                            filter_info = m_filter.group('filter') or m_filter.group('filter2')
                            print(f">>> Detected Filter: {filter_info}")

                        # Detect balancer start to trigger clearing of old data
                        if "BALANCER: started" in line or ("BALANCER_DBG" in line and not started):
                            print(">>> Balancer START detected! Clearing pre-start data...")
                            
                            # Reset data collection to ensure we only save what happens after START
                            for k in rows: rows[k].clear()
                            motor_sync = {'bal': None, 'motor': None, 'offset_ms': None}
                            udp_packet_count = 0

                            # Re-parse recent history to catch metadata if missed
                            for prev_line in recent_lines:
                                m_f = FILTER_RE.search(prev_line)
                                if m_f: 
                                    filter_info = m_f.group('filter') or m_f.group('filter2')
                                
                                m_g_pid = GAIN_RE_PID.search(prev_line)
                                m_g_lqr = GAIN_RE_LQR.search(prev_line)
                                if m_g_pid:
                                    gains = {k: float(m_g_pid.group(k)) for k in ['kp', 'ki', 'kd']}
                                    gain_info = f"PID: Kp={gains['kp']} Ki={gains['ki']} Kd={gains['kd']}"
                                    strategy_mode = 'PID'
                                elif m_g_lqr:
                                    gains = {k: float(m_g_lqr.group(k)) for k in ['kp', 'kg', 'kd', 'ks']}
                                    gain_info = f"LQR: Kp={gains['kp']} Kg={gains['kg']} Kd={gains['kd']} Ks={gains['ks']}"
                                    strategy_mode = 'LQR'
                            started = True

                        # Detect trim info
                        if "using calibrated trim =" in line:
                            m = re.search(r"using calibrated trim = ([-0-9.]+) deg", line)
                            if m:
                                trim_info = {"val": float(m.group(1)), "type": "calibrated", "detected": True}
                                print(f">>> {trim_info['type'].upper()} TRIM detected: {trim_info['val']} deg")
                        elif "dynamic trim captured =" in line:
                            m = re.search(r"dynamic trim captured = ([-0-9.]+) deg", line)
                            if m:
                                trim_info = {"val": float(m.group(1)), "type": "dynamic", "detected": True}
                                print(f">>> {trim_info['type'].upper()} TRIM detected: {trim_info['val']} deg")

                        should_parse = started
                        if not should_parse:
                            if line.startswith("CPU:") or line.startswith("MOTOR:"):
                                should_parse = True

                        if should_parse:
                            parse_line(line, rows, motor_sync)
                            if "BALANCER: stopped" in line:
                                # Add 1s grace period to ignore old logs from previous runs
                                if time.time() - start_time > 1.0:
                                    print("\n>>> Balancer STOP detected. Ending capture.")
                                    raise StopIteration
        except (KeyboardInterrupt, StopIteration):
            print("\nCapture finished.")
        finally:
            if sock:
                try:
                    # Cleanup telemetry streams
                    sock.sendall(b"SYS TELEM UDP STOP\n")
                    time.sleep(0.05)
                    if args.cpu_telemetry_ms and args.cpu_telemetry_ms > 0:
                        sock.sendall(b"SYS CPU STOP\n")
                        time.sleep(0.05)
                    if args.motor_telemetry_ms and args.motor_telemetry_ms > 0:
                        sock.sendall(b"MOTOR TELEMETRY ALL 0\n")
                        time.sleep(0.05)
                except Exception:
                    pass
                sock.close()
            if udp_sock:
                udp_sock.close()

    # Build DataFrame
    keys = set(rows['drive'].keys()) | set(rows['bal'].keys()) | set(rows['setdrive'].keys()) | set(rows['imu'].keys()) | set(rows['motor'].keys()) | set(rows['cpu'].keys())
    if not keys:
        print("No balancer data found.")
        return
    
    times = sorted(keys)
    recs = []
    for t in times:
        r = {'time_ms': t}
        r.update(rows['imu'].get(t, {}))
        r.update(rows['drive'].get(t, {}))
        r.update(rows['bal'].get(t, {}))
        r.update(rows['setdrive'].get(t, {}))
        r.update(rows['motor'].get(t, {}))
        r.update(rows['cpu'].get(t, {}))
        recs.append(r)
    
    df = pd.DataFrame(recs).sort_values('time_ms')
    
    # Auto-detect strategy mode if not explicitly caught from header
    if strategy_mode == 'PID':
        lqr_cols = ['lqr_angle', 'lqr_gyro', 'lqr_dist', 'lqr_speed']
        if all(c in df.columns for c in lqr_cols):
            # If we have these columns and they contain non-zero data, it's LQR
            if df[lqr_cols].any().any():
                strategy_mode = 'LQR'
                if gain_info == "Unknown Gains":
                    gain_info = "LQR (Back-detected from logs)"

    df.to_csv(args.csv, index=False)
    print(f"Wrote CSV: {args.csv}")

    # Extra stats calculation for display
    stats_text = ""
    # Ticks to meters scale: (math.pi * 0.067) / 51200.0 (67mm wheel)
    t2m = (math.pi * 0.067) / 51200.0
    
    try:
        valid_pitch = df['pitch'].dropna()
        if not valid_pitch.empty:
            mean_p = valid_pitch.mean()
            std_p = valid_pitch.std()
            stats_text += f"Pitch: Mean={mean_p:.2f}°, Std={std_p:.2f}°\n"
            
            # Dominant frequency estimation using zero crossings
            pitch_vals = valid_pitch.values
            pitch_centered = pitch_vals - np.mean(pitch_vals)
            crossings = np.where(np.diff(np.sign(pitch_centered)))[0]
            duration = (df['time_ms'].iloc[-1] - df['time_ms'].iloc[0]) / 1000.0
            if duration > 0:
                freq = len(crossings) / (2 * duration)
                stats_text += f"Est. Oscillation Freq: {freq:.2f} Hz\n"

        if 'encL' in df.columns and 'encR' in df.columns:
            start_enc = (df['encL'].iloc[0] + df['encR'].iloc[0]) / 2
            end_enc = (df['encL'].iloc[-1] + df['encR'].iloc[-1]) / 2
            drift_ticks = end_enc - start_enc
            drift_m = drift_ticks * t2m
            stats_text += f"Avg Encoder Drift: {drift_ticks:.1f} ticks ({drift_m:.3f} m)\n"
            
        if strategy_mode == 'LQR' and 'iterm' in df.columns:
            final_trim = df['iterm'].ffill().iloc[-1]
            stats_text += f"Final Adaptive Trim: {final_trim:.3f}°"
    except Exception as e:
        print(f"Stats calculation failed: {e}")

    # Plot
    try:
        import matplotlib.pyplot as plt
        expected = ['pitch', 'pitch_setpoint', 'pid_out', 'ax', 'ay', 'az', 'gx', 'gy', 'gz']
        for c in expected:
            if c not in df.columns: df[c] = np.nan
        
        for c in df.columns:
            if c != 'time_ms':
                df[c] = pd.to_numeric(df[c], errors='coerce')

        if args.ffill:
            df = df.ffill(limit=10)

        # Calculate PID/LQR components if gains are known
        if strategy_mode == 'PID' and 'pid_in' in df.columns and 'iterm' in df.columns and 'pid_out' in df.columns:
            df['p_term'] = (df['pid_in'] - trim_info['val']) * gains['kp']
            df['d_term'] = df['pid_out'] - df['p_term'] - df['iterm'] # Roughly iterm + d_term
        elif strategy_mode == 'LQR' and 'pitch' in df.columns and 'gy' in df.columns:
            # LQR u = Kp*theta + Kg*gyro - Kd*dist - Ks*speed
            # theta and gyro are in degrees in the log
            # dist is (encL+encR)/2
            # speed is d(dist)/dt
            df['lqr_angle'] = (df['pitch'] - trim_info['val']) * gains.get('kp', 0.0)
            df['lqr_gyro'] = df['gy'] * gains.get('kg', 0.0)
            if 'encL' in df.columns and 'encR' in df.columns:
                df['dist'] = (df['encL'] + df['encR']) / 2.0
                df['dist_err'] = df['dist'] - df['dist'].iloc[0] # Relative to start of capture
                df['lqr_dist'] = -df['dist_err'] * gains.get('kd', 0.0)
                # Velocity estimation (simple)
                dt_s = df['time_ms'].diff() / 1000.0
                df['speed'] = df['dist'].diff() / dt_s
                df['lqr_speed'] = -df['speed'] * gains.get('ks', 0.0)

        time_origin_ms = int(df['time_ms'].iloc[0])
        try:
            bal_keys = list(rows['bal'].keys())
            if bal_keys:
                time_origin_ms = int(min(bal_keys))
        except Exception:
            pass

        t_axis = (df['time_ms'] - time_origin_ms) / 1000.0
        
        # Calculate Gyro Pitch (simple integration for lag comparison)
        if 'gy' in df.columns and len(df) > 1:
            dt_integrated = df['time_ms'].diff() / 1000.0
            
            # Estimate gyro bias from the first 5 samples where the robot is usually still
            gyro_bias_est = df['gy'].head(5).mean()
            if pd.isna(gyro_bias_est): gyro_bias_est = 0.0
            
            # Use the first valid pitch or 0 as starting point for integration
            start_pitch = df['pitch'].ffill().bfill().iloc[0] if not df['pitch'].dropna().empty else 0.0
            gyro_pitch = [start_pitch]
            for i in range(1, len(df)):
                gy_val = df['gy'].iloc[i]
                dt_val = dt_integrated.iloc[i]
                if pd.isna(gy_val) or pd.isna(dt_val):
                    gyro_pitch.append(gyro_pitch[-1])
                else:
                    # Subtract estimated bias and integrate
                    # Note: BNO055 Y is inverted relative to robot pitch in this mapping
                    val = gyro_pitch[-1] - (gy_val - gyro_bias_est) * dt_val
                    gyro_pitch.append(val)
            df['gyro_pitch_est'] = gyro_pitch

        has_motor = any(k.startswith('motor_') for k in df.columns)
        has_cpu = ('cpu_core0_pct' in df.columns) or ('cpu_core1_pct' in df.columns)
        nrows = 8 + (1 if has_cpu else 0) + (1 if has_motor else 0)
        plt.figure(figsize=(18, 30 if has_motor else 28))
        
        # Subplot 1: Pitch
        ax1 = plt.subplot(nrows, 1, 1)
        ax1.plot(t_axis, df['pitch_setpoint'], 'r--', label='Target (User)', alpha=0.7)
        ax1.plot(t_axis, df['pitch'], 'b-', label='Filtered Pitch (deg)', linewidth=2)
        
        if strategy_mode == 'LQR' and 'iterm' in df.columns:
            ax1.plot(t_axis, df['iterm'], 'm:', label='Dynamic Pitch Trim (Adaptive)', linewidth=2)
            
        if 'gyro_pitch_est' in df.columns:
            ax1.plot(t_axis, df['gyro_pitch_est'], 'c--', label='Gyro Integration (Ref)', alpha=0.4)
        
        # Display stats overlay
        if stats_text:
            ax1.text(0.02, 0.95, stats_text, transform=ax1.transAxes, verticalalignment='top', 
                     bbox=dict(boxstyle='round', facecolor='white', alpha=0.8), fontsize=10)

        # Display trim info
        if trim_info['detected']:
             color = 'gray' if trim_info['type'] == 'calibrated' else 'red'
             label = f"Trim ({trim_info['val']} deg - {trim_info['type']})"
             ax1.axhline(y=trim_info['val'], color=color, linestyle=':', label=label, alpha=0.8)

        ax1.set_ylabel('Degrees')
        ax1.legend(loc='upper right')
        ax1.grid(True)
        ax1.set_title(f'Balancer Pitch stability ({filter_info} | {gain_info})')

        # Subplot 2: PID Components (Decomposition)
        ax2 = plt.subplot(nrows, 1, 2, sharex=ax1)
        if strategy_mode == 'PID':
            if 'p_term' in df.columns:
                ax2.plot(t_axis, df['p_term'], label='P contribution (Kp*err)', alpha=0.8)
            if 'iterm' in df.columns:
                ax2.plot(t_axis, df['iterm'], label='I contribution (Ki*sum)', alpha=0.8)
            if 'd_term' in df.columns:
                ax2.plot(t_axis, df['d_term'], label='D contribution (Kd*rate)', alpha=0.8)
            ax2.set_title('PID Decomposition (Influence of each gain)')
        elif strategy_mode == 'LQR':
            if 'lqr_angle' in df.columns:
                ax2.plot(t_axis, df['lqr_angle'], label='Angle (Kp*theta)', alpha=0.8)
            if 'lqr_gyro' in df.columns:
                ax2.plot(t_axis, df['lqr_gyro'], label='Gyro (Kg*rate)', alpha=0.8)
            if 'lqr_dist' in df.columns:
                ax2.plot(t_axis, df['lqr_dist'], label='Dist (Kd*dist)', alpha=0.8)
            if 'lqr_speed' in df.columns:
                ax2.plot(t_axis, df['lqr_speed'], label='Speed (Ks*vel)', alpha=0.8)
            ax2.set_title('LQR Decomposition (Influence of each gain)')
        
        ax2.set_ylabel('PID/LQR Terms')
        ax2.legend(loc='upper right')
        ax2.grid(True)

        # Subplot 3: Loop Frequency and Latency
        ax3 = plt.subplot(nrows, 1, 3, sharex=ax1)
        if 'lp_hz' in df.columns:
            # Detect target frequency
            max_hz = df['lp_hz'].max()
            target_hz = 1000.0 if max_hz > 700 else 500.0
            
            # Use rolling mean for frequency to avoid the "block of red" effect
            freq_smooth = df['lp_hz'].rolling(window=50, min_periods=1, center=True).mean()
            ax3.plot(t_axis, freq_smooth, 'r-', label=f'Loop Freq (Hz, avg)', linewidth=1.5)
            
            # Fill region where frequency drops significantly (e.g. < 95% of target)
            warning_mask = df['lp_hz'] < (target_hz * 0.95)
            if warning_mask.any():
                # Plot faint red dots for raw drops, and a light background for sustained drops
                ax3.scatter(t_axis[warning_mask], df['lp_hz'][warning_mask], color='red', s=1, alpha=0.3, label='Freq Drop')
            
            ax3.axhline(y=target_hz, color='k', linestyle=':', alpha=0.5)
            ax3.set_ylim(0, target_hz * 1.2)

        ax3_twin = ax3.twinx()

        if 'bus_lat_l_us' in df.columns and 'bus_lat_l_age_ms' in df.columns:
            lat_l = df['bus_lat_l_us'].copy()
            lat_l[df['bus_lat_l_age_ms'] > 50] = np.nan
            ax3_twin.plot(t_axis, lat_l / 1000.0, 'b-', label='Bus Lat L (ms)', alpha=0.5)
        if 'bus_lat_r_us' in df.columns and 'bus_lat_r_age_ms' in df.columns:
            lat_r = df['bus_lat_r_us'].copy()
            lat_r[df['bus_lat_r_age_ms'] > 50] = np.nan
            ax3_twin.plot(t_axis, lat_r / 1000.0, 'b--', label='Bus Lat R (ms)', alpha=0.5)

        if 'last_encoder_age_ms' in df.columns:
            enc_age = df['last_encoder_age_ms'].copy()
            # Treat UINT32_MAX as "never updated"
            enc_age[enc_age >= 4_000_000_000] = np.nan
            ax3_twin.plot(t_axis, enc_age, color='orange', linestyle='-', label='Encoder Age (ms)', alpha=0.7)

        # Estimated transfer time for speed command + ACK (7 bytes TX + 5 bytes RX)
        transfer_us = (12.0 * 10.0 * 1e6) / float(args.mks_baud)

        if 'ack_pending_l_us' in df.columns:
            val = df['ack_pending_l_us'].rolling(window=20, min_periods=1).mean() / 1000.0
            ax3_twin.plot(t_axis, val, 'c-', label='ACK Pending L (ms, avg)', alpha=0.6)
            proc = (df['ack_pending_l_us'] - transfer_us).clip(lower=0.0)
            proc = proc.rolling(window=20, min_periods=1).mean() / 1000.0
            ax3_twin.plot(t_axis, proc, color='darkgreen', linestyle='-', label='MKS Proc L (ms, avg)', alpha=0.8)
        if 'ack_pending_r_us' in df.columns:
            val = df['ack_pending_r_us'].rolling(window=20, min_periods=1).mean() / 1000.0
            ax3_twin.plot(t_axis, val, 'c--', label='ACK Pending R (ms, avg)', alpha=0.6)
            proc = (df['ack_pending_r_us'] - transfer_us).clip(lower=0.0)
            proc = proc.rolling(window=20, min_periods=1).mean() / 1000.0
            ax3_twin.plot(t_axis, proc, color='darkgreen', linestyle='--', label='MKS Proc R (ms, avg)', alpha=0.8)

        if 'prof_f' in df.columns:
            val = df['prof_f'].rolling(window=20, min_periods=1).mean() / 1000.0
            ax3_twin.plot(t_axis, val, color='lime', linestyle='-', label='Fusion (ms, avg)', alpha=0.7)
        if 'prof_l' in df.columns:
            val = df['prof_l'].rolling(window=20, min_periods=1).mean() / 1000.0
            ax3_twin.plot(t_axis, val, 'm-', label='LQR (ms, avg)', alpha=0.7)
        if 'prof_t' in df.columns:
            val = df['prof_t'].rolling(window=20, min_periods=1).mean() / 1000.0
            ax3_twin.plot(t_axis, val, 'k:', label='Total Compute (ms, avg)', alpha=0.8)

        # Auto-scale latency axis: at least 1.5ms
        max_lat_ms = (df['lat'].max() / 1000.0) if ('lat' in df.columns and not df['lat'].empty) else 1.0
        if 'bus_lat_l_us' in df.columns and not df['bus_lat_l_us'].empty:
            max_lat_ms = max(max_lat_ms, (df['bus_lat_l_us'].max() / 1000.0))
        if 'bus_lat_r_us' in df.columns and not df['bus_lat_r_us'].empty:
            max_lat_ms = max(max_lat_ms, (df['bus_lat_r_us'].max() / 1000.0))
        if 'last_encoder_age_ms' in df.columns and not df['last_encoder_age_ms'].empty:
            enc_age_max = df['last_encoder_age_ms'].replace([np.inf, -np.inf], np.nan).dropna().max()
            if pd.notna(enc_age_max) and enc_age_max < 4_000_000_000:
                max_lat_ms = max(max_lat_ms, enc_age_max)
        if 'ack_pending_l_us' in df.columns and not df['ack_pending_l_us'].empty:
            max_lat_ms = max(max_lat_ms, (df['ack_pending_l_us'].max() / 1000.0))
        if 'ack_pending_r_us' in df.columns and not df['ack_pending_r_us'].empty:
            max_lat_ms = max(max_lat_ms, (df['ack_pending_r_us'].max() / 1000.0))
        if 'prof_t' in df.columns and not df['prof_t'].empty:
            max_lat_ms = max(max_lat_ms, (df['prof_t'].max() / 1000.0))
        ax3_twin.set_ylim(0.0, max(1.5, max_lat_ms * 1.1))
        
        ax3.set_ylabel('Frequency (Hz)', color='r')
        ax3_twin.set_ylabel('Latency/Compute (ms)', color='b')
        ax3_twin.legend(loc='upper right', fontsize=8)
        ax3.set_title(f'Timing Stability & Profiling (Target: {int(target_hz)}Hz)')
        ax3.grid(True)

        # Subplot 4: Total PID and Motor Command
        ax4 = plt.subplot(nrows, 1, 4, sharex=ax1)
        if 'pid_out' in df.columns:
            ax4.plot(t_axis, df['pid_out'], 'g-', label='Total PID/LQR Out', linewidth=1.5)
        if 'cmd' in df.columns:
            ax4.plot(t_axis, df['cmd'], 'k--', label='Throttle Cmd', alpha=0.8)
        if 'steer' in df.columns:
            ax4.plot(t_axis, df['steer'], 'r-', label='Steer Cmd', alpha=0.9)
        ax4.set_ylabel('Command')
        ax4.legend(loc='upper right')
        ax4.grid(True)

        # Subplot 5: Navigation Analysis (Linear Distance & Rotation)
        ax5 = plt.subplot(nrows, 1, 5, sharex=ax1)
        if 'encL' in df.columns and 'encR' in df.columns:
            linear_pos = (df['encL'] + df['encR']) / 2.0
            rotation = (df['encL'] - df['encR']) / 2.0  # Proxy for heading
            # Center them to start at 0 for easier reading
            linear_pos -= linear_pos.iloc[0]
            rotation -= rotation.iloc[0]
            
            ax5.plot(t_axis, linear_pos, 'k-', label='Linear Position (Avg Ticks)', linewidth=1.5)
            ax5.set_ylabel('Linear (Ticks)', color='k')
            
            # Add secondary scale for meters
            # Ticks to meters scale: (math.pi * 0.067) / 51200.0 (67mm wheel)
            t2m = (math.pi * 0.067) / 51200.0
            ax5_m = ax5.twinx()
            ax5_m.set_ylabel('Linear (Meters)', color='gray')
            # Set the limits of the meter axis proportional to the ticks axis
            y1_min, y1_max = ax5.get_ylim()
            ax5_m.set_ylim(y1_min * t2m, y1_max * t2m)
            ax5_m.tick_params(axis='y', labelcolor='gray')

            ax5_rot = ax5.twinx()
            ax5_rot.spines['right'].set_position(('outward', 60))
            ax5_rot.plot(t_axis, rotation, 'c--', label='Rotation/Yaw (Delta Ticks)', alpha=0.6)
            ax5_rot.set_ylabel('Rotation (Ticks)', color='c')
            ax5_rot.tick_params(axis='y', labelcolor='c')
            
            # Combine legends
            lines, labels = ax5.get_legend_handles_labels()
            lines2, labels2 = ax5_rot.get_legend_handles_labels()
            ax5.legend(lines + lines2, labels + labels2, loc='upper left')
            
        ax5.set_title('Navigation Analysis (Forward/Backward & Yaw Orientation)')
        ax5.grid(True)

        # Subplot 6: Yaw Stability and Heading Hold
        ax6 = plt.subplot(nrows, 1, 6, sharex=ax1)
        if 'gz' in df.columns:
            ax6.plot(t_axis, np.degrees(df['gz']), 'r-', label='Yaw Rate (deg/s)', alpha=0.8)
        if 'steer' in df.columns:
            ax6_twin = ax6.twinx()
            ax6_twin.plot(t_axis, df['steer'], 'b-', label='Steer Output (Heading Hold)', alpha=0.5)
            ax6_twin.set_ylabel('Steer Output', color='b')
            ax6_twin.set_ylim(-1.0, 1.0)
            
            # Combine legends
            lines, labels = ax6.get_legend_handles_labels()
            lines2, labels2 = ax6_twin.get_legend_handles_labels()
            ax6.legend(lines + lines2, labels + labels2, loc='upper left')
        else:
            ax6.legend(loc='upper left')
            
        ax6.set_ylabel('Rate (deg/s)', color='r')
        ax6.set_title('Yaw Stability and Heading Hold')
        ax6.grid(True)

        # Subplot 7: IMU Raw Pitch-Axis
        ax7 = plt.subplot(nrows, 1, 7, sharex=ax1)
        ax7.plot(t_axis, df['gx'], label='Gyro X (Roll)', alpha=0.3)
        ax7.plot(t_axis, df['gy'], label='Gyro Y (Pitch)', alpha=0.8)
        ax7.plot(t_axis, df['ax'], label='Accel X (Linear)', alpha=0.6)
        ax7.set_ylabel('IMU Raw')
        ax7.legend(loc='upper right')
        ax7.grid(True)

        # Subplot 8: FFT Analysis (Pitch)
        # Goal: highlight resonances (e.g. 8-20Hz) and ignore the low-frequency fall/drift.
        ax8 = plt.subplot(nrows, 1, 8)
        pitch_df = df[['time_ms', 'pitch']].dropna()
        if len(pitch_df) > 32:  # Lowered requirement (from 64)
            dt_series = pitch_df['time_ms'].diff().dropna()
            if not dt_series.empty:
                dt_avg_s = dt_series.mean() / 1000.0
                if dt_avg_s > 0:
                    fs = 1.0 / dt_avg_s
                    t = (pitch_df['time_ms'].values - pitch_df['time_ms'].values[0]) / 1000.0
                    y = pitch_df['pitch'].values

                    # Detrend (linear) to suppress slow drift / fall dominating the FFT
                    if len(y) >= 3:
                        a, b = np.polyfit(t, y, 1)
                        y_detr = y - (a * t + b)
                    else:
                        y_detr = y - np.mean(y)

                    n = len(y_detr)
                    window = np.hanning(n)
                    yf = np.fft.rfft(y_detr * window)
                    xf = np.fft.rfftfreq(n, d=1.0 / fs)
                    mag = np.abs(yf)

                    ax8.plot(xf, mag, 'r-', linewidth=1.5)

                    # Focus resonance search band (ignore <5Hz because it captures the fall/lean)
                    f_lo = 5.0
                    f_hi = 30.0
                    band = (xf >= f_lo) & (xf <= f_hi)
                    peak_freq = None
                    peak_mag = None

                    if np.any(band):
                        band_x = xf[band]
                        band_mag = mag[band]

                        # Top-3 peaks in band
                        top_n = min(3, len(band_mag))
                        top_idx = np.argsort(band_mag)[-top_n:][::-1]
                        peaks = [(float(band_x[i]), float(band_mag[i])) for i in top_idx]

                        if peaks:
                            peak_freq, peak_mag = peaks[0]
                            for f, _m in peaks:
                                ax8.axvline(f, color='k', linestyle=':', alpha=0.2)

                            df_hz = fs / n if n > 0 else 0.0
                            peaks_txt = "\n".join([f"{f:5.2f} Hz" for f, _m in peaks])
                            info = (
                                f"Peak (5-30Hz): {peak_freq:.2f} Hz\n"
                                f"Δf≈{df_hz:.2f} Hz, fs≈{fs:.1f} Hz\n"
                                f"Top peaks:\n{peaks_txt}"
                            )
                            ax8.text(
                                0.02,
                                0.95,
                                info,
                                transform=ax8.transAxes,
                                verticalalignment='top',
                                bbox=dict(boxstyle='round', facecolor='white', alpha=0.8),
                                fontsize=9,
                            )

                    title = "Pitch FFT Analysis"
                    if peak_freq is not None:
                        title += f" (Peak 5-30Hz: {peak_freq:.2f} Hz)"
                    ax8.set_title(title)
                    ax8.set_xlim(0, min(40.0, fs / 2.0))
                    if len(mag) > 0:
                        ax8.set_ylim(0, float(mag.max()) * 1.1)
        else:
            ax8.set_title(f"Pitch FFT Analysis (Not enough data: {len(pitch_df)} samples)")
            
        ax8.set_ylabel('Magnitude')
        ax8.set_xlabel('Frequency (Hz)')
        ax8.grid(True)

        next_row = 9
        if has_cpu:
            ax_cpu = plt.subplot(nrows, 1, next_row, sharex=ax1)
            # Use rolling means and fill_between for clarity
            if 'cpu_core0_pct' in df.columns:
                c0 = df['cpu_core0_pct'].rolling(window=100, min_periods=1, center=True).mean()
                ax_cpu.fill_between(t_axis, 0, c0, color='blue', alpha=0.2, label='Core 0 (System) Range')
                ax_cpu.plot(t_axis, c0, 'b-', alpha=0.9, label='Core 0 (System) Avg', linewidth=1)
                
            if 'cpu_core1_pct' in df.columns:
                c1 = df['cpu_core1_pct'].rolling(window=100, min_periods=1, center=True).mean()
                ax_cpu.fill_between(t_axis, 0, c1, color='red', alpha=0.2, label='Core 1 (Control) Range')
                ax_cpu.plot(t_axis, c1, 'r-', alpha=0.9, label='Core 1 (Control) Avg', linewidth=1)

            ax_cpu.set_ylabel('CPU Load (%)')
            ax_cpu.set_ylim(0.0, 100.0)
            ax_cpu.set_title('CPU Load (per core)')
            ax_cpu.legend(loc='upper right')
            ax_cpu.grid(True)
            next_row += 1

        # Optional: Motor command freshness / bus health
        if has_motor:
            ax9 = plt.subplot(nrows, 1, next_row, sharex=ax1)
            if 'motor_bus_lat_us' in df.columns:
                ax9.plot(t_axis, df['motor_bus_lat_us'] / 1000.0, 'k-', alpha=0.4, label='motor bus lat (ms)')
            if 'motor_l_cmd_age_ms' in df.columns:
                ax9.plot(t_axis, df['motor_l_cmd_age_ms'], 'b-', alpha=0.8, label='L cmd age (ms)')
            if 'motor_r_cmd_age_ms' in df.columns:
                ax9.plot(t_axis, df['motor_r_cmd_age_ms'], 'r-', alpha=0.8, label='R cmd age (ms)')
            ax9.set_ylabel('Age(ms)/Lat(ms)')
            ax9.set_xlabel('Time (s)')
            accel_title = ''
            if 'motor_accel' in df.columns:
                accel_vals = df['motor_accel'].dropna()
                accel_vals = accel_vals[accel_vals != 0]
                uniq = list(pd.unique(accel_vals))
                if len(uniq) == 1:
                    accel_title = f" (accel={int(uniq[0])})"
                elif len(uniq) > 1:
                    accel_title = f" (accel changes: {','.join(str(int(x)) for x in uniq[:5])})"

            ax9.set_title('Motor Health: command age and bus latency' + accel_title)
            ax9.legend(loc='upper right')
            ax9.grid(True)

        plt.tight_layout()
        try:
            ax1.set_xlim(left=0.0)
        except Exception:
            pass
        plt.savefig(args.png)
        print(f"Saved plot: {args.png}")
        
    except Exception as e:
        print(f"Plotting failed: {e}")

if __name__ == '__main__':
    main()

