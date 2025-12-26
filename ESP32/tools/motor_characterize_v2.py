#!/usr/bin/env python3
"""
Motor Characterization Script v2 - Telemetry Stream Based
Characterizes motor latency, tau, deadzone, and saturation using telemetry stream.
"""

import socket
import time
import re
import os
import math
import numpy as np
import matplotlib.pyplot as plt
from dataclasses import dataclass
from typing import List, Optional, Tuple


@dataclass
class TelemetrySample:
    """Single telemetry sample"""
    timestamp_us: int          # Firmware timestamp (microseconds)
    encoder: int               # Encoder count
    speed: float               # Speed (counts/sec)
    cmd_timestamp_us: int      # Command timestamp (microseconds)
    host_time: float           # Host reception time


@dataclass
class StepTestResult:
    """Results from a step response test"""
    motor: str
    direction: str              # 'forward' or 'backward'
    command: float
    samples: List[TelemetrySample]
    latency_ms: Optional[float]
    tau_s: Optional[float]
    steady_speed: Optional[float]
    command_entry_us: Optional[int] = None  # Firmware command entry timestamp


@dataclass
class SweepTestResult:
    """Results from deadzone/saturation sweep test"""
    motor: str
    commands: List[float]
    speeds: List[float]
    deadzone_range: Tuple[float, float]
    linear_range: Tuple[float, float]
    saturation_start: float
    linear_fit_slope: Optional[float] = None      # a in y=ax+b (counts/s per command unit)
    linear_fit_intercept: Optional[float] = None  # b in y=ax+b (counts/s)


class MotorCharacterizer:
    """Motor characterization using telemetry stream"""
    
    def __init__(self, host: str, port: int = 2333, output_dir: str = "artifacts/motor_tests", 
                 telemetry_interval_ms: int = 20):
        self.host = host
        self.port = port
        self.output_dir = output_dir
        self.telemetry_interval_ms = telemetry_interval_ms
        self.sock = None
        self.console_log = []
        
        os.makedirs(output_dir, exist_ok=True)
        
    def connect(self):
        """Connect to ESP32 WiFi console"""
        print(f"Connecting to {self.host}:{self.port}...")
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.settimeout(5.0)
        self.sock.connect((self.host, self.port))
        time.sleep(0.5)
        # Drain any pending data
        self.sock.settimeout(0.1)
        try:
            self.sock.recv(4096)
        except socket.timeout:
            pass
        self.sock.settimeout(5.0)
        print("Connected!")
        
    def send_command(self, cmd: str):
        """Send command to console"""
        self.sock.sendall((cmd + '\n').encode('utf-8'))
        time.sleep(0.05)
        
    def recv_lines(self, timeout: float = 0.5) -> List[str]:
        """Receive lines from console"""
        self.sock.settimeout(timeout)
        lines = []
        try:
            data = self.sock.recv(8192).decode('utf-8', errors='ignore')
            for line in data.split('\n'):
                line = line.strip()
                if line:
                    self.console_log.append(line)
                    lines.append(line)
        except socket.timeout:
            pass
        return lines
        
    def parse_telemetry(self, line: str) -> Optional[TelemetrySample]:
        """Parse a telemetry line"""
        if 'MOTOR: telemetry' not in line:
            return None
            
        try:
            # Extract fields
            ts_match = re.search(r'ts_us=(\d+)', line)
            enc_match = re.search(r'enc=(-?\d+)', line)
            sp_match = re.search(r'sp=([+-]?\d+\.?\d*)', line)
            cmd_ts_match = re.search(r'cmd_ts_us=(\d+)', line)
            id_match = re.search(r'id=(\d+)', line)
            
            if not all([ts_match, enc_match, sp_match, cmd_ts_match, id_match]):
                return None
                
            return TelemetrySample(
                timestamp_us=int(ts_match.group(1)),
                encoder=int(enc_match.group(1)),
                speed=float(sp_match.group(1)),
                cmd_timestamp_us=int(cmd_ts_match.group(1)),
                host_time=time.time()
            )
        except Exception as e:
            return None
            
    def enable_telemetry(self, motor: str = None):
        """Enable motor telemetry stream.

        If `motor` is provided ('LEFT' or 'RIGHT') start telemetry only for that motor,
        otherwise start for both sides.
        """
        print(f"Enabling telemetry stream ({self.telemetry_interval_ms}ms interval)...")
        # Request WiFi console to forward motor channel
        self.send_command("LOG ENABLE MOTOR")
        time.sleep(0.3)
        
        # Wait for confirmation
        confirmed = False
        for _ in range(5):
            lines = self.recv_lines(0.2)
            for line in lines:
                if 'LOG: enabled' in line or 'LOG enabled' in line:
                    print("Console confirmed motor logging enabled.")
                    confirmed = True
                    break
            if confirmed:
                break
        
        if not confirmed:
            print("Warning: did not see console confirmation for LOG ENABLE MOTOR")
        
        # Enable motors
        self.send_command("MOTOR ENABLE")
        time.sleep(0.5)
        
        # Wait for motor init messages
        for _ in range(10):
            lines = self.recv_lines(0.2)
            for line in lines:
                if 'motor_driver:' in line and ('init' in line or 'ENABLED' in line):
                    print(f"Motor init confirmed: {line[:60]}...")
                    break

        # Start telemetry for requested motor(s)
        if motor is None:
            # both sides
            self.send_command(f"MOTOR TELEMETRY LEFT {self.telemetry_interval_ms}")
            time.sleep(0.2)
            self.send_command(f"MOTOR TELEMETRY RIGHT {self.telemetry_interval_ms}")
        else:
            m = motor.upper()
            if m in ('LEFT', 'RIGHT'):
                self.send_command(f"MOTOR TELEMETRY {m} {self.telemetry_interval_ms}")
            else:
                # fallback to both
                self.send_command(f"MOTOR TELEMETRY LEFT {self.telemetry_interval_ms}")
                time.sleep(0.2)
                self.send_command(f"MOTOR TELEMETRY RIGHT {self.telemetry_interval_ms}")

        time.sleep(1.0)  # Wait for telemetry to start
        # Verify telemetry is working
        for _ in range(20):
            lines = self.recv_lines(0.2)
            for line in lines:
                if 'MOTOR: telemetry' in line:
                    print("Telemetry stream confirmed!")
                    return
        print("Warning: Telemetry stream not confirmed")
        
    def disable_telemetry(self, motor: str = None):
        """Disable telemetry and motors"""
        if motor:
            self.send_command(f"MOTOR TELEMETRY {motor} 0")
            time.sleep(0.2)
        else:
            print("Disabling motors...")
            self.send_command("MOTOR SET LEFT 0")
            self.send_command("MOTOR SET RIGHT 0")
            time.sleep(0.2)
            self.send_command("MOTOR TELEMETRY LEFT 0")
            self.send_command("MOTOR TELEMETRY RIGHT 0")
        self.send_command("MOTOR TELEMETRY RIGHT 0")
        time.sleep(0.2)
        self.send_command("MOTOR DISABLE")
        
    def wait_for_zero_speed(self, motor_id: int, timeout: float = 5.0) -> bool:
        """Wait for motor speed to stabilize at zero"""
        print(f"  Waiting for motor id={motor_id} to stabilize at zero...")
        start = time.time()
        stable_count = 0
        required_stable = 3
        
        while time.time() - start < timeout:
            lines = self.recv_lines(0.05)
            for line in lines:
                sample = self.parse_telemetry(line)
                if sample and self._extract_motor_id(line) == motor_id:
                    if abs(sample.speed) < 100:  # counts/sec threshold
                        stable_count += 1
                        if stable_count >= required_stable:
                            print(f"  Motor stabilized (speed: {sample.speed:.1f} counts/s)")
                            return True
                    else:
                        stable_count = 0
            time.sleep(0.02)
            
        print(f"  Warning: Motor did not stabilize within {timeout}s")
        return False
        
    def _extract_motor_id(self, line: str) -> Optional[int]:
        """Extract motor ID from telemetry line"""
        match = re.search(r'id=(\d+)', line)
        return int(match.group(1)) if match else None
        
    def run_step_test(self, motor: str, command: float, duration: float = 3.0) -> StepTestResult:
        """Run a step response test"""
        motor_id = 0 if motor.upper() == 'LEFT' else 1
        direction = 'forward' if command > 0 else 'backward'
        
        print(f"\n{'='*60}")
        print(f"Step test: {motor} motor, command={command:.2f} ({direction})")
        print(f"{'='*60}")
        
        # Enable telemetry for this specific motor
        self.enable_telemetry(motor)
        
        # Ensure motor starts at zero
        self.send_command(f"MOTOR SET {motor} 0")
        self.wait_for_zero_speed(motor_id, timeout=5.0)
        
        # Collect baseline sample
        baseline_sample = None
        for _ in range(20):
            lines = self.recv_lines(0.05)
            for line in lines:
                sample = self.parse_telemetry(line)
                if sample and self._extract_motor_id(line) == motor_id:
                    baseline_sample = sample
                    break
            if baseline_sample:
                break
                
        print(f"  Baseline: enc={baseline_sample.encoder if baseline_sample else 'N/A'}")
        
        # Send step command
        cmd_str = f"MOTOR SET {motor} {command:.2f}"
        print(f"  Sending: {cmd_str}")
        cmd_time = time.time()
        log_pos_before_cmd = len(self.console_log)  # Mark position before sending
        self.send_command(cmd_str)
        
        # Wait briefly and capture firmware logs (command entry, etc.)
        time.sleep(0.1)
        self.recv_lines(0.1)  # Drain firmware response logs
        
        # Collect samples during step
        samples = []
        start_time = time.time()
        
        while time.time() - start_time < duration:
            lines = self.recv_lines(0.02)
            for line in lines:
                sample = self.parse_telemetry(line)
                if sample and self._extract_motor_id(line) == motor_id:
                    samples.append(sample)
            time.sleep(0.01)
            
        # Stop motor
        self.send_command(f"MOTOR SET {motor} 0")
        
        # Disable telemetry for this motor
        self.disable_telemetry(motor)
        
        print(f"  Collected {len(samples)} samples")
        
        # Analyze results
        latency_ms, command_entry_us = self._calculate_latency(samples, baseline_sample, cmd_time, log_pos_before_cmd, motor)
        tau_s = self._calculate_tau(samples, baseline_sample, cmd_time, log_pos_before_cmd, motor)
        steady_speed = self._calculate_steady_speed(samples)
        
        if latency_ms:
            print(f"  Latency: {latency_ms:.1f} ms")
        if tau_s:
            print(f"  Tau: {tau_s:.3f} s")
        if steady_speed:
            print(f"  Steady speed: {steady_speed:.1f} counts/s")
            
        return StepTestResult(
            motor=motor,
            direction=direction,
            command=command,
            samples=samples,
            latency_ms=latency_ms,
            tau_s=tau_s,
            steady_speed=steady_speed,
            command_entry_us=command_entry_us
        )
        
    def _calculate_latency(self, samples: List[TelemetrySample], 
                          baseline: Optional[TelemetrySample],
                          cmd_time: float,
                          log_pos: int,
                          motor: str) -> Tuple[Optional[float], Optional[int]]:
        """Calculate latency from first movement detection using command entry timestamp
        Returns: (latency_ms, command_entry_us)
        """
        if not samples or not baseline:
            return None, None
            
        # Find command entry timestamp for the SPECIFIC motor in NEW console logs
        # IMPORTANT: Skip raw=0 (stop commands), only use raw≠0 (movement commands)
        command_entry_us = None
        new_lines = self.console_log[log_pos:]
        motor_pattern = f'command entry {motor.upper()}'
        for line in new_lines:
            if motor_pattern in line and 'us' in line and 'raw=0)' not in line:
                match = re.search(r'at (\d+) us', line)
                if match:
                    command_entry_us = int(match.group(1))
                    break
        
        if not command_entry_us:
            return None, None
        
        # Find first sample with significant movement AFTER command entry
        for sample in samples:
            if sample.timestamp_us <= command_entry_us:
                continue  # Skip samples before/at command
            
            # Detect first significant speed (any movement above 10 counts/s)
            if abs(sample.speed) >= 10:
                latency_us = sample.timestamp_us - command_entry_us
                return latency_us / 1000.0, command_entry_us  # Convert to ms
                
        return None, command_entry_us
        
    def _calculate_tau(self, samples: List[TelemetrySample],
                       baseline: Optional[TelemetrySample],
                       cmd_time: Optional[float] = None,
                       log_pos: Optional[int] = None,
                       motor: Optional[str] = None) -> Optional[float]:
        """Calculate time constant (tau) - time to reach 63% of final value"""
        if len(samples) < 10 or baseline is None:
            return None

        # Compute steady-state speed (use absolute value)
        steady = self._calculate_steady_speed(samples)
        if steady is None or abs(steady) < 1.0:
            return None

        v_63 = 0.63 * abs(steady)

        # Determine command entry time from firmware logs
        command_entry_us = None
        if log_pos is not None and motor is not None:
            new_lines = self.console_log[log_pos:]
            motor_pattern = f'command entry {motor.upper()}'
            for line in new_lines:
                if motor_pattern in line and 'us' in line:
                    if 'raw=0)' not in line:
                        match = re.search(r'at (\d+) us', line)
                        if match:
                            command_entry_us = int(match.group(1))
                            break

        # Find ACTUAL start of movement (first sample with speed > 5% of steady)
        # This handles deadzone delay where motor doesn't move immediately
        movement_start_us = None
        movement_threshold = 0.05 * abs(steady)  # 5% of final speed
        
        for sample in samples:
            # Skip samples before command entry if we have it
            if command_entry_us and sample.timestamp_us < command_entry_us:
                continue
            
            if abs(sample.speed) >= movement_threshold:
                movement_start_us = sample.timestamp_us
                break
        
        if movement_start_us is None:
            # No movement detected, use command entry or first sample as fallback
            movement_start_us = command_entry_us if command_entry_us else samples[0].timestamp_us

        # Now find tau from ACTUAL movement start (not command entry)
        for sample in samples:
            if sample.timestamp_us <= movement_start_us:
                continue
                
            if abs(sample.speed) >= v_63:
                tau_us = sample.timestamp_us - movement_start_us
                if tau_us <= 0:
                    return None
                return tau_us / 1e6  # seconds

        return None
        
    def _calculate_steady_speed(self, samples: List[TelemetrySample]) -> Optional[float]:
        """Calculate steady-state speed"""
        if len(samples) < 10:
            return None
            
        # Average last 30% of samples
        n_final = max(5, int(len(samples) * 0.3))
        final_speeds = [s.speed for s in samples[-n_final:]]
        return np.mean(final_speeds)
        
    def run_sweep_test(self, motor: str, commands: List[float]) -> SweepTestResult:
        """Run deadzone/saturation sweep test"""
        motor_id = 0 if motor.upper() == 'LEFT' else 1
        
        print(f"\n{'='*60}")
        print(f"Sweep test: {motor} motor, {len(commands)} steps")
        print(f"{'='*60}")
        
        # Ensure telemetry is enabled for this motor during the sweep
        self.enable_telemetry(motor)

        measured_speeds = []
        
        for i, cmd in enumerate(commands):
            print(f"  Step {i+1}/{len(commands)}: command={cmd:.2f}")
            
            # Set command
            self.send_command(f"MOTOR SET {motor} {cmd:.2f}")
            
            # Wait for settling - longer for first point to stabilize from zero
            if i == 0:
                time.sleep(1.5)  # First point needs more time to reach steady state
            else:
                time.sleep(0.5)
            
            # Collect samples
            samples = []
            for _ in range(15):  # Collect ~300ms of data
                lines = self.recv_lines(0.02)
                for line in lines:
                    sample = self.parse_telemetry(line)
                    if sample and self._extract_motor_id(line) == motor_id:
                        samples.append(sample)
                time.sleep(0.02)
                
            # Average speed from samples
            if samples:
                avg_speed = np.mean([s.speed for s in samples])
                measured_speeds.append(avg_speed)
                print(f"    Speed: {avg_speed:.1f} counts/s")
            else:
                measured_speeds.append(0.0)
                print(f"    Speed: N/A (no samples)")
                
        # Stop motor
        self.send_command(f"MOTOR SET {motor} 0")

        # Disable telemetry for this motor after sweep
        self.disable_telemetry(motor)

        # Analyze deadzone and saturation
        deadzone, linear, saturation, slope, intercept = self._analyze_sweep(commands, measured_speeds)
        
        print(f"\n  Deadzone range: [{deadzone[0]:.2f}, {deadzone[1]:.2f}]")
        print(f"  Linear range: [{linear[0]:.2f}, {linear[1]:.2f}]")
        print(f"  Saturation starts: {saturation:.2f}")
        if slope is not None and intercept is not None:
            print(f"  Linear fit: speed = {slope:.1f} * cmd + {intercept:.1f}")
        
        return SweepTestResult(
            motor=motor,
            commands=commands,
            speeds=measured_speeds,
            deadzone_range=deadzone,
            linear_range=linear,
            saturation_start=saturation,
            linear_fit_slope=slope,
            linear_fit_intercept=intercept
        )
        
    def _analyze_sweep(self, commands: List[float], speeds: List[float]) -> Tuple[Tuple[float, float], Tuple[float, float], float, Optional[float], Optional[float]]:
        """Analyze sweep to find deadzone, linear range, saturation, and linear fit coefficients"""
        abs_speeds = [abs(s) for s in speeds]
        
        # Find saturation (where speed stops increasing significantly) - search from high speed end
        max_speed = max(abs_speeds)
        saturation_thresh = 0.95 * max_speed
        
        # Find deadzone (where speed is negligible) - use percentage of max speed for robustness
        # Deadzone is where motor doesn't respond linearly yet (typically < 5-10% of max speed)
        deadzone_thresh = 0.05 * max_speed  # 5% of max speed
        
        if commands[0] >= 0 and commands[-1] < 0:
            # Negative commands: 0.0 → -1.0
            # Deadzone at start (zero speed, 0.0), saturation at end (high speed, -1.0)
            deadzone_end_idx = 0
            for i, spd in enumerate(abs_speeds):
                if spd <= deadzone_thresh:
                    deadzone_end_idx = i  # Last point still in deadzone
                else:
                    break  # Stop at first point clearly out of deadzone
            deadzone_end_idx = max(0, deadzone_end_idx)
            
            saturation_idx = len(abs_speeds) - 1
            for i in range(len(abs_speeds) - 1, -1, -1):
                if abs_speeds[i] >= saturation_thresh:
                    saturation_idx = i
                    break
            
            # Linear range is between deadzone and saturation
            linear_start_idx = min(deadzone_end_idx + 1, len(commands) - 1)
            linear_end_idx = max(saturation_idx - 1, 0)
            
            saturation_start = commands[saturation_idx]
            deadzone_range = (0.0, commands[deadzone_end_idx])
            linear_range = (commands[linear_start_idx], commands[linear_end_idx])
        else:
            # Positive commands: 0.0 → 1.0
            # Deadzone at start (zero speed, 0.0), saturation at end (high speed, 1.0)
            deadzone_end_idx = 0
            for i, spd in enumerate(abs_speeds):
                if spd > deadzone_thresh:
                    deadzone_end_idx = i
                    break
            
            saturation_idx = len(abs_speeds) - 1
            for i in range(len(abs_speeds) - 1, 0, -1):
                if abs_speeds[i] >= saturation_thresh:
                    saturation_idx = i
                    break
            
            # Linear range is between deadzone and saturation
            linear_start_idx = min(deadzone_end_idx + 1, len(commands) - 1)
            linear_end_idx = max(saturation_idx - 1, 0)
            
            deadzone_range = (0.0, commands[deadzone_end_idx])
            linear_range = (commands[linear_start_idx], commands[linear_end_idx])
            saturation_start = commands[saturation_idx]
        
        # Calculate linear regression on linear range
        slope, intercept = None, None
        if linear_start_idx < linear_end_idx:
            # Extract points in linear range
            linear_cmds = commands[linear_start_idx:linear_end_idx+1]
            linear_speeds = speeds[linear_start_idx:linear_end_idx+1]
            
            if len(linear_cmds) >= 2:
                # Fit line: speed = slope * cmd + intercept
                coeffs = np.polyfit(linear_cmds, linear_speeds, 1)
                slope = coeffs[0]
                intercept = coeffs[1]
            
        return deadzone_range, linear_range, saturation_start, slope, intercept
        
    def save_results(self, step_results: List[StepTestResult], sweep_results: List[SweepTestResult]):
        """Save all results to files and generate plots"""
        print(f"\n{'='*60}")
        print("Saving results...")
        print(f"{'='*60}")
        
        # Save summary
        summary_path = os.path.join(self.output_dir, "characterization_summary.txt")
        with open(summary_path, 'w') as f:
            f.write("MOTOR CHARACTERIZATION SUMMARY\n")
            f.write("="*60 + "\n\n")
            
            # Step test results
            f.write("LATENCY AND TAU (Step Response Tests)\n")
            f.write("-"*60 + "\n")
            for result in step_results:
                f.write(f"\n{result.motor} {result.direction} (cmd={result.command:.2f}):\n")
                if result.latency_ms:
                    f.write(f"  Latency: {result.latency_ms:.1f} ms\n")
                if result.tau_s:
                    f.write(f"  Tau: {result.tau_s:.3f} s\n")
                if result.steady_speed:
                    f.write(f"  Steady speed: {result.steady_speed:.1f} counts/s\n")
                    
            # Calculate average tau
            taus = [r.tau_s for r in step_results if r.tau_s is not None]
            if taus:
                avg_tau = np.mean(taus)
                f.write(f"\nAverage Tau (all motors/directions): {avg_tau:.3f} s\n")
                f.write(f"Recommended Tau for control: {avg_tau:.3f} s\n")
                
            # Calculate average latency
            latencies = [r.latency_ms for r in step_results if r.latency_ms is not None]
            if latencies:
                avg_latency = np.mean(latencies)
                f.write(f"\nAverage Latency: {avg_latency:.1f} ms\n")
                f.write(f"Recommended Latency for control: {avg_latency/1000:.4f} s\n")
                
            # Sweep test results
            f.write("\n\nDEADZONE AND SATURATION (Sweep Tests)\n")
            f.write("-"*60 + "\n")
            for result in sweep_results:
                f.write(f"\n{result.motor}:")
                direction = 'Backward' if result.commands[-1] < 0 else 'Forward'
                f.write(f" ({direction})\n")
                f.write(f"  Deadzone range: [{result.deadzone_range[0]:.2f}, {result.deadzone_range[1]:.2f}]\n")
                f.write(f"  Linear range: [{result.linear_range[0]:.2f}, {result.linear_range[1]:.2f}]\n")
                f.write(f"  Saturation starts: {result.saturation_start:.2f}\n")
                if result.linear_fit_slope is not None and result.linear_fit_intercept is not None:
                    f.write(f"  Linear fit: speed = {result.linear_fit_slope:.1f} * cmd + {result.linear_fit_intercept:.1f}\n")
                
        print(f"Summary saved: {summary_path}")
        
        # Generate plots
        self._plot_step_results(step_results)
        self._plot_sweep_results(sweep_results)
        
        # Save console log
        log_path = os.path.join(self.output_dir, "console_log.txt")
        with open(log_path, 'w') as f:
            f.write('\n'.join(self.console_log))
        print(f"Console log saved: {log_path}")
        
    def _plot_step_results(self, results: List[StepTestResult]):
        """Plot step response results"""
        fig, axes = plt.subplots(2, 2, figsize=(14, 10))
        fig.suptitle('Motor Step Response Tests', fontsize=16)
        
        for i, result in enumerate(results):
            if i >= 4:
                break
            ax = axes[i // 2, i % 2]
            
            if result.samples:
                # Use command entry as t0 if available, otherwise use first sample
                if result.command_entry_us:
                    t0 = result.command_entry_us
                else:
                    t0 = result.samples[0].timestamp_us
                    
                times = [(s.timestamp_us - t0) / 1e6 for s in result.samples]  # seconds
                speeds = [abs(s.speed) for s in result.samples]
                
                ax.plot(times, speeds, 'b-', linewidth=2, label='Measured')
                
                # Mark 63% point if tau available
                if result.tau_s and result.steady_speed:
                    ax.axhline(y=abs(result.steady_speed) * 0.63, color='r', linestyle='--', 
                              label=f'Tau threshold (63%)')
                    ax.axvline(x=result.tau_s, color='r', linestyle='--', 
                              label=f'Tau = {result.tau_s*1000:.1f}ms')
                    
                ax.set_xlabel('Time (s)')
                ax.set_ylabel('Speed (counts/s)')
                ax.set_title(f'{result.motor} {result.direction}\n(cmd={result.command:.2f})')
                ax.grid(True, alpha=0.3)
                ax.legend()
                
        plt.tight_layout()
        plot_path = os.path.join(self.output_dir, "step_responses.png")
        plt.savefig(plot_path, dpi=150)
        plt.close()
        print(f"Step response plot saved: {plot_path}")
        
    def _plot_sweep_results(self, results: List[SweepTestResult]):
        """Plot sweep test results"""
        fig, axes = plt.subplots(2, 2, figsize=(14, 10))
        fig.suptitle('Motor Deadzone and Saturation', fontsize=16)
        
        for i, result in enumerate(results):
            if i >= 4:
                break
            ax = axes[i // 2, i % 2]
            
            ax.plot(result.commands, [abs(s) for s in result.speeds], 'bo-', linewidth=2, markersize=4)
            
            # Plot linear fit if available
            if result.linear_fit_slope is not None and result.linear_fit_intercept is not None:
                lr = result.linear_range
                fit_cmds = np.linspace(lr[0], lr[1], 50)
                fit_speeds = result.linear_fit_slope * fit_cmds + result.linear_fit_intercept
                ax.plot(fit_cmds, np.abs(fit_speeds), 'g--', linewidth=2, label=f'Linear fit: y={result.linear_fit_slope:.0f}x+{result.linear_fit_intercept:.0f}')
            
            # Mark deadzone
            dz = result.deadzone_range
            ax.axvspan(dz[0], dz[1], alpha=0.2, color='red', label='Deadzone')
            
            # Mark linear range
            lr = result.linear_range
            ax.axvspan(lr[0], lr[1], alpha=0.2, color='green', label='Linear')
            
            # Mark saturation
            sat = result.saturation_start
            if result.commands[-1] < 0:
                ax.axvspan(result.commands[-1], sat, alpha=0.2, color='yellow', label='Saturation')
            else:
                ax.axvspan(sat, result.commands[-1], alpha=0.2, color='yellow', label='Saturation')
                
            direction = 'Backward' if result.commands[-1] < 0 else 'Forward'
            ax.set_xlabel('Command (normalized)')
            ax.set_ylabel('Speed (|counts/s|)')
            ax.set_title(f'{result.motor} Motor - {direction}')
            ax.grid(True, alpha=0.3)
            ax.legend()
            
        plt.tight_layout()
        plot_path = os.path.join(self.output_dir, "sweep_characteristics.png")
        plt.savefig(plot_path, dpi=150)
        plt.close()
        print(f"Sweep plot saved: {plot_path}")


def main():
    import argparse
    
    parser = argparse.ArgumentParser(description='Motor Characterization v2 - Telemetry Based')
    parser.add_argument('host', help='ESP32 IP address')
    parser.add_argument('--port', type=int, default=2333, help='WiFi console port')
    parser.add_argument('--step-duration', type=float, default=3.0, help='Step test duration (seconds)')
    parser.add_argument('--sweep-step', type=float, default=0.05, help='Sweep command step size')
    parser.add_argument('--telemetry-interval', type=int, default=20, help='Telemetry interval in milliseconds (default: 20ms)')
    parser.add_argument('--output', default='artifacts/motor_tests', help='Output directory')
    
    args = parser.parse_args()
    
    print("="*60)
    print("MOTOR CHARACTERIZATION v2")
    print("Telemetry Stream Based")
    print("="*60)
    print(f"\nTarget: {args.host}:{args.port}")
    print(f"Output: {args.output}")
    print("\n*** SAFETY CHECK ***")
    print("Ensure wheels are OFF THE GROUND and robot cannot move!")
    confirm = input("Type YES to continue: ")
    if confirm.upper() != "YES":
        print("Aborted.")
        return
        
    # Initialize
    char = MotorCharacterizer(args.host, args.port, args.output, args.telemetry_interval)
    char.connect()
    char.enable_telemetry()  # Initial setup - just enable logging
    
    time.sleep(0.5)
    
    try:
        # Step response tests for latency and tau
        step_results = []
        
        # LEFT forward
        step_results.append(char.run_step_test('LEFT', 1.0, args.step_duration))
        time.sleep(1.0)
        
        # LEFT backward  
        step_results.append(char.run_step_test('LEFT', -1.0, args.step_duration))
        time.sleep(1.0)
        
        # RIGHT forward
        step_results.append(char.run_step_test('RIGHT', 1.0, args.step_duration))
        time.sleep(1.0)
        
        # RIGHT backward
        step_results.append(char.run_step_test('RIGHT', -1.0, args.step_duration))
        time.sleep(1.0)
        
        # Sweep tests for deadzone and saturation
        sweep_results = []
        
        # Generate sweep commands (negative then positive, rounded to 2 decimals)
        # For negative: go from 0 to -1 (to avoid inertia affecting deadzone measurement)
        neg_commands = [round(-i * args.sweep_step, 2) for i in range(int(1.0 / args.sweep_step) + 1)]
        pos_commands = [round(i * args.sweep_step, 2) for i in range(int(1.0 / args.sweep_step) + 1)]
        
        # LEFT sweep (negative then positive)
        sweep_results.append(char.run_sweep_test('LEFT', neg_commands))
        time.sleep(1.0)
        sweep_results.append(char.run_sweep_test('LEFT', pos_commands))
        time.sleep(1.0)
        
        # RIGHT sweep (negative then positive)
        sweep_results.append(char.run_sweep_test('RIGHT', neg_commands))
        time.sleep(1.0)
        sweep_results.append(char.run_sweep_test('RIGHT', pos_commands))
        time.sleep(1.0)
        
        # Save results
        char.save_results(step_results, sweep_results)
        
        print("\n" + "="*60)
        print("CHARACTERIZATION COMPLETE!")
        print("="*60)
        print(f"\nResults saved to: {args.output}/")
        
    finally:
        char.disable_telemetry()
        if char.sock:
            char.sock.close()


if __name__ == '__main__':
    main()
