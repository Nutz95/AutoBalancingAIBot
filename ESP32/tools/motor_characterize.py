#!/usr/bin/env python3
"""
motor_characterize.py
- Connects to the ESP32 Wi‑Fi console and performs open-loop step tests
    to characterize motor response (time-to-rise, steady speed, latency).

Usage: python motor_characterize.py <host> [port] [--motor LEFT|RIGHT] [--outdir DIR]

Safety: Wheels MUST be off the ground or robot firmly lifted. The script
will enable motors and command raw velocity values. Use at your own risk.

Notes on sampling and estimator alpha:
 - The sampling interval (`--sample_dt`) controls how frequently the
     script reads encoder values during a step. For motor characterization
     a default of 50 ms (0.05 s) is a good trade-off between temporal
     resolution and console/firmware load.
 - The `SpeedEstimator` smoothing factor `alpha` should be chosen with
     the sampling interval in mind. A practical relation is:

             alpha = dt / (tau + dt)

     where `tau` is the desired estimator time-constant (seconds) and `dt`
     is the sampling interval in seconds. Example: for `tau = 0.2 s` and
     `dt = 0.05 s`, alpha ≈ 0.05 / (0.2 + 0.05) ≈ 0.20.
 - If you reduce `dt`, reduce `alpha` accordingly to preserve similar
     effective smoothing. Typical alpha values for characterization runs
     with `dt = 0.05` are in the 0.1–0.3 range depending on desired
     smoothing.
"""
import socket
import sys
import time
import re
import argparse
import os
import math

def make_socket():
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        s.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)
    except Exception:
        pass
    return s


class MotorChar:
    def __init__(self, host, port, motor_side, outdir):
        self.host = host
        self.port = port
        self.motor_side = motor_side.upper()
        self.outdir = outdir
        os.makedirs(outdir, exist_ok=True)
        self.sock = None
        # regex to capture encoder read output and present_pos from MOTOR PARAMS
        # old servo-style log: "motor_driver: encoder id=<id> val=<val>"
        self.re_enc = re.compile(r"motor_driver: encoder id=(\d+) val=(-?\d+)")
        # firmware prints: "motor_driver: encoder side=<n> val=<v>" (side: 0=LEFT,1=RIGHT)
        self.re_enc_side = re.compile(r"motor_driver: encoder side=(\d+) val=(-?\d+)")
        # serial_commands presentation: "MOTOR: encoder id=<id> value=<val>"
        self.re_enc_motor = re.compile(r"MOTOR: encoder id=(\d+) value=(-?\d+)")
        # combined MOTOR both-side format: "MOTOR: encoder L(id=X)=V R(id=Y)=W"
        self.re_enc_motor_both = re.compile(r"MOTOR: encoder L\(id=(\d+)\)=(-?\d+)\s+R\(id=(\d+)\)=(-?\d+)")
        # DCMirrorDriver status line that prints left_enc/right_enc
        self.re_dc_both = re.compile(r"DCMirrorDriver:.*left_enc=(-?\d+).*right_enc=(-?\d+)")
        self.re_present = re.compile(r"present_pos=(-?\d+)")
        self.buffer = b''
        # track last raw position for unwrapping and accumulated position
        self._last_raw_pos = None
        self._accumulated_pos = 0
        # whether to send normalized speeds (-1..1) instead of raw ints
        self.use_normalized = False

    def connect(self, timeout=5):
        s = make_socket()
        s.settimeout(timeout)
        s.connect((self.host, self.port))
        s.settimeout(None)
        self.sock = s
        # prepare console log file
        try:
            self._console_log_path = os.path.join(self.outdir, 'console_log.txt')
            # touch/clear file
            with open(self._console_log_path, 'w', encoding='utf-8') as _f:
                _f.write('--- console log start ---\n')
        except Exception:
            self._console_log_path = None

    def close(self):
        if self.sock:
            try:
                self.sock.shutdown(socket.SHUT_RDWR)
            except Exception:
                pass
            self.sock.close()
            self.sock = None

    def send(self, line):
        if not line.endswith('\n'):
            line = line + '\n'
        self.sock.sendall(line.encode('utf-8'))

    def _recv_lines(self, timeout=0.1):
        # read available data and return decoded lines
        self.sock.settimeout(timeout)
        out = []
        try:
            data = self.sock.recv(4096)
            if not data:
                return out
            self.buffer += data
            while b'\n' in self.buffer:
                line, self.buffer = self.buffer.split(b'\n', 1)
                try:
                    s = line.decode('utf-8', errors='ignore').strip()
                except Exception:
                    s = repr(line)
                out.append(s)
        except Exception:
            pass
        finally:
            self.sock.settimeout(None)
        return out

    def _log_console_line(self, line):
        if not getattr(self, '_console_log_path', None):
            return
        try:
            with open(self._console_log_path, 'a', encoding='utf-8') as f:
                f.write(line + '\n')
        except Exception:
            pass

    def wait_for_any(self, substrings, timeout=2.0):
        """Read console lines until any substring in `substrings` appears or timeout.
        Returns the matched line or None on timeout."""
        deadline = time.time() + timeout
        while time.time() < deadline:
            for l in self._recv_lines(0.1):
                for s in substrings:
                    if s in l:
                        return l
        return None

    

    

    

    def read_encoder(self, id_token):
        # Prefer MOTOR PARAMS (present_pos single-turn) and perform unwrap locally
        # Fallback: MOTOR READ (accumulated) or MOTOR POS printed accumulated
        # 1) MOTOR PARAMS -> try to read present_pos and unwrap/accumulate
        self.send(f"MOTOR PARAMS {id_token}")
        deadline = time.time() + 0.8
        while time.time() < deadline:
            for l in self._recv_lines(0.15):
                self._log_console_line(l)
                m = self.re_present.search(l)
                if m:
                    try:
                        raw = int(m.group(1))
                        # Unwrap: single-turn raw in 0..4095
                        if self._last_raw_pos is None:
                            self._last_raw_pos = raw
                            return int(self._accumulated_pos)
                        delta = raw - self._last_raw_pos
                        # handle wrap-around
                        if delta > 2048:
                            delta -= 4096
                        elif delta < -2048:
                            delta += 4096
                        self._accumulated_pos += delta
                        self._last_raw_pos = raw
                        return int(self._accumulated_pos)
                    except Exception:
                        continue

        # 2) MOTOR READ (ask firmware for accumulated value)
        self.send(f"MOTOR READ {id_token}")
        deadline = time.time() + 0.6
        while time.time() < deadline:
            for l in self._recv_lines(0.15):
                self._log_console_line(l)
                # try multiple encoder line formats (servo, MOTOR:, DC driver)
                m = self.re_enc.search(l)
                if m:
                    try:
                        return int(m.group(2))
                    except Exception:
                        continue
                m_side = self.re_enc_side.search(l)
                if m_side:
                    try:
                        side_idx = int(m_side.group(1))
                        val = int(m_side.group(2))
                        tok = id_token.upper() if id_token is not None else ''
                        if tok.startswith('L') or tok == 'LEFT':
                            if side_idx == 0:
                                return val
                        if tok.startswith('R') or tok == 'RIGHT':
                            if side_idx == 1:
                                return val
                        # if numeric id or unspecified, prefer side_idx==0 (LEFT)
                        return val
                    except Exception:
                        continue
                m2 = self.re_enc_motor.search(l)
                if m2:
                    try:
                        return int(m2.group(2))
                    except Exception:
                        continue
                m3 = self.re_enc_motor_both.search(l)
                if m3:
                    try:
                        tok = id_token.upper() if id_token is not None else ''
                        if tok.startswith('L'):
                            return int(m3.group(2))
                        elif tok.startswith('R'):
                            return int(m3.group(4))
                        else:
                            # default to left value
                            return int(m3.group(2))
                    except Exception:
                        continue
                m4 = self.re_dc_both.search(l)
                if m4:
                    try:
                        tok = id_token.upper() if id_token is not None else ''
                        if tok.startswith('L'):
                            return int(m4.group(1))
                        elif tok.startswith('R'):
                            return int(m4.group(2))
                        else:
                            return int(m4.group(1))
                    except Exception:
                        continue

    def run_step_test(self, motor_token, step_speeds, step_duration=1.0, sample_dt=0.02):
        results = []
        # Ensure motor telemetry is enabled on the console and motors enabled
        print('Enabling MOTOR logs and motors...')
        # Request WiFi console to forward motor channel
        self.send('LOG ENABLE MOTOR')
        # Wait for confirmation that LOG was enabled (printed on DEFAULT channel)
        ok = self.wait_for_any(['LOG: enabled', 'LOG enabled'], timeout=2.0)
        if ok:
            print('Console confirmed motor logging enabled.')
        else:
            print('Warning: did not see console confirmation for LOG ENABLE MOTOR; continuing anyway.')

        # Now enable motors and wait for servo/init messages
        self.send('MOTOR ENABLE')
        # Listen for common motor init messages (servo UART init, encoder init, or motors ENABLED)
        init_ok = self.wait_for_any(['motor_driver: servos set to Wheel mode', 'motor_driver: encoder init raw', 'motor_driver: motors ENABLED', 'motor_driver: initializing servo UART'], timeout=3.0)
        if init_ok:
            print('Console shows motor init:', init_ok)
        else:
            print('Warning: did not observe servo initialization messages; telemetry may be unavailable.')
        # for each step
        for sp in step_speeds:
            label = 'normalized' if self.use_normalized else 'raw'
            # format display to 2 decimal places for readability and to avoid float artifacts
            try:
                display_sp = f"{float(sp):.2f}" if self.use_normalized else str(int(sp))
            except Exception:
                display_sp = str(sp)
            print(f'Commanding {motor_token} speed {display_sp} ({label}) for {step_duration}s')
            # clear any pending input
            time.sleep(0.05)
            # send VEL direct; integer raw for servos, float normalized for DC
            if self.use_normalized:
                # For normalized commands prefer MOTOR SET (handled by AbstractMotorDriver)
                # which maps to setMotorCommand(side, -1..1). Send rounded value with 2 decimals.
                try:
                    send_val = f"{float(sp):.2f}"
                except Exception:
                    send_val = str(sp)
                self.send(f"MOTOR SET {motor_token} {send_val}")
            else:
                self.send(f"MOTOR VEL {motor_token} {int(sp)}")
            t0 = time.time()
            samples = []
            while (time.time() - t0) < step_duration:
                enc = self.read_encoder(motor_token)
                samples.append((time.time(), enc))
                time.sleep(sample_dt)
            # stop motor (0) - use same command family as used for step (SET vs VEL)
            if self.use_normalized:
                self.send(f'MOTOR SET {motor_token} 0')
            else:
                self.send(f'MOTOR VEL {motor_token} 0')
            time.sleep(0.05)
            results.append((sp, samples))
        # disable motors at end
        print('Disabling motors...')
        self.send('MOTOR DISABLE')
        return results


def _parse_dc_config(path='config/motor_configs/dc_motor_config.h'):
    """Parse DC PWM config values from firmware header. Returns (max_speed, pwm_bits).
    Falls back to defaults if file or defines are not present.
    """
    max_speed = 7000
    pwm_bits = 8
    try:
        with open(path, 'r', encoding='utf-8') as f:
            data = f.read()
        m = re.search(r"#define\s+DC_VELOCITY_MAX_SPEED\s+(\d+)", data)
        if m:
            max_speed = int(m.group(1))
        m2 = re.search(r"#define\s+DC_PWM_RESOLUTION_BITS\s+(\d+)", data)
        if m2:
            pwm_bits = int(m2.group(1))
    except Exception:
        pass
    return max_speed, pwm_bits


def _print_mapping_for_speeds(speeds, normalized=True):
    max_speed, pwm_bits = _parse_dc_config()
    pwm_max = (1 << pwm_bits) - 1
    print(f"DC mapping: DC_VELOCITY_MAX_SPEED={max_speed}, PWM_BITS={pwm_bits}, PWM_MAX={pwm_max}")
    print("norm/raw/duty/duty%")
    for s in speeds:
        if normalized:
            cmd = float(s)
            raw = int(round(cmd * float(max_speed)))
        else:
            raw = int(s)
            cmd = float(raw) / float(max_speed) if max_speed != 0 else 0.0
        absv = abs(raw)
        duty = 0
        if max_speed != 0:
            duty = int((absv * pwm_max) // max_speed)
        duty_pct = (duty / pwm_max * 100.0) if pwm_max > 0 else 0.0
        print(f"{cmd:+.4f} / {raw:6d} / {duty:3d} / {duty_pct:6.2f}%")
    


def analyze_and_save(outdir, motor_token, results):
    # Save CSV per step and print simple metrics (steady delta)
    # Collect estimates for motor gain and dynamics
    K_per_raw_list = []
    tau_list = []
    latency_list = []
    for sp, samples in results:
        if not samples:
            continue
        fname = os.path.join(outdir, f"motor_{motor_token}_step_{sp}.csv")
        # find baseline: first non-None encoder sample for this step
        base = None
        for (_t, _v) in samples:
            if _v is not None:
                base = _v
                break
        if base is None:
            base = 0
        # write CSV with time offset and encoder values adjusted so first
        # observed sample becomes zero (baseline subtracted)
        with open(fname, 'w', encoding='utf-8') as f:
            f.write('t,enc\n')
            t0 = samples[0][0] if len(samples) > 0 else 0.0
            for (t, enc) in samples:
                if enc is None:
                    enc_out = ''
                else:
                    enc_out = int(enc - base)
                f.write(f"{t - t0:.6f},{enc_out}\n")
        print(f'Saved {fname} ({len(samples)} samples)')
        # crude metrics: first non-none, last non-none (adjusted by baseline)
        vals = [(t, v - base) for (t, v) in samples if v is not None]
        if len(vals) >= 2:
            t_start, v_start = vals[0]
            t_end, v_end = vals[-1]
            delta = v_end - v_start
            duration = t_end - t_start if (t_end - t_start) > 0 else 1.0
            # latency: first sample where change exceeds noise threshold
            noise_thresh = 2  # encoder counts
            latency = None
            for (t, v) in vals:
                if abs(v - v_start) >= noise_thresh:
                    latency = t - t_start
                    break
            # rise time approx: time to reach 63% of final delta
            tau_time = None
            target = v_start + 0.63 * delta
            if delta != 0:
                for (t, v) in vals:
                    if (delta > 0 and v >= target) or (delta < 0 and v <= target):
                        tau_time = t - t_start
                        break
            # steady slope estimate: linear fit on last 40% of samples
            import math
            n = len(vals)
            i0 = max(0, int(n * 0.6))
            xs = [vals[i][0] - vals[i0][0] for i in range(i0, n)]
            ys = [vals[i][1] for i in range(i0, n)]
            slope = None
            if len(xs) >= 2 and not math.isclose(xs[-1], xs[0]):
                # simple linear regression
                xm = sum(xs) / len(xs)
                ym = sum(ys) / len(ys)
                num = sum((xs[i] - xm) * (ys[i] - ym) for i in range(len(xs)))
                den = sum((xs[i] - xm) ** 2 for i in range(len(xs)))
                slope = num / den if den != 0 else None

            print(f'  raw speed {sp}: delta={delta} counts over {duration:.3f}s')
            print(f'    latency ~ {latency:.3f}s' if latency is not None else '    latency: no detectable movement')
            print(f'    tau (~63%) = {tau_time:.3f}s' if tau_time is not None else '    tau: not reached')
            if slope is not None:
                print(f'    steady slope ≈ {slope:.1f} counts/sec')
                # estimate K (counts/sec per unit raw speed)
                try:
                    K_est = slope / float(sp)
                    K_per_raw_list.append(K_est)
                except Exception:
                    pass
            if tau_time is not None:
                tau_list.append(tau_time)
            if latency is not None:
                latency_list.append(latency)
            else:
                print('    steady slope: insufficient data')
        else:
            print('  no encoder samples')

    # write a summary CSV
    summary_path = os.path.join(outdir, 'summary.csv')
    with open(summary_path, 'w', encoding='utf-8') as f:
        f.write('raw_speed,delta,latency_s,tau_s,steady_counts_per_sec\n')
        for sp, samples in results:
            # use same baseline-subtracted convention for summary
            base = None
            for (_t, _v) in samples:
                if _v is not None:
                    base = _v
                    break
            if base is None:
                base = 0
            vals = [(t, v - base) for (t, v) in samples if v is not None]
            if len(vals) < 2:
                f.write(f"{sp},,,\n")
                continue
            t_start, v_start = vals[0]
            t_end, v_end = vals[-1]
            delta = v_end - v_start
            duration = t_end - t_start if (t_end - t_start) > 0 else 1.0
            noise_thresh = 2
            latency = ''
            for (t, v) in vals:
                if abs(v - v_start) >= noise_thresh:
                    latency = f"{t - t_start:.6f}"
                    break
            tau_time = ''
            target = v_start + 0.63 * delta
            if delta != 0:
                for (t, v) in vals:
                    if (delta > 0 and v >= target) or (delta < 0 and v <= target):
                        tau_time = f"{t - t_start:.6f}"
                        break
            # slope on last 40%
            n = len(vals)
            i0 = max(0, int(n * 0.6))
            xs = [vals[i][0] - vals[i0][0] for i in range(i0, n)]
            ys = [vals[i][1] for i in range(i0, n)]
            slope = ''
            if len(xs) >= 2 and xs[-1] != xs[0]:
                xm = sum(xs) / len(xs)
                ym = sum(ys) / len(ys)
                num = sum((xs[i] - xm) * (ys[i] - ym) for i in range(len(xs)))
                den = sum((xs[i] - xm) ** 2 for i in range(len(xs)))
                if den != 0:
                    slope = f"{(num/den):.3f}"
            f.write(f"{sp},{delta},{latency},{tau_time},{slope}\n")
    print(f'Saved summary: {summary_path}')
    # Collect commanded vs measured steady speeds for plotting
    cmd_vals = []
    meas_vals = []
    with open(summary_path, 'r', encoding='utf-8') as f:
        next(f)
        for line in f:
            parts = line.strip().split(',')
            if len(parts) < 5:
                continue
            try:
                cmd = float(parts[0])
                slope_s = parts[4]
                if slope_s != '':
                    meas = float(slope_s)
                    cmd_vals.append(cmd)
                    meas_vals.append(meas)
            except Exception:
                continue
    # Generate a plot of encoder vs time for each step and save to outdir
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        import math

        plt.figure(figsize=(10, 6))
        ax = plt.gca()
        for sp, samples in results:
            if not samples:
                continue
            t0 = samples[0][0]
            # compute baseline (first non-None) and subtract so plots start at 0
            base = None
            for (_t, _v) in samples:
                if _v is not None:
                    base = _v
                    break
            if base is None:
                base = 0
            xs = [s[0] - t0 for s in samples]
            ys = [(float(s[1]) - base) if s[1] is not None else float('nan') for s in samples]
            ax.plot(xs, ys, marker='o', label=f'{sp} raw')

        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Encoder (counts, baseline-subtracted)')
        ax.set_title('Motor step responses (encoder vs time)')
        ax.grid(True)
        ax.legend()
        out_png = os.path.join(outdir, 'plot.png')
        plt.tight_layout()
        plt.savefig(out_png, dpi=200)
        print(f'Saved plot: {out_png}')
        # plot commanded vs measured steady speeds
        try:
            if len(cmd_vals) > 0:
                plt.figure(figsize=(6, 6))
                ax2 = plt.gca()
                ax2.scatter(cmd_vals, meas_vals, marker='o')
                ax2.set_xlabel('Commanded speed (raw or normalized)')
                ax2.set_ylabel('Measured steady speed (counts/sec)')
                ax2.set_title('Commanded vs Measured Steady Speed')
                ax2.grid(True)
                # optional linear fit
                try:
                    import numpy as _np
                    a, b = _np.polyfit(cmd_vals, meas_vals, 1)
                    xs = _np.array([min(cmd_vals), max(cmd_vals)])
                    ys = a * xs + b
                    ax2.plot(xs, ys, linestyle='--', color='orange', label=f'fit: y={a:.3f}x+{b:.3f}')
                    ax2.legend()
                except Exception:
                    pass
                out2 = os.path.join(outdir, 'cmd_vs_meas.png')
                plt.tight_layout()
                plt.savefig(out2, dpi=200)
                print(f'Saved plot: {out2}')
        except Exception as e:
            print('Command vs measured plotting failed:', e)
    except Exception as e:
        print('Plotting failed:', e)

    # Suggest simple PI gains for a velocity loop based on measured K and tau
    try:
        import statistics as _stats
        if len(K_per_raw_list) > 0:
            K_med = _stats.median(K_per_raw_list)
        else:
            K_med = None
        tau_med = _stats.median(tau_list) if len(tau_list) > 0 else None
        lat_med = _stats.median(latency_list) if len(latency_list) > 0 else None

        if K_med is not None and tau_med is not None:
            # Conservative desired closed-loop time constant (seconds)
            combo = ( (lat_med if lat_med is not None else 0.0) + tau_med )
            Tc_des = max(0.05, 0.5 * combo)
            # For model G(s)=K/(tau*s+1), choose Kp so that tau/(1+K*Kp) ~ Tc_des
            # => 1+K*Kp = tau / Tc_des  => Kp = (tau / Tc_des - 1)/K
            denom = K_med
            if denom == 0:
                raise ZeroDivisionError
            Kp_vel = max(0.0, (tau_med / Tc_des - 1.0) / denom)
            # Set Ki so integral time ~ tau_med (Ti = tau) => Ki = Kp / Ti
            Ki_vel = Kp_vel / tau_med if tau_med > 0 else 0.0

            print('\nVelocity-loop PI suggestion (approx):')
            print(f'  Estimated K (counts/sec per raw unit): {K_med:.3f}')
            print(f'  Median latency: {lat_med if lat_med is not None else float("nan"):.3f} s')
            print(f'  Median tau: {tau_med:.3f} s')
            print(f'  Desired closed-loop time-constant Tc_des: {Tc_des:.3f} s')
            print(f'  Suggested KP_vel ≈ {Kp_vel:.6f} (raw units -> controller output scale)')
            print(f'  Suggested KI_vel ≈ {Ki_vel:.6f} (per second)')
            print('\nNotes:')
            print(' - These are conservative starting values for an inner velocity PI loop.')
            print(' - If you have an inner velocity loop in firmware, try these as starting gains.')
            print(' - For the outer balancer PID (angle -> motor command), use much smaller gains')
            print('   because actuator latency (~0.3s) is large; prefer cascade control (angle->speed).')
        else:
            print('\nInsufficient data to suggest PI gains (need slope and tau measurements).')
    except Exception as e:
        print('Gain suggestion failed:', e)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('host')
    parser.add_argument('port', nargs='?', type=int, default=2333)
    parser.add_argument('--motor', choices=['LEFT', 'RIGHT'], default='LEFT')
    parser.add_argument('--outdir', default='artifacts/motor_tests')
    parser.add_argument('--speeds', default='2000,4000,6000,7000', help='comma list of speeds; raw ints or normalized floats depending on --normalized')
    parser.add_argument('--steps', help='generate speeds using start:end:step (e.g. -1:1:0.1). Example: --steps "-0.3:0.3:0.05"')
    parser.add_argument('--range', nargs=3, type=float, metavar=('START','END','STEP'),
                        help='alternate: provide start end step as three floats (avoids shell quoting for negative start). Example: --range -0.3 0.3 0.05')
    parser.add_argument('--normalized', action='store_true', help='treat speeds as normalized values in [-1,1] (for DC motors)')
    parser.add_argument('--dur', type=float, default=10.0, help='duration (s) for each step')
    parser.add_argument('--sample_dt', type=float, default=0.05, help='sampling interval (s). Note: choose estimator alpha using alpha = dt/(tau+dt); for dt=0.05 and tau=0.2 use alpha≈0.2')
    parser.add_argument('--show-mapping', action='store_true', help='print normalized->raw->PWM mapping for the generated speeds and exit')
    parser.add_argument('--precision', type=float, default=0.01, help='round normalized speeds to this precision (e.g. 0.01)')
    parser.add_argument('--two-phase', action='store_true', help='run two sweeps: 0->-X then 0->+X (useful to avoid momentum bias when crossing deadzone)')
    parser.add_argument('--discard-seconds', type=float, default=0.0, help='seconds to discard at start of each step when analyzing (not yet applied to CSV)')
    args = parser.parse_args()

    print('*** MOTOR CHARACTERIZATION TEST ***')
    print('Safety check: ensure wheels are off the ground and robot cannot move.')
    ok = input('Type YES to confirm and continue: ')
    if ok.strip().upper() != 'YES':
        print('Aborted by user')
        return

    mc = MotorChar(args.host, args.port, args.motor, args.outdir)
    mc.use_normalized = args.normalized
    try:
        mc.connect()
        # Allow generating speeds from --steps or parse explicit --speeds
        if args.range:
            try:
                start = float(args.range[0])
                end = float(args.range[1])
                step = float(args.range[2])
                if step == 0:
                    raise ValueError('step cannot be zero')
                vals = []
                v = start
                if step > 0:
                    while v <= end + 1e-9:
                        vals.append(v)
                        v += step
                else:
                    while v >= end - 1e-9:
                        vals.append(v)
                        v += step
                if args.normalized:
                    speeds = [float(f) for f in vals]
                else:
                    speeds = [int(f) for f in vals]
            except Exception as e:
                print('Invalid --range value:', e)
                return
        elif args.steps:
            try:
                parts = args.steps.split(':')
                if len(parts) != 3:
                    raise ValueError('steps must be start:end:step')
                start = float(parts[0])
                end = float(parts[1])
                step = float(parts[2])
                # build inclusive sequence respecting floating step sign
                vals = []
                v = start
                if step == 0:
                    raise ValueError('step cannot be zero')
                if step > 0:
                    while v <= end + 1e-9:
                        vals.append(v)
                        v += step
                else:
                    while v >= end - 1e-9:
                        vals.append(v)
                        v += step
                if args.normalized:
                    speeds = [float(f) for f in vals]
                else:
                    speeds = [int(f) for f in vals]
            except Exception as e:
                print('Invalid --steps value:', e)
                return
        else:
            if args.normalized:
                speeds = [float(s.strip()) for s in args.speeds.split(',') if s.strip()]
            else:
                speeds = [int(s.strip()) for s in args.speeds.split(',') if s.strip()]

        # Quantize normalized speeds to requested precision to avoid floating artifacts
        if args.normalized and args.precision and args.precision > 0:
            prec = float(args.precision)
            # number of decimal places for formatting
            try:
                nd = max(0, int(round(-math.log10(prec))))
            except Exception:
                nd = 2
            def quant(v):
                q = round(round(v / prec) * prec, nd)
                # treat very small values as zero to avoid tiny floats like 1e-16
                if abs(q) < (prec / 2.0):
                    q = 0.0
                return q
            speeds = [quant(s) for s in speeds]
        # If requested, print mapping and exit (useful to inspect quantization)
        if args.show_mapping:
            _print_mapping_for_speeds(speeds, normalized=args.normalized)
        else:
            # support two-phase sweep: 0->-X then 0->+X to avoid momentum carryover
            if args.two_phase and len(speeds) > 0:
                # determine max magnitude from provided speeds
                max_neg = abs(min(0.0, min(speeds)))
                max_pos = abs(max(0.0, max(speeds)))
                max_range = max(max_neg, max_pos)
                if max_range == 0:
                    print('Two-phase requested but max range is 0; aborting two-phase.')
                    res = mc.run_step_test(args.motor, speeds, step_duration=args.dur, sample_dt=args.sample_dt)
                    analyze_and_save(args.outdir, args.motor, res)
                else:
                    # determine step increment (take difference of first two if possible)
                    step_inc = args.precision if args.precision and args.precision > 0 else 0.01
                    if len(speeds) >= 2:
                        step_inc = abs(speeds[1] - speeds[0])
                        if step_inc == 0:
                            step_inc = args.precision
                    # build negative sweep 0 -> -max_range
                    neg_vals = []
                    v = 0.0
                    while v >= -max_range - 1e-9:
                        neg_vals.append(round(v, 6))
                        v -= step_inc
                    # ensure last point is exactly -max_range
                    if abs(neg_vals[-1] + max_range) > 1e-9:
                        neg_vals.append(-max_range)
                    # build positive sweep 0 -> +max_range
                    pos_vals = []
                    v = 0.0
                    while v <= max_range + 1e-9:
                        pos_vals.append(round(v, 6))
                        v += step_inc
                    if abs(pos_vals[-1] - max_range) > 1e-9:
                        pos_vals.append(max_range)

                    # quantize sequences using same precision logic as above
                    def _quant_list(lst):
                        if args.normalized and args.precision and args.precision > 0:
                            prec = float(args.precision)
                            try:
                                nd = max(0, int(round(-math.log10(prec))))
                            except Exception:
                                nd = 2
                            def q(v):
                                qq = round(round(v / prec) * prec, nd)
                                if abs(qq) < (prec / 2.0):
                                    qq = 0.0
                                return qq
                            return [q(x) for x in lst]
                        else:
                            return lst

                    neg_q = _quant_list(neg_vals)
                    pos_q = _quant_list(pos_vals)

                    print(f'Running two-phase: 0 -> {-max_range} ({len(neg_q)} steps), then 0 -> {max_range} ({len(pos_q)} steps)')
                    res1 = mc.run_step_test(args.motor, neg_q, step_duration=args.dur, sample_dt=args.sample_dt)
                    res2 = mc.run_step_test(args.motor, pos_q, step_duration=args.dur, sample_dt=args.sample_dt)
                    combined = []
                    combined.extend(res1)
                    combined.extend(res2)
                    analyze_and_save(args.outdir, args.motor, combined)
            else:
                res = mc.run_step_test(args.motor, speeds, step_duration=args.dur, sample_dt=args.sample_dt)
                analyze_and_save(args.outdir, args.motor, res)
    finally:
        mc.close()


if __name__ == '__main__':
    main()
