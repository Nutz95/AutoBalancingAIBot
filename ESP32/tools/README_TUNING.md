# Balance Tuning Tools

## Motor Characterization

### Prerequisites

```bash
pip install numpy matplotlib
```

### Motor Characterization Script

Use `motor_characterize_v2.py` to measure motor dynamics (latency, tau, deadzone, saturation) via WiFi console.

**Safety First**: Ensure wheels are **OFF THE GROUND** before running tests!

### Basic Usage

```bash
# Connect to ESP32 via WiFi and run full characterization
python motor_characterize_v2.py 192.168.1.29 --step-duration 3.0

# Custom sweep resolution
python motor_characterize_v2.py 192.168.1.29 --sweep-step 0.05 --output artifacts/motor_tests

# Use different port (default: 2333)
python motor_characterize_v2.py 192.168.1.29 --port 2333
```

### What It Measures

1. **Latency** - Time from command entry to first motor response (~15-25ms expected)
2. **Tau** - Time constant to reach 63% of steady-state speed (~100-200ms typical)
3. **Deadzone** - Command range where motor doesn't respond (typically ±0.10 to ±0.20)
4. **Saturation** - Command range where speed plateaus (typically >0.95)
5. **Linear fit** - y=ax+b relationship between command and speed in linear range

### Output Files

All results saved to `artifacts/motor_tests/` (or custom `--output` path):

- `characterization_summary.txt` - Text summary with all measurements
- `step_responses.png` - 2x2 grid showing step response curves (LEFT/RIGHT, forward/backward)
- `sweep_characteristics.png` - 2x2 grid showing deadzone/saturation curves with linear fit
- `console_log.txt` - Raw console log for debugging

### Interpreting Results

**Step Response Plots**:

- Blue curve shows measured speed over time
- Red dashed lines mark 63% point (tau) and latency
- Faster rise = lower tau (more responsive motor)

**Sweep Plots**:

- Red zone = Deadzone (motor doesn't move)
- Green zone = Linear range (speed ∝ command)
- Yellow zone = Saturation (speed plateaus)
- Green dashed line = Linear fit equation (use for speed prediction)

### Balance Tuning Workflow

1. **Run motor characterization** (wheels off ground):

   ```bash
   python motor_characterize_v2.py 192.168.1.29
   ```

2. **Update firmware constants** with measured values:
   - `MOTOR_LATENCY_S` = measured latency (e.g., 0.019)
   - `MOTOR_TAU_S` = measured tau (e.g., 0.140)
   - `MOTOR_DEADZONE` = measured deadzone (e.g., 0.15)

3. **Position robot** - Hold upright, ready to balance

4. **Tune PID gains** - Use balance tuning helpers (see below)

5. **Analyze balance performance** - Look at plots:
   - **Pitch oscillations** → too much Kp or not enough Kd
   - **Drift over time** → add Ki (but usually not needed)
   - **Slow response** → increase Kp
   - **High frequency noise** → reduce Kd or add filtering

## PID Tuning Guidelines

### Current gains

```
Kp = 10.0
Ki = 0.0
Kd = 5.0
```

### Symptoms & Solutions

**Oscillates rapidly** (high frequency)

- Reduce Kd: `BALANCE GAINS 10.0 0.0 3.0`
- Or reduce Kp: `BALANCE GAINS 8.0 0.0 5.0`

**Oscillates slowly** (low frequency, overshoots)

- Increase Kd: `BALANCE GAINS 10.0 0.0 8.0`

**Too slow to respond**

- Increase Kp: `BALANCE GAINS 15.0 0.0 5.0`

**Drifts away from center over time**

- Add small Ki: `BALANCE GAINS 10.0 0.1 5.0`
- Warning: Ki can cause instability, start small!

**Falls over immediately**

- Check sign (already fixed)
- Reduce gains: `BALANCE GAINS 5.0 0.0 2.0`

### Systematic Tuning (Ziegler-Nichols inspired)

1. Set Ki=0, Kd=0, start with low Kp=1.0
2. Increase Kp until sustained oscillations: `Kp_critical`
3. Measure oscillation period: `T_critical` (seconds)
4. Calculate gains:
   - `Kp = 0.6 * Kp_critical`
   - `Ki = 1.2 * Kp_critical / T_critical`
   - `Kd = 0.075 * Kp_critical * T_critical`
5. Fine-tune from there

## Balance Capture Helpers

- The repo includes helper scripts to capture balance telemetry during tuning:
  - `ESP32/tools/capture_tuning.py` — Python capture helper (auto-detects port and saves CSV). Use `-p <COM>` to pick a port.
  - `ESP32/tools/capture_tuning.ps1` — PowerShell wrapper around the Python helper for Windows users.
  - `ESP32/tools/analyze_tuning_capture.py` — offline analyzer that produces PNGs and `summary.txt`.

## Development Notes

   For development workflows, agent configs (Planner/Coder/Verifier) are provided in `.github/agents/`.
   The `Coder` agent will respect the repository `CODING_RULES.md` when generating or updating firmware code.

   After a capture and analysis, archive the raw CSV and generated PNGs under the openspec change, for example:

- `openspec/changes/add-imufusion-madgwick/artifacts/plots/`
- `openspec/changes/add-imufusion-madgwick/artifacts/manifest.txt`

   This keeps tuning evidence with the corresponding design change and enables reproducible follow-ups.

## Tips

- Start with robot **almost vertical** (within ±10°)
- Test in **short bursts** to avoid crashes
- Save good gains: `BALANCE GAINS <kp> <ki> <kd>` (persists to NVS)
- Check gains: `BALANCE GET_GAINS`
- Use diagnostics channel: already enabled by `BALANCE START`
