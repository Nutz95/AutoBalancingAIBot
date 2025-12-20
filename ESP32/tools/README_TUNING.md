# Balance Tuning Tools

## Quick Start

### Installation
```bash
pip install pyserial matplotlib numpy
```

### Basic Usage
```bash
# Auto-detect port, capture 10 seconds, plot results
python balance_tuning.py --plot

# Specify port and duration
python balance_tuning.py COM7 --duration 15 --plot

# Save data without plotting
python balance_tuning.py COM7 --duration 10 --output my_test.csv
```

## Workflow

1. **Position robot** - Hold upright, ready to balance
2. **Run script** - Script will:
   - Enable diagnostics
   - Start balancer
   - Capture data for specified duration
   - Stop balancer
   - Save CSV files
   - Plot results (if `--plot` specified)
3. **Analyze** - Look at plots:
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

## Output Files

- `balance_capture_YYYYMMDD_HHMMSS_diag.csv` - Diagnostics (pitch, rate, motor cmds)
- `balance_capture_YYYYMMDD_HHMMSS_balancer.csv` - Balancer (cmd, error, pitch)
- `balance_capture_YYYYMMDD_HHMMSS.png` - Plot (if --plot used)

## Capture helpers & archive

- The repo includes small helper scripts to capture TUNING / balancer telemetry and run basic analysis:
   - `ESP32/tools/capture_tuning.py` — Python capture helper (auto-detects port and saves CSV). Use `-p <COM>` to pick a port.
   - `ESP32/tools/capture_tuning.ps1` — PowerShell wrapper around the Python helper for Windows users.
   - `ESP32/tools/analyze_tuning_capture.py` — offline analyzer that produces PNGs and `summary.txt`.

   Agents note
   -----------
   For development workflows, agent configs (Planner/Coder/Verifier) are provided in `.github/agents/`.
   The `Coder` agent will respect the repository `CODING_RULES.md` when generating or updating firmware code.

- After a capture and analysis, archive the raw CSV and generated PNGs under the openspec change for the IMU fusion work, for example:
   - `openspec/changes/add-imufusion-madgwick/artifacts/plots/`
   - `openspec/changes/add-imufusion-madgwick/artifacts/manifest.txt`

This keeps tuning evidence with the corresponding design change and enables reproducible follow-ups.

## Tips

- Start with robot **almost vertical** (within ±10°)
- Test in **short bursts** to avoid crashes
- Save good gains: `BALANCE GAINS <kp> <ki> <kd>` (persists to NVS)
- Check gains: `BALANCE GET_GAINS`
- Use diagnostics channel: already enabled by `BALANCE START`
