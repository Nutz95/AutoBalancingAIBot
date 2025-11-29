# PID Autotuning Guide

## Overview

The autotuning system uses the **Relay Feedback Method** (Åström-Hägglund) to automatically calculate optimal PID gains for your balancing robot. This is safer and faster than manual trial-and-error tuning.

## How It Works

1. **Relay Control**: Applies bang-bang control (±30% power) to induce oscillations
2. **Oscillation Measurement**: Records pitch zero-crossings and peak amplitudes
3. **Gain Calculation**: Computes ultimate gain (Ku) and period (Tu) from oscillations
4. **Tyreus-Luyben Tuning**: Calculates conservative PID gains:
   - `Kp = 0.45 × Ku`
   - `Ki = 0` (start without integral)
   - `Kd = Kp × Tu / 6.3`

## Safety Features

- **Pitch limit**: Aborts if robot tilts beyond ±50°
- **Timeout**: Stops after 10 seconds if no oscillation detected
- **Cycle validation**: Requires 3 consistent oscillation cycles
- **Manual stop**: Can abort anytime with `AUTOTUNE STOP`

## Usage

### Serial Commands

```
AUTOTUNE START     - Start autotuning (HOLD THE ROBOT!)
AUTOTUNE STOP      - Abort autotuning
AUTOTUNE STATUS    - Check current state
AUTOTUNE APPLY     - Apply calculated gains (after COMPLETE)
```

### Menu Navigation

1. Enter main menu (press any key)
2. Select "5 - Balancer (PID)"
3. Select "8 - AUTOTUNE START"
4. **IMPORTANT: Hold the robot** - it will oscillate!
5. Wait for completion message (~5-8 seconds)
6. Select "11 - AUTOTUNE APPLY" to use the new gains
7. Test with "1 - BALANCE START"

## Procedure

### Step 1: Preparation

```
MOTOR ENABLE          # Enable motors
AUTOTUNE START        # Start tuning process
```

**CRITICAL**: As soon as you start autotuning, **physically hold the robot** to prevent it from falling. The robot will oscillate back and forth - this is normal and expected!

### Step 2: During Tuning

- Hold the robot upright but allow it to pitch forward/backward
- Don't restrict the oscillations - let it move naturally
- Watch serial output for status messages
- Typical duration: 5-8 seconds

### Step 3: Results

When complete, you'll see:

```
AUTOTUNE: COMPLETE - use 'AUTOTUNE APPLY' to set gains
```

Check the calculated gains:

```
AUTOTUNE STATUS
```

Output example:

```
AUTOTUNE: COMPLETE (use AUTOTUNE APPLY to set gains)
```

### Step 4: Apply Gains

```
AUTOTUNE APPLY
```

Output example:

```
AUTOTUNE: Applied gains - Kp=7.2000 Ki=0.0000 Kd=9.1500 | Ku=16.00 Tu=402.3ms Amp=14.5°
```

### Step 5: Test Balance

```
BALANCE START         # Test the new gains
```

If balance is unstable, you can:
- Run `AUTOTUNE START` again (results may vary each run)
- Manually adjust with `BALANCE GAINS <kp> <ki> <kd>`
- Try reducing Kp by 10-20% if oscillations persist

## Troubleshooting

### "Timeout - no oscillation detected"

**Cause**: Robot not oscillating enough

**Solution**:
- Give the robot a gentle push when autotuning starts
- Check that motors are enabled (`MOTOR ENABLE`)
- Verify you're holding the robot loosely enough to oscillate

### "Pitch exceeded safety limit"

**Cause**: Robot tilted too far (>50°)

**Solution**:
- Hold the robot more firmly
- Start from a more upright position
- Check mechanical balance (center of gravity)

### Gains seem too aggressive

**Cause**: Tyreus-Luyben rules are conservative but may still overshoot

**Solution**:
- Reduce Kp: `BALANCE GAINS <kp*0.8> 0.0 <kd>`
- Increase Kd: `BALANCE GAINS <kp> 0.0 <kd*1.2>`

### Robot falls during test

**Cause**: Holding too loosely or mechanical issues

**Solution**:
- Practice the motion first with motors disabled
- Use a safety tether or catch mechanism
- Verify mechanical assembly is solid

## Theory: Why This Works

The relay method forces the system to oscillate at its natural frequency. By measuring:

- **Ultimate Period (Tu)**: Time for one complete oscillation
- **Oscillation Amplitude**: Peak-to-peak pitch angle
- **Ultimate Gain (Ku)**: Gain at which system oscillates indefinitely

We can calculate stable PID gains without risking instability or crashes.

**Advantages over Ziegler-Nichols**:
- Safer (no need to find stability boundary)
- Faster (3-4 cycles vs 10+ cycles)
- More reliable (relay is consistent)
- Can recover from falls (just restart)

## Advanced: Custom Configuration

To modify autotune parameters, edit `balancer_controller_impl.cpp`:

```cpp
void startAutotune() {
    AutotuneController::Config config;
    config.relay_amplitude = 0.3f;       // ±30% power (decrease if too violent)
    config.deadband = 0.5f;              // Pitch deadband in degrees
    config.max_pitch_abort = 50.0f;      // Safety limit
    config.timeout_ms = 10000;           // Max time to wait
    config.min_cycles = 3;               // Cycles needed for averaging
    
    g_autotune.start(&config);
    // ...
}
```

## Integration with Manual Tuning

Autotuning provides a good **starting point**, but you may want to fine-tune manually:

1. Run autotune to get baseline gains
2. Capture data: `python balance_tuning.py COM10 --duration 10 --plot`
3. Analyze plots (see `README_TUNING.md`)
4. Adjust gains based on observed behavior:
   - **Too much overshoot**: Reduce Kp
   - **Too slow response**: Increase Kp
   - **High-frequency oscillations**: Increase Kd
   - **Steady-state error**: Add small Ki (0.1-0.5)

## Next Steps

After autotuning:

1. **Validate**: Test balance for 30+ seconds
2. **Capture data**: Use `balance_tuning.py` to record performance
3. **Fine-tune**: Make small adjustments if needed
4. **Persist**: Gains are auto-saved to NVS after `AUTOTUNE APPLY`

See `README_TUNING.md` for detailed PID tuning methodology and data analysis.
