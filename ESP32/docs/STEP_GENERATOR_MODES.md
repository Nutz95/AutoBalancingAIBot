# Step/Dir generation modes for MKS servo hybrid control

This note explains the three pulse-generation strategies that matter for this project:

- `RMT` (current default)
- `MCPWM`
- a possible future **Marlin-style software stepper scheduler**

It is intentionally practical and project-specific.

## Current architecture

The MKS hybrid driver uses two independent control paths:

- **RS485** for configuration, torque enable, and telemetry
- **Step/Dir** for speed command generation

In the current firmware:

- left motor Step pin: `MKS_SERVO_P1_STEP_PIN`
- right motor Step pin: `MKS_SERVO_P2_STEP_PIN`
- left motor Dir pin: `MKS_SERVO_P1_DIR_PIN`
- right motor Dir pin: `MKS_SERVO_P2_DIR_PIN`

Important:

- there is **one `m_stepGenerator` object** in `MksServoMotorDriver`
- but it drives **two independent output channels**, one for each motor
- it is **not** one pulse stream duplicated onto two GPIOs

So the software object is shared, but the hardware outputs are distinct.

## Mode selection in this project

The selection is controlled in `config/motor_configs/mks_servo_config.h`:

- `MKS_SERVO_USE_STEP_DIR`
- `MKS_SERVO_STEP_GENERATOR_TYPE`

Current meaning:

- `MKS_SERVO_STEP_GENERATOR_TYPE == 0` → `MCPWM`
- `MKS_SERVO_STEP_GENERATOR_TYPE == 1` → `RMT`

Current default in the repo:

- `MKS_SERVO_USE_STEP_DIR = 1`
- `MKS_SERVO_STEP_GENERATOR_TYPE = 1`

So the project currently uses **RMT**.

## RMT mode

### What it is

`RMT` is an ESP32 peripheral designed to emit timed high/low sequences very accurately.

In this project, it is used to generate a looped square wave on each Step pin.

### Hardware organization

The current implementation maps:

- left motor → `RMT_CHANNEL_2`
- right motor → `RMT_CHANNEL_3`

That means each motor gets its own dedicated RMT channel.

### Why it was chosen

RMT is attractive here because it tends to behave well at:

- low step frequencies
- stop/restart transitions
- fine timing control without a high-rate ISR

### Strengths

- very low CPU cost once configured
- precise pulse timing
- good low-speed behavior
- hardware keeps pulsing even if the control loop is busy for a short moment

### Weaknesses

- configuration/rearm sequence is more delicate than plain PWM
- concurrency bugs are easier to introduce if two tasks touch the same backend carelessly
- debugging is less intuitive than a software scheduler

### Why access serialization may still be needed

Even though left and right use different channels, both are still managed by:

- the same software object (`m_stepGenerator`)
- the same hardware peripheral family (`RMT`)
- the same driver API sequence (`stop -> write items -> start`)

So serialization is not about “one signal for both motors”.
It is about making concurrent updates deterministic.

## MCPWM mode

### What it is

`MCPWM` is a motor-oriented PWM peripheral. In this project it generates a square wave whose frequency is adjusted to match the desired step rate.

### Hardware organization

The current implementation uses:

- left motor → `MCPWM_TIMER_0`
- right motor → `MCPWM_TIMER_1`

Both live in `MCPWM_UNIT_0`.

### Strengths

- conceptually simple continuous frequency output
- good for medium/high frequencies
- often easier to reason about than RMT loop items

### Weaknesses

- can be less comfortable at very low frequencies or frequent stop/restart changes
- on this project, past tests suggested worse behavior than RMT for the current MKS hybrid setup

## Possible future: Marlin-style software step scheduler

### Idea

Instead of using a dedicated pulse peripheral, a timer ISR would periodically update both Step outputs in software.

This is close in spirit to how firmware like Marlin schedules step pulses:

- one central timing base
- per-axis accumulators / counters
- both motors updated from the same timing point

### Why this could be attractive

- logic is very explicit
- left/right synchronization can be reasoned about easily
- easier to instrument in software
- avoids some peripheral-specific edge cases

### Downsides

- higher CPU cost
- more ISR activity
- more jitter than a fully hardware-generated pulse train
- requires careful implementation to avoid hurting IMU/control responsiveness

### Is it viable here?

Probably yes, especially if:

- the real operating speeds remain moderate
- CPU headroom stays comfortable
- hardware backends continue to be painful to stabilize

It would not be the most elegant hardware solution, but it could be a very valid **robustness-first** fallback.

## Practical guidance

### If you want maximum hardware offload

Prefer `RMT`, but keep the implementation extremely conservative:

- deterministic ownership rules
- strong diagnostics
- minimal hidden state

### If you want simpler hardware behavior

Try `MCPWM` as a comparison backend.

### If hardware backends remain flaky

Prototype a Marlin-style software pulse driver for validation.

That mode could be especially useful as:

- a debug backend
- a fallback backend
- a reference implementation for expected pulse behavior

## Recommendation for this project

Short-term:

- keep the hardware Step/Dir approach
- continue stabilizing the currently selected RMT backend
- keep MCPWM available as a comparison path

Medium-term:

- if RMT and MCPWM both remain fragile in restart/recovery scenarios, add a software timed-step backend for comparison and debugging

## Related files

- `config/motor_configs/mks_servo_config.h`
- `include/motor_drivers/MksServoMotorDriver.h`
- `src/motor_drivers/MksServoMotorDriver.cpp`
- `include/motor_drivers/RmtStepGenerator.h`
- `src/motor_drivers/RmtStepGenerator.cpp`
- `include/motor_drivers/McpwmStepGenerator.h`
- `src/motor_drivers/McpwmStepGenerator.cpp`