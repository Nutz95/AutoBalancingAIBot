# imu-warmup Specification

## Purpose
TBD - created by archiving change add-imufusion-warmup-init. Update Purpose after archive.
## Requirements
### Requirement: Fusion warmup API

The firmware MUST expose a warmup/init API for the IMU fusion module that
allows collection of a configurable number of IMU samples and returns a
readiness indicator. The API MUST include functions similar to:

- `beginWarmup(count)` — start a warmup sequence collecting `count` samples.
- `isReady()` — return true when warmup is complete and fusion is initialized.
- `getWarmupProgress()` — return a 0..1 progress fraction for monitoring.

#### Scenario: Warmup API availability

- Given the firmware contains an IMU fusion module,
- When code calls `beginWarmup(200)`,
- Then the fusion module collects samples and `getWarmupProgress()` increments
  until `isReady()` returns true.

### Requirement: Balancer safety gating

The balancer MUST check `fusion.isReady()` before enabling motors or closing
the control loop. If the fusion is not ready the balancer MUST remain in a
safe state and refuse to enable outputs until readiness is achieved or a
documented override is used.

#### Scenario: Balancer gating

- Given the system boots and fusion warmup is in progress,
- When the operator issues `BALANCE START`,
- Then the balancer SHALL NOT enable motors until `fusion.isReady()` is true,
  and SHALL inform the operator via status output that warmup is required.

### Requirement: LED warmup indicator

The firmware SHALL provide a minimal status LED API and use it to indicate
fusion warmup state: red while warming, green when ready. Boards without a
configured LED SHALL leave the LED off and operate normally.

#### Scenario: LED indicates warmup

- Given the board has a configured status LED,
- When fusion warmup starts, the system SHALL set LED to red; when warmup
  completes, it SHALL set LED to green.

