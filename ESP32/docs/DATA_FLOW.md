# Data Flow & Control Architecture

This document describes the high-represenation synchronous control loop used by the AutoBalancingBot.

## Control Loop Architecture

The system uses a dual-core approach on the ESP32-S3:
- **Core 1 (High Priority)**: Processes IMU data, sensor fusion, and the LQR control law at a strict 1000Hz.
- **Core 0 (System)**: Handles Wi-Fi, Bluetooth (NimBLE), and low-priority background tasks.

```mermaid
graph TD
    subgraph Sensors
        IMU[BMI160 IMU 1000Hz]
        ENC[RS485 Encoders 100Hz]
    end

    subgraph "Core 1 - Real-Time Control"
        Producer[IMU Producer Task]
        Consumer[IMU Consumer Task 1000Hz]
        Fusion[Fusion Complementary1D]
        Interp[Encoder Interpolator]
        LQR[Cascaded LQR Strategy]
        StepGen[Step Gen RMT 400Hz]
    end

    IMU -- "Raw SPI" --> Producer
    Producer -- "Queue" --> Consumer
    Consumer --> Fusion
    ENC -- "RS485 Sub-task" --> Interp
    
    Fusion -- "Pitch / Rate" --> LQR
    Interp -- "Smoothed Position/Speed" --> LQR
    
    LQR -- "Command (Normalized -1..1)" --> StepGen
    StepGen -- "GPIO Step/Dir" --> Motors[MKS Motors]
    
    subgraph "Background Adaptation"
        Trim[Adaptive Trim Integrator]
    end
    
    LQR -. "Drift Error" .-> Trim
    Trim -. "Shifted Setpoint" .-> LQR
```

## Data Path Details

1. **IMU Path**: Raw data is read via SPI at 1kHz. The `Producer` task pushes samples to a queue for the `Consumer`.
2. **Fusion Path**: `Complementary1D` filter computes pitch and pitch-rate with zero phase lag.
3. **Odometry Path**: Encoders are read via RS485 at 100Hz on independent buses. A 1st-order hold `Interpolator` upsamples this to 1000Hz for the LQR.
4. **Control Path**: The `CascadedLqrStrategy` computes the sum of four states:
   - `Kp * angle_error`
   - `Kg * pitch_rate`
   - `Kd * position_error`
   - `Ks * velocity_error`
5. **Actuation Path**: The normalized command is converted to a pulse frequency and sent to the `RMT` (Remote Control) peripheral to generate hardware Step/Dir signals.
