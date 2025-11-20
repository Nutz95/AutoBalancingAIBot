## ADDED Requirements

### Requirement: ESP32-S3 starter project
The system SHALL provide a PlatformIO-compatible starter project for ESP32-S3 under `ESP32/`, including `platformio.ini`, `src/main.cpp`, and README files.

#### Scenario: Starter available
- **WHEN** the change is applied
- **THEN** the `ESP32/` folder contains `platformio.ini`, `src/main.cpp`, and `README.md`

### Requirement: Tooling defaults
`platformio.ini` SHALL include default `upload_port`, `upload_speed`, `monitor_port`, and `monitor_speed` settings to simplify developer workflow.

#### Scenario: Tooling defaults
- **WHEN** the starter is generated
- **THEN** `platformio.ini` contains reasonable defaults (e.g., `COM11`, `921600`, `COM10`, `921600`)
