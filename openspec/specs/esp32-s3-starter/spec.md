# ESP32-S3 PlatformIO Starter Project

Add an empty ESP32-S3 project in `ESP32/` that can be built using PlatformIO. This starter will provide the minimal structure and configuration for development with the ESP32-S3 and Arduino framework.

## Purpose

Provide a minimal, buildable PlatformIO starter project for ESP32-S3 (Arduino framework) under the `ESP32/` folder so developers can quickly build and extend firmware for the AutoBalancingAIBot.
## Requirements
### Requirement: Project scaffold

The system SHALL create a PlatformIO-compatible project scaffold under `ESP32/` containing `platformio.ini`, `src/main.cpp`, and README files for `include/`, `lib/`, and `test/`.

#### Scenario: Scaffold created

- **WHEN** the change is applied
- **THEN** the `ESP32/` folder exists with `platformio.ini`, `src/main.cpp`, and `README.md` present

### Requirement: Buildable with PlatformIO

The project SHALL build successfully using `pio run` with the `esp32-s3-devkitc-1` environment defined in `platformio.ini`.

#### Scenario: Build success

- **WHEN** a developer runs `pio run` inside `ESP32/`
- **THEN** the build completes without errors for the `esp32-s3-devkitc-1` environment

### Requirement: Tooling configuration

The `platformio.ini` SHALL include reasonable default upload and monitor settings to simplify development: `upload_port`, `upload_speed`, `monitor_port`, and `monitor_speed`.

#### Scenario: Tooling defaults present

- **WHEN** the scaffold is generated
- **THEN** `platformio.ini` contains `upload_port = COM11`, `upload_speed = 921600`, `monitor_port = COM10`, and `monitor_speed = 921600` (these are defaults and MAY be overridden by developers)

### Requirement: Minimal runtime behavior

The runtime SHALL print a heartbeat message to serial (e.g., "Hello ESP32-S3!") once per second.

#### Scenario: Serial heartbeat

- **WHEN** the board is powered and connected to serial monitor at 115200
- **THEN** the device prints "Hello ESP32-S3!" at ~1s intervals

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

## Project Structure

```text
ESP32/
  README.md
  platformio.ini
  src/
    main.cpp
  include/
    README
  lib/
    README
  test/
    README
```

## platformio.ini

```ini
[env:esp32-s3-devkitc-1]
platform = espressif32
board = esp32-s3-devkitc-1
framework = arduino
lib_deps = Adafruit NeoPixel, workloads/SCServo, Adafruit SSD1306, https://github.com/Nutz95/bmi088-arduino.git
upload_port = COM11
upload_speed = 921600
monitor_port = COM10
monitor_speed = 921600
```

## src/main.cpp

```cpp
#include <Arduino.h>

void setup() {
    Serial.begin(115200);
}

void loop() {
    Serial.println("Hello ESP32-S3!");
    delay(1000);
}
```

## README.md

Minimal description of the ESP32-S3 starter project, usage instructions, and build steps.

## include/README, lib/README, test/README

Minimal description or left empty.

## Acceptance Criteria

- The structure and files above are created in `ESP32/`
- The project builds successfully with PlatformIO
- No application logic beyond a serial print in `main.cpp`

## Notes

- This change does not overwrite existing files unless confirmed
- Ready for extension with libraries and application code
