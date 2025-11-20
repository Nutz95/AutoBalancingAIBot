# Change: add-esp32-s3-starter

## Why
Provide a minimal, buildable ESP32-S3 (Arduino / PlatformIO) starter project under `ESP32/` so developers can quickly begin firmware work for AutoBalancingAIBot.

## What Changes
- Add a capability spec describing the ESP32-S3 starter
- Create scaffold files under `ESP32/` (platformio.ini, src/main.cpp, README.md and placeholders)
- Include default tooling settings in `platformio.ini` (upload/monitor ports and speeds)

## Impact
- A new `openspec/specs/esp32-s3-starter/spec.md` capability is present (current truth)
- Implementation files added under the `ESP32/` folder
- No breaking changes to existing code

## Acceptance Criteria
- `ESP32/` contains `platformio.ini`, `src/main.cpp`, and `README.md`
- `platformio.ini` contains reasonable defaults for upload and monitor (`COM11`/`921600`, `COM10`/`921600`)
- The project builds with PlatformIO (`pio run`) using `esp32-s3-devkitc-1`
