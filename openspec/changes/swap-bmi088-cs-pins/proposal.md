# Change proposal: swap-bmi088-cs-pins

## Summary
This change corrects the default chip-select pin assignments for the BMI088 sensor: the accelerometer CS and gyroscope CS were reversed. Update the `BMI088Config` defaults, add an OpenSpec delta describing the change, and provide small verification steps.

## Motivation
Users reported that accelerometer values corresponded to gyroscope outputs and vice-versa. The root cause is that the default CS pins were inverted. Correcting the defaults avoids misconfiguration for users relying on the scaffolded values.

## Scope
- Update `ESP32/include/BMI088Config.h` to set `accel_cs_pin = 4` and `gyro_cs_pin = 14`.
- Add OpenSpec change documents under `openspec/changes/swap-bmi088-cs-pins/` describing the requirement delta and tasks.
 - Apply small driver fixes discovered during debugging:
	 - Ensure SPI helper performs a dummy transfer before reading registers (matches library semantics).
	 - Use correct SPI modes for each chip (ACCEL: `SPI_MODE0`, GYRO: `SPI_MODE3`).
	 - Add WHO_AM_I probes and concise diagnostics to confirm mapping at runtime.

## Acceptance criteria
- The code default values are updated and build cleanly.
- Validation steps in `tasks.md` pass: a quick runtime smoke test prints consistent accel/gyro values to Serial and WHO_AM_I IDs are reported (accel expected 0x1E, gyro expected 0x0F or documented alternative ID returned by some modules).
