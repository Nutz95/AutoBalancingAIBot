## Implementation Tasks

- [ ] 1. Create capability delta `openspec/changes/add-bmi088-reader/specs/bmi088/spec.md`
- [ ] 2. Implement `ESP32/include/BMI088Config.h` and `ESP32/include/BMI088Driver.h`
- [ ] 3. Implement `ESP32/src/BMI088Driver.cpp`
- [ ] 5. Implement IMU FreeRTOS tasks (producer/consumer) using a single-slot queue
- [ ] 4. Add `lib_deps` to `ESP32/platformio.ini` for the BMI088 Arduino lib
- [ ] 5. Validate change with `openspec validate add-bmi088-reader --strict`
- [ ] 6. Build project and ensure compilation (`pio run`)
 
Notes:
- Steps 3 and 4 already implemented; next is to add tasks and validate.
