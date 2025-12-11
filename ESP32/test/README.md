# Running Unit Tests

This directory contains host-native unit tests for the ESP32 balancer firmware.

## Prerequisites

**MSYS2 with MinGW-w64 toolchain** is required to compile and run these tests on Windows.

### Installing MSYS2

1. Download the installer from [https://www.msys2.org](https://www.msys2.org)
2. Run the installer and follow the instructions (default location: `C:\msys64`)
3. After installation, open **MSYS2 MSYS** terminal and update the package database:
   ```bash
   pacman -Syu
   ```
4. Close and reopen the terminal if prompted, then complete the update:
   ```bash
   pacman -Su
   ```
5. Install the MinGW-w64 toolchain (includes g++):
   ```bash
   pacman -S --needed base-devel mingw-w64-x86_64-toolchain
   ```
6. Close the MSYS terminal

## Running the Tests

### Method 1: Using the build script (Recommended)

1. Open **MSYS2 MinGW 64-bit** from the Windows Start Menu
   - ⚠️ **Important**: Use "MSYS2 MinGW 64-bit", NOT "MSYS2 MSYS" or "MSYS2 UCRT64"

2. Navigate to the test directory:
   ```bash
   cd /i/GIT/AutoBalancingAIBot/ESP32/test
   ```
   **Note**: This example assumes your repository is at `I:\GIT\AutoBalancingAIBot` on Windows.
   - Windows drives are accessed as `/c/`, `/d/`, `/i/`, etc. in MSYS2
   - Adjust the path according to your actual repository location
   - Example: If your repo is at `C:\Projects\AutoBalancingAIBot`, use `/c/Projects/AutoBalancingAIBot/ESP32/test`

3. Run the build script:
   ```bash
   ./build_test.sh
   ```

This will automatically:
- Compile the IMU Fusion test
- Run the IMU tests
- Compile the PID Controller test  
- Run the PID tests
- Display results for both

### Method 2: Manual compilation

From the **MSYS2 MinGW 64-bit** shell:

```bash
cd /i/GIT/AutoBalancingAIBot/ESP32

# Compile and run IMU Fusion test
g++ -std=c++17 -O2 -I include test/imu_fusion_tests.cpp src/imu_fusion.cpp \
    -o build/imu_fusion_tests.exe -DM_PI=3.14159265358979323846
build/imu_fusion_tests.exe

# Compile and run PID Controller test
g++ -std=c++17 -O2 -I include test/pid_controller_tests.cpp src/pid_controller.cpp \
    -o build/pid_controller_tests.exe
build/pid_controller_tests.exe
```

**Note**: Replace `/i/GIT/AutoBalancingAIBot` with your actual repository path in MSYS2 format.

## Test Files

- `imu_fusion_tests.cpp` - Comprehensive IMU fusion algorithm tests (Madgwick filter)
- `pid_controller_tests.cpp` - PID controller unit tests (proportional, integral, derivative, anti-windup)
- `build_test.sh` - Bash script to compile and run all tests

## Expected Output

When tests pass successfully, you should see:

```
======================================================================
Building and running host-native unit tests
======================================================================

----------------------------------------------------------------------
1. IMU Fusion Test
----------------------------------------------------------------------
✅ Compilation succeeded

Running IMU Fusion Test...
Running IMU fusion tests...
PASS: Quaternion normalization
PASS: Stationary Z-up
PASS: High rotation rate stable norm
PASS: Zero accel does not produce NaN
PASS: Noise robustness within 2 deg
PASS: Linear acceleration affects estimate (non-zero pitch)
PASS: Pitch rate finite after variable dt and spike
PASS: Fuzz test completed without non-finite quaternion norm
PASS: Gravity-vector consistency (angle < 5 deg)

--- Magnetometer-dependent tests: non-strict (expected failures without mag) ---
NOTE: Running magnetometer-dependent tests (may fail without magnetometer)
EXPECTED FAIL (no magnetometer): Stationary X-up
EXPECTED FAIL (no magnetometer): Stationary -X-up
EXPECTED FAIL (no magnetometer): Stationary Y-up
EXPECTED FAIL (no magnetometer): Stationary -Y-up

Failures: 0

----------------------------------------------------------------------
2. PID Controller Test
----------------------------------------------------------------------
✅ Compilation succeeded

Running PID Controller Test...
PID tests: PASS

----------------------------------------------------------------------
3. Driver Manager Test
----------------------------------------------------------------------
✅ Compilation succeeded

Running Driver Manager Test...
driver_manager tests: PASS

======================================================================
✅ All tests completed successfully!
======================================================================
```

## Troubleshooting

### "g++: command not found"
- Make sure you installed the toolchain: `pacman -S mingw-w64-x86_64-toolchain`
- Ensure you're using **MSYS2 MinGW 64-bit** shell (not MSYS or UCRT64)

### "Permission denied" when running ./build_test.sh
```bash
chmod +x build_test.sh
```

### Tests fail to compile from Windows PowerShell or Command Prompt
The g++ compiler from MSYS2 **only works within the MSYS2 shell environment**. You must use the MSYS2 MinGW 64-bit terminal to compile and run these tests.

### Path issues (drive letters)
In MSYS2, Windows drives are mounted at `/c/`, `/d/`, `/i/`, etc.
Example: `I:\GIT\project` becomes `/i/GIT/project`

## Additional Resources

See `README_MSYS2.md` for detailed MSYS2 installation instructions and advanced troubleshooting.
