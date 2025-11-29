# Build and run unit tests (MSYS2)

This document explains how to build and run the host-native unit tests on Windows using MSYS2 / MinGW-w64.

The project provides comprehensive test binaries (`imu_fusion_tests.cpp` and `pid_controller_tests.cpp` in `ESP32/test/`) which test the IMU fusion algorithm and PID controller implementation.

Goal: produce native Windows executables from sources in `ESP32/` and run the test binaries locally.

## ⚠️ IMPORTANT: Confirmed Working Method (Updated 2025-11-29)

**g++ compilation only works reliably from within the MSYS2 MinGW64 shell.**

Direct invocation from PowerShell or Python fails silently (exit code 1, no error output) due to environment/DLL issues.

### ✅ Recommended: Use the bash script

```bash
# 1. Open 'MSYS2 MinGW 64-bit' from Windows Start Menu
# 2. Navigate to the test directory:
cd /i/GIT/AutoBalancingAIBot/ESP32/test

# 3. Run the test script:
./build_test.sh
```

This script compiles and runs both IMU and PID tests successfully.

### ✅ Alternative: Manual compilation in MSYS2

```bash
cd /i/GIT/AutoBalancingAIBot/ESP32

# IMU Fusion test
g++ -std=c++17 -O2 -I include test/imu_fusion_tests.cpp src/imu_fusion.cpp \
    -o build/imu_fusion_tests.exe -DM_PI=3.14159265358979323846
build/imu_fusion_tests.exe

# PID Controller test
g++ -std=c++17 -O2 -I include test/pid_controller_tests.cpp src/pid_controller.cpp \
    -o build/pid_controller_tests.exe
build/pid_controller_tests.exe
```

### ❌ Known Issue: PowerShell/Python compilation fails

~~The provided `build_test.ps1` and `build_test.py` scripts~~ PowerShell and Python scripts were tested but **fail to compile** when run from Windows due to g++ environment issues. Only the bash script `build_test.sh` works reliably in MSYS2.

---

Prerequisites
- 64-bit Windows
- Administrator access to install MSYS2 (recommended)
- Internet access to download MSYS2 and packages

Install MSYS2 (recommended)
1. Download the installer from https://www.msys2.org
2. Install MSYS2 following the official instructions.
3. Start the `MSYS2 MinGW 64-bit` shell (important: use the MinGW64 session to get the x86_64 toolchain).

Update MSYS2 and install the toolchain
In the `MSYS2 MinGW 64-bit` shell run:

```bash
pacman -Syu
# close and re-open the shell if pacman requests it
pacman -Su
pacman -S --needed base-devel mingw-w64-x86_64-toolchain
```

This installs `g++` and the MinGW-w64 toolchain (x86_64).

Compile the test (example)
From the `MSYS2 MinGW 64-bit` shell, change to the project folder and compile the comprehensive suite:

```bash
cd /i/GIT/AutoBalancingAIBot/ESP32
mkdir -p build
g++ -std=c++17 -O2 -I include test/imu_fusion_tests.cpp src/imu_fusion.cpp -o build/imu_fusion_tests.exe -DM_PI=3.14159265358979323846
```

Notes:
- `-I include` uses the local headers in `ESP32/include`.
- We compile both the test and the implementation (`src/imu_fusion.cpp`) together.
- `-DM_PI=...` defines `M_PI` to avoid editing sources.

Compiling the PID unit test (example)
-----------------------------------
To compile the host PID unit test that lives in `test/pid_controller_tests.cpp`, compile both the test and the implementation together:

```bash
cd /i/GIT/AutoBalancingAIBot/ESP32
mkdir -p build
g++ -std=c++17 -O2 -I include test/pid_controller_tests.cpp src/pid_controller.cpp -o build/pid_controller_tests.exe
```

Notes and troubleshooting for silent failures
-------------------------------------------
- **Known Issue (PowerShell Direct g++ Invocation)**: When running `D:\msys64\mingw64\bin\g++.exe` directly from PowerShell (outside MSYS), compilation may fail with exit code 1 and zero error output. This is a system/environment issue (DLL or path mapping incompatibility). **Workaround**: Compile from within an MSYS2 MinGW64 shell or run `powershell.exe -File build_test.ps1` from inside MSYS.
- If your PowerShell run of the script returns "Compilation failed (exit code 1, no executable created)" with no other compiler output, try running the compile command directly in the **MSYS2 MinGW 64-bit** shell — MSYS2's g++ prints native diagnostics there which can be easier to capture.
- Ensure the `mingw-w64-x86_64-toolchain` is installed and you are using the *MinGW64* session (or have `mingw64\bin` on your PATH when running from PowerShell).
- If the test compiles in MSYS2 but fails from PowerShell, try running the command from PowerShell but adding the MSYS `mingw64/bin` to PATH first, for example:

```powershell
$mingw = 'D:\msys64\mingw64\bin'
$env:Path = "$mingw;$env:Path"
g++ -std=c++17 -O2 -I include test/pid_controller_tests.cpp src/pid_controller.cpp -o build/pid_controller_tests.exe
```

- If the compiler still exits with code 1 and no output, try invoking `g++` via the MSYS bash and capture stderr, e.g. from PowerShell:

```powershell
cmd /c "D:\msys64\usr\bin\bash.exe -lc 'g++ -std=c++17 -O2 -I /i/GIT/AutoBalancingAIBot/ESP32/include /i/GIT/AutoBalancingAIBot/ESP32/test/pid_controller_tests.cpp /i/GIT/AutoBalancingAIBot/ESP32/src/pid_controller.cpp -o /i/GIT/AutoBalancingAIBot/ESP32/build/pid_controller_tests.exe'"
```

This often surfaces errors that are swallowed by certain PowerShell redirections.

Run the test
Still in the MSYS2 shell:

```bash
./build/imu_fusion_tests.exe
```

Or from PowerShell (if `mingw64/bin` is on your `PATH` or you run the executable directly):

```powershell
cd 'I:\GIT\AutoBalancingAIBot\ESP32'
.\build\imu_fusion_tests.exe
```

Troubleshooting
- `g++: command not found` → ensure the `mingw-w64-x86_64-toolchain` package is installed and use the `MSYS2 MinGW 64-bit` shell.
- `M_PI` undefined → the example command passes `-DM_PI=...` to define it.
- Missing symbols → add the additional source files that define them to the compile command.
- Windows path mapping: `I:\...` is available in MSYS2 as `/i/...`.

Integration with PlatformIO
- To build firmware for the ESP32, continue using PlatformIO (`pio run`) from the `ESP32` folder.
- The host-native tests shown here are for algorithm verification on the PC; they do not exercise the flashed ESP32 firmware.

Expected output
The comprehensive test prints a sequence of PASSES and FAIL / EXPECTED FAIL markers. Key points:

- The suite runs a set of tests that are independent of a magnetometer (convergence, noise robustness, fuzz, etc.).
- Magnetometer-dependent tests (X/Y-up orientations) are run twice by the test binary:
  1. non-strict run — prints `EXPECTED FAIL (no magnetometer)` and does not count as a failure (default behavior),
  2. strict run — counts failures (useful when a magnetometer is present or simulated).

Example (partial) output:

```
PASS: Quaternion normalization
PASS: Stationary Z-up
... (other passes)

NOTE: Running magnetometer-dependent tests (may fail without magnetometer)
EXPECTED FAIL (no magnetometer): Stationary X-up
EXPECTED FAIL (no magnetometer): Stationary -X-up
... (other expected fails)

--- Magnetometer-dependent tests: strict (simulate mag, failures will count) ---
Skipping strict magnetometer-dependent checks (RUN_STRICT_MAG=false)

Failures: 0
```

If you want to enable the strict phase (make those checks count as failures), set the `RUN_STRICT_MAG`
toggle at the top of `ESP32/test/imu_fusion_tests.cpp` to `true`, or compile with `-DRUN_STRICT_MAG=1`.

Optional: using MSVC
If you prefer Microsoft Visual C++, open a "Developer Command Prompt" and run:

```powershell
cl /EHsc /std:c++17 /I include test\imu_fusion_tests.cpp src\imu_fusion.cpp /Fe:build\imu_fusion_tests.exe /D M_PI=3.14159265358979323846
```

FAQ
- Q: Can I run the binary from PowerShell without MSYS2?
  A: Yes, as long as the binary is a native Windows executable and any required runtime components are available.
- Q: Why MSYS2 instead of Cygwin?
  A: MSYS2/MinGW-w64 produces native Windows binaries without a heavy POSIX compatibility DLL, which is preferable for distributing and running tests locally.

If you'd like, I can also:
- add a short section to `ESP32/README.md` linking to this test README, or
- create a small script to compile the test automatically.

File created by the assistant.

---
**Quick: compile & run from PowerShell**
- Prerequisite: `D:\msys64\mingw64\bin` (or equivalent) should be reachable from PowerShell. The included script will add it to `PATH` temporarily if needed.
- To compile and run automatically from PowerShell:
```powershell
cd 'I:\GIT\AutoBalancingAIBot\ESP32\test'
.\build_test.ps1
```
- What the script does:
  - detects `g++` (MinGW-w64);
  - compiles `test/imu_fusion_unit.cpp` + `src/imu_fusion.cpp` to `build\imu_fusion_unit.exe`;
  - adds `mingw64\bin` to the session `PATH` temporarily so runtime DLLs are found;
  - runs the test and prints output.

Troubleshooting note:
- If the executable is created but exits with an error like `0xC0000139`, ensure `mingw64\bin` is on the PATH of the session that runs the executable or run the binary from the MSYS2 MinGW64 shell.
