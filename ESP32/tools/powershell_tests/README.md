# PowerShell test scripts for ESP32 serial commands

These small PowerShell scripts open a serial port and send test command sequences to the firmware running on the ESP32. They capture responses and print them to console and a timestamped log file.

Requirements
- Windows PowerShell (5.1) or PowerShell Core
- Device connected on a COM port (adjust scripts to use correct COM port)

Usage
- Edit the scripts to set the correct `$PortName` (e.g. `COM11`) and optionally the baud rate.
- Run a script from PowerShell:
  - `.
un_gyro_calib.ps1`
  - `.
un_accel_calib.ps1`
  - `.	est_motor_commands.ps1`
  - `.	est_persistence.ps1`

Notes
- The scripts assume the firmware echoes status lines (e.g., `CALIB PROGRESS`, `CALIB DONE`, `motor_driver:` lines).
- For safety, the `test_motor_commands.ps1` uses moderate values and disables motors at the end. If you switch to the real driver, immobilize the robot before running.
