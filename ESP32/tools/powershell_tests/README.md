# PowerShell test scripts for ESP32 serial commands

These small PowerShell scripts open a serial port and send test command sequences to the firmware running on the ESP32. They capture responses and print them to console and a timestamped log file.

Requirements
- Windows PowerShell (5.1) or PowerShell Core
- Device connected on a COM port (adjust scripts to use correct COM port)

Usage
- Run a script from PowerShell (example):
  - `powershell -NoProfile -ExecutionPolicy Bypass -File .\test_motor_commands.ps1 -PortName COM10 -BaudRate 921600`

Ctrl+C and the yellow message

If you see the message printed in yellow:

  Note: Ctrl+C handler registration not supported in this host; the script will rely on PowerShell interrupt to stop immediately.

it means your PowerShell host doesn't allow registering a `CancelKeyPress` handler. This is informational only — the script will still stop on Ctrl+C in a normal console.

`-ForcePoll` option

If you run the scripts in an environment where Ctrl+C does not reliably interrupt (some restricted hosts or editor integrations), you can enable a polling-based detection of Ctrl+C which checks console key events while the script is reading the serial port.

How to enable `-ForcePoll`

- Option A: pass `-ForcePoll` to the test script (recommended):

  ```powershell
  powershell -NoProfile -ExecutionPolicy Bypass -File .\test_motor_commands.ps1 -PortName COM10 -BaudRate 921600 -ForcePoll
  ```

- Option B: call `Open-SerialPort` yourself in an interactive session and pass `-ForcePoll`:

  ```powershell
  . .\serial_helper.ps1
  $sp = Open-SerialPort -PortName COM10 -BaudRate 921600 -ReadTimeoutMs 200 -ForcePoll
  ```

Notes
- Close any other serial monitor (PlatformIO Monitor, other terminal) before running the tests — only one process may open the port at a time.
- The helper reads any available raw bytes first (via `ReadExisting`) and then attempts `ReadLine()` to capture responses regardless of timing.
- If you'd like, I can add a `-ForcePoll` switch to the other test scripts as well; tell me which ones you run frequently and I'll add it.
