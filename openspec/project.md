# Project Context

## Purpose
An autobalancing AI bot with 2 servo motors (STS 3215 HS from Waveshare), using ESP32-S3 as controller and connected to a BMI088 gyroscope through SPI.
The bot has 2 22mm wheels on each side, a platform with all the electronics on top and a camera on a small mast.

## Tech Stack
- ESP32-S3 code using PlatformIO. The ESP will be dedicated to balancing the robot and controlling the motion of the robot (Left/Right/Forward/Backward)
- Maixpy/MaixCDK to control the Maixcam Camera with Yolo model. It will be connected to the ESP32S3 through RX/TX and will tell the ESP32 where to go (Go forward/Backward a certain amount (cm as int), turn right/left (a certain angle in decimal)).
- HTML/CSS to provide a simple control interface on the ESP32S3 for basic motions (forward/backward, Turn Left/Right with angles)
- The ESP32S3 code will be event driven as much as possible.
- The Feetech STS 3215-HS servos are using RX/TX protocol and are linked in series.

### Waveshare STS 3215 HS Product page
The official product page is located here: https://www.waveshare.com/wiki/ST3215-HS_Servo_Motor

### Waveshare STS 3215 HS Servo specifications
Here are the spec for the Waveshare STS 3215 HS servos:
    Input Voltage: 6-12.6V
    Mechanical Limited Angle: No Limit
    Rotating Angle: 360° (servo mode angle control)/motor mode continuous rotation
    Baudrate: 1Mbps
    Gear: high-precision metal gear
    Idling Speed: 0.094sec/60° (106RPM)@12V
    Position Sensor Resolution: 360°/4096
    ID Range: 0-253
    Feedback: Position, Load, Speed, Input Voltage
    Load Current: 240mAA
    Locked-rotor Current: 2.4A
    Dimension: 45.22mm x 35mm x 24.72mm
### Waveshare STS 3215 HS Features
    High speed, with a maximum speed of up to 106 RPM.
    Support connecting in series, simultaneously controlling up to 253 servo motors (provided that there is sufficient power) and obtaining feedback information from each motor.
    360° magnetic encoder, allowing for a wider angle control range.
    High precision, with an angle control accuracy of 360°/4096.
    Any angle can be set as the motor's midpoint, facilitating assembly.
    Acceleration and deceleration function for smoother motion effects.
    Compact structural design for a more aesthetically pleasing product appearance.
    Wide voltage input range of 6-12.6V, capable of direct power supply from 2s or 3s lithium batteries.
    High torque, up to 20kg.cm@12V.
    Programmable work modes: Servo mode for angle control / Motor mode for continuous rotation.

## Project Conventions
### Documentation Update

After each complete OpenSpec iteration (spec application and code generation), update the project documentation (README.md, specs, and relevant doc files) to reflect the latest changes, features, and architecture. This ensures the documentation stays synchronized with the codebase and project evolution.
### Code Style

- Use C++ for the ESP32 code
- Use MaixPy Micropython for Maixcam code
- For the Arduino code for the ESP32, create a class that will serve as a manager for each device/functionality (Servos, BMI088, HTML Server, AutoBalancer (PID, KalmanFilter, etc...), Configuration)
- Follow SOLID principles as much as possible
- Avoid duplicated code
- Create clear class and function names that make sense for what the class/functions are doing. Long names are acceptable if they improve clarity.
- Create a Log class to handle Info/Debug/Error log to Serial output (ESP32S3 WROOM Dev kit has a special USB Serial Output for that purpose. Use it.)
- The log level will be configurable via a constant (e.g., LOG_LEVEL), and the Log class will provide functions for each log type (Info, Debug, Error).

### Architecture Patterns

- The code for the ESP32S3 will be located in the /ESP32/ folder and its subfolders.
- The code for Maixcam Micropython will be located in the /Maixcam/ folder.

- The start of the program will instantiate all components/needed classes/setup constants/setup basic HTML server.
- The ESP32S3 will have a main loop.
- A timer will be configured to be triggered at 200Hz. A function will be called to query the Gyroscope/Accelerometer and update its dedicated class/component (BMI088 class).
- A timer will call a function to autobalance the robot. It needs to be adjusted according to the balancing needs.
- Mechanisms to handle the HTML server connections/commands.
- An event-based mechanism to handle the RX/TX commands from the Maixcam.
- On BMI088/AutoBalancer/ServoDriver classes, add logs that can be reused in tests. (Gyro/Accelerometer data, ServoDriver commands sent for debug, AutoBalancer inputs and outputs for debug purposes.)
- Add a constant DEBUG or LOG_LEVEL to trigger Debug logs and select the log level.
- Use the 3rd party Arduino library to control the BMI088 (BMI08x-Arduino from Bosch Sensortec: https://github.com/bolderflight/bmi088-arduino.git).
- Use the official SCServo library from Feetech to drive the STS 3215 HS servos.

### Testing Strategy

- Create unit tests for at least the Balancing PID algorithm, executable without access to the hardware (tests on PC or simulator).

### Git Workflow

No need for complicated workflow, simple commits will do fine. No need for branches.

## Domain Context

- You will be creating C/C++ code for the ESP32S3 platform.
- It will use STS 3215 HS servos as motors for autobalancing.
- The servos are linked to a URT-1 driver board.
- The id of the servo for the Right wheel is 7.
- The id of the servo for the Left Wheel is 8.
- The RX (port U0RX-GPIO44)/TX (port U0TX-GPIO43) pins configured at 1000000 baud.
- These pins will be connected to the TX/RX pins of the URT-1 driver board configured as 3.3V level.
- The servos will be connected in series and linked to the Driver board G/V1/S pins.
- The servos will be powered with 12V directly from a LIPO 3S 12V 2200mAh battery.
- The battery is protected for undervoltage by a specific XH-M609 circuit board.
- The ESP32S3 board power supply is provided by a 12V to 5V 3A DC-DC converter.
- A main power switch and a fuse are placed between the LIPO battery and the XH-M609 undervoltage protection board.
- The ESP32S3 is connected through SPI to the BMI088 Gyro/Accelerometer board.
- The SPI pins from the ESP32S3 used to drive the BMI088 board are:
The SPI pins from the ESP32S3 used to drive the BMI088 board are:

- `GPIO4` : GPIO4 — default chip-select for the accelerometer
- `GPIO11` : FSPID (dual SPI) — MOSI
- `GPIO12` : FSPICLK — SPI clock
- `GPIO13` : FSPIQ — MISO
- `GPIO14` : default chip-select for the Gyroscope (chip select 2)

### BMI088 driver notes (runtime/debug)

- The project includes recent fixes to the BMI088 driver discovered during hardware testing:
  - SPI helper now performs a dummy transfer before reading register data to match the 3rd-party library semantics.
  - Per-chip SPI modes are used: accelerometer accesses use `SPI_MODE0`, gyroscope accesses use `SPI_MODE3`.
  - WHO_AM_I probes are emitted at startup to confirm mapping; expected IDs are `0x1E` (accel) and `0x0F` (gyro). Some modules may report an alternative gyro ID (observed `0x22`); the driver prints observed IDs to help triage wiring or module differences.

- Debug / validation steps:
  - Build & upload: `pio run --target clean; pio run --target upload`.
  - Monitor serial at `921600` baud: `pio device monitor --baud 921600`.
  - On startup the driver prints WHO_AM_I probe results and a short raw-register dump; verify the accel/gyro mapping and that gravity (~9.8 m/s²) appears on the expected axis when you tilt the robot.


  Command list:
  For ESP Project use base folder /ESP32/ folder:
- build ESP : "pio run".
- Build and Upload ESP project : "pio run --target clean; pio run --target upload"
- Upload project after build: "pio --target upload"
- Monitor device through serial: "pio device monitor"

  For ServoDriverST project use base folder /ESP32/waveshare_resources/ST_Servo/ServoDriverST/:


## Important Constraints

- The robot must self-balance using its STS 3215 HS servos as motors.

## External Dependencies

- Arduino Lib BMI08x-Arduino from Bosch Sensortec
- Official SCServo lib from Feetech
- MaixPy
- MaixCDK
