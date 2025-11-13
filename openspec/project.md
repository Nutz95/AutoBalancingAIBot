# Project Context

## Purpose
An autobalancing AI bot with 2 servo motors (STS 3215 HS from Feetech), using ESP32-S3 as controler and connected to a BMI088 gyroscope through SPI.
The bot has 2 22mm wheels on each sides,a platform with all the electronics on top and a camera on a small mast.

## Tech Stack
- ESP32-S3 code using PlatformIO. The esp will be dedicated in balancing the robot and controling the motion of the robot (Left/Right/Forward/Backward)
- Maixpy/MaixCDK to control the Maixcam Camera with Yolo model. It will be connected to the ESP32S3 through RX/TX and will tell the ESP32 where to go (Go forward/Backward a certain amount (cm as int), turn right/left (a certain angle in decimal)).
- HTML/CSS to provide a simple controling interface on the ESP32S3 for basic motions (forward/backward, Turn Left/Right with angles)
- The ESP32S3 code will be event driven as much as possible.
- The Feetech STS 3215-HS servos are using RX/TX protocol and are linked in series

## Project Conventions

### Code Style
- Use C++ for the ESP32 code
- Use MaixPy Micropython for Maixcam codeS
- For the Arduino Code for the ESP32, you will create a class that will serve as a manager for each device/functionalities (Servos, BMI088, HTML Server, AutoBalancer (PID, KalmanFilter, etc...), Configuration)
- Follow SOLID principle as much as possible
- Avoid duplicated codes
- Create clear class and functions names that makes sens on what the class/functions are doing. You can use long names if you see fit.
- Create a Log class to handle Info/Debug/Error log to Serial output (ESP32S3 WROOM Dev kit has a special USB Serial Output for that purpose. Use it.).

### Architecture Patterns
- The code for the ESP2S3 will be located into /ESP32 sub folders.
- The code for the Maixcam Micropython will be located into /Maixcam subfolder.

- The start of the programm will instantiate all components/needed classes/setup constants/setup basic html server.
- The ESP32S3 will have a main loop.
- A timer will be confirgured to be triggered at 200Hz. A function will be called to querry the Gyroscope/accelerometer and update it's dedicated class/component (BMI088 class)
- A timer that will call a function to autobalance the robot. It needs to be adjusted according to the balancing needs.
- SOme mechanisme to handle the HTML server connections/commands
- A event basemecanisme that will handle the RX/TX commands from the Maixcam.
- On BMI088/AutoBalancer/ServoDriver classes, you will add logs that i can reuse on the tests. (I might need the gyro/accelerometer data, the ServoDriver commands sent for debug, the autobalancer inputs and outputs for debug purpose.)
- Add a constant DEBUG in order to trigger Debug logs.
- You will use the 3rd party arduino lib to control the BMI088 (BMI08x-Arduino from bosch Sensortec)
- You will use the official SCServo lib from Feetch to drive the STS 3215 HS servos.

### Testing Strategy
-Create a test for at least the Balancing PID algorithm.

### Git Workflow
No need for complicated workflow, simple commits will do fine. No need for branches.

## Domain Context
- You will be creating C/C++ code for the ESP32S3 platform.
- It will use STS 3215 HS servos as motors for autobalancing.
- The servos are linked to a URT-1 driver boards
- The id of the servo for the Right wheel is 7
- The id of the servo for the Left Wheel is 8
- The RX (port U0RX-GPIO44)/TX (port U0TX-GPIO43) pins configures as 1000000 bauds
- These pins will be connected to the TX/RX pins of the URT-1 driver board configured as 3.3V level.
- The servos will be connected in series and linked to the Driver board G/V1/S pins.
- The servos will be provided with 12V directly from a LIPO 3S 12V 2200mAh battery.
- The battery is protected for undervoltage by a specific XH-M609 circuit board.
- The ESP32S3 board power supply is provided by a 12V to 5V3A DC-DC converter.
- A main power switch and a fuse are placed between the LIPO battery and the XH-M609 undervoltage protection board.
- The ESP32S3 is connected through SPI to the BMI088 Gyro/Accelerometer board.
- The SPI pins from the ESP32S3 used to drive the BMI088 board are:
GPIO10 : FSPICS0 (chip select),
GPIO11 : FSPID (dual SPI) (MOSI),
GPIO12 : FSPICLK (clock),
GPIO13 : FSPIQ (quad SPI)(MISO),
- To build the ESP project you will need to use the command "pio run" from within the /ESP/ subFolder


## Important Constraints
- The robot must self balance using it's STS 3215 HS servos as motor.

## External Dependencies
- Arduino Lib BMI08x-Arduino from bosch Sensortec
- Official SCServo lib from Feetch
- MaixPy
- MaixCDK
