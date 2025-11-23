// motor_driver.cpp
#include "motor_driver.h"
#include "../config/motor_config.h"
#include <Arduino.h>
#if MOTOR_DRIVER_REAL
#include <SCServo.h>
#endif

namespace abbot {
namespace motor {

static bool s_motors_enabled = false;
#if MOTOR_DRIVER_REAL
static SMS_STS s_servoBus;
static bool s_servoInitialized = false;
#endif

void initMotorDriver() {
  // Ensure safe default
  Serial.println("motor_driver: init (motors disabled by default)");
#if MOTOR_DRIVER_REAL
  Serial.println("motor_driver: compiled in REAL mode (SCServo). Initializing bus (Serial1)...");
  // NOTE: Do NOT initialize Serial1 here — some devkits expose multiple USB
  // interfaces and initializing the servo UART at boot can cause a transient
  // re-enumeration or pin contention on some boards. Delay initialization
  // until motors are explicitly enabled (see enableMotors()).
  s_servoInitialized = false;
#else
  Serial.println("motor_driver: compiled in STUB mode");
#endif
}

void enableMotors() {
  if (s_motors_enabled) return;
  s_motors_enabled = true;
  Serial.println("motor_driver: motors ENABLED");

#if MOTOR_DRIVER_REAL
  // Initialize Serial1 and servo bus on-demand to avoid interfering with
  // USB enumeration at boot. This also prevents transient disappearance of
  // the native USB/CDC port on some devkits.
  if (!s_servoInitialized) {
    Serial.print("motor_driver: initializing servo UART on-demand (Serial1)\n");
    Serial1.begin(SC_SERVO_BAUD, SERIAL_8N1, SC_SERVO_RX_PIN, SC_SERVO_TX_PIN);
    s_servoBus.pSerial = &Serial1;
    s_servoBus.IOTimeOut = 200;
    delay(50);
    s_servoInitialized = true;
    // Ensure torque off before enabling explicitly
    s_servoBus.EnableTorque(LEFT_MOTOR_ID, 0);
    s_servoBus.EnableTorque(RIGHT_MOTOR_ID, 0);
  }
  // Now safely enable torque
  s_servoBus.EnableTorque(LEFT_MOTOR_ID, 1);
  s_servoBus.EnableTorque(RIGHT_MOTOR_ID, 1);
#endif
}

void disableMotors() {
  if (!s_motors_enabled) return;
  s_motors_enabled = false;
  Serial.println("motor_driver: motors DISABLED");

#if MOTOR_DRIVER_REAL
  if (s_servoInitialized) {
    s_servoBus.EnableTorque(LEFT_MOTOR_ID, 0);
    s_servoBus.EnableTorque(RIGHT_MOTOR_ID, 0);
  }
#endif
}

bool areMotorsEnabled() {
  return s_motors_enabled;
}
 
void setMotorCommand(int id, float command) {
  // clamp
  if (command > 1.0f) command = 1.0f;
  if (command < -1.0f) command = -1.0f;

  if (!s_motors_enabled) {
    Serial.print("motor_driver: setMotorCommand ignored (motors disabled) id="); Serial.print(id);
    Serial.print(" cmd="); Serial.println(command, 4);
    return;
  }

  // Apply per-motor inversion from config
  int invert = 0;
  if (id == LEFT_MOTOR_ID) invert = LEFT_MOTOR_INVERT;
  else if (id == RIGHT_MOTOR_ID) invert = RIGHT_MOTOR_INVERT;
  if (invert) command = -command;

#if MOTOR_DRIVER_REAL
  if (!s_servoInitialized) {
    Serial.println("motor_driver: REAL set called but servo bus not initialized");
    return;
  }
  // Map normalized command [-1..1] to PWM magnitude (0..800 approx)
  // Behavior varies by configured control mode
  if (MOTOR_CONTROL_MODE == MOTOR_CONTROL_POSITION) {
    // Map [-1..1] to position range around MOTOR_POSITION_CENTER
    int16_t pos = (int16_t)(MOTOR_POSITION_CENTER + command * MOTOR_POSITION_RANGE);
    // Speed parameter: scale magnitude to a reasonable speed value
    uint16_t speed = (uint16_t)(fabs(command) * 500.0f);
    int rc = s_servoBus.WritePosEx((uint8_t)id, (s16)pos, speed, 0);
    Serial.print("motor_driver: REAL set (POSITION) id="); Serial.print(id); Serial.print(" pos="); Serial.print(pos); Serial.print(" speed="); Serial.print(speed); Serial.print(" rc="); Serial.println(rc);
  } else if (MOTOR_CONTROL_MODE == MOTOR_CONTROL_VELOCITY) {
    // Velocity-like commands: use SMS_STS WriteSpe API
    // Map normalized [-1..1] to signed speed value (servo units)
    // Keep a conservative maximum magnitude to start with
    int16_t speed = (int16_t)(command * (float)SC_SERVO_MAX_SPEED);
    // Ensure the servo is in wheel/motor mode to accept speed commands.
    // Some ST-series firmwares require WheelMode() before velocity writes.
    s_servoBus.WheelMode((uint8_t)id);
    delay(5);
    int rc = s_servoBus.WriteSpe((uint8_t)id, (s16)speed, 0);
    Serial.print("motor_driver: REAL set (VELOCITY) id="); Serial.print(id); Serial.print(" speed="); Serial.print(speed); Serial.print(" rc="); Serial.println(rc);
  } else if (MOTOR_CONTROL_MODE == MOTOR_CONTROL_MOTOR) {
    // Motor (low-level) mode: switch to wheel mode (constant-speed) and send speed
    s_servoBus.WheelMode((uint8_t)id);
    int16_t pwm = (int16_t)(command * 1000.0f);
    int rc = s_servoBus.WriteSpe((uint8_t)id, (s16)pwm, 0);
    Serial.print("motor_driver: REAL set (MOTOR/WHEEL) id="); Serial.print(id); Serial.print(" pwm="); Serial.print(pwm); Serial.print(" rc="); Serial.println(rc);
  } else {
    Serial.print("motor_driver: REAL set unknown MOTOR_CONTROL_MODE="); Serial.println((int)MOTOR_CONTROL_MODE);
  }
#else
  Serial.print("motor_driver: STUB set id="); Serial.print(id); Serial.print(" cmd="); Serial.println(command,4);
#endif
}

void setMotorCommandRaw(int id, int16_t rawSpeed) {
  if (!s_motors_enabled) {
    Serial.print("motor_driver: setMotorCommandRaw ignored (motors disabled) id="); Serial.print(id);
    Serial.print(" raw="); Serial.println(rawSpeed);
    return;
  }
#if MOTOR_DRIVER_REAL
  if (!s_servoInitialized) {
    Serial.println("motor_driver: REAL raw set called but servo bus not initialized");
    return;
  }
  // Force wheel/motor mode and write raw speed
  s_servoBus.WheelMode((uint8_t)id);
  delay(5);
  int rc = s_servoBus.WriteSpe((uint8_t)id, (s16)rawSpeed, 0);
  Serial.print("motor_driver: REAL set (RAW) id="); Serial.print(id); Serial.print(" raw="); Serial.print(rawSpeed); Serial.print(" rc="); Serial.println(rc);
#else
  (void)rawSpeed;
  Serial.print("motor_driver: STUB raw set id="); Serial.print(id); Serial.print(" raw="); Serial.println(rawSpeed);
#endif
}

int32_t readEncoder(int id) {
#if MOTOR_DRIVER_REAL
  if (!s_servoInitialized) {
    Serial.print("motor_driver: REAL readEncoder but servo bus not initialized id="); Serial.println(id);
    return 0;
  }
  // Request feedback and return position
  int fb = s_servoBus.FeedBack(id);
  if (fb <= 0) {
    // feedback failed
    Serial.print("motor_driver: REAL FeedBack failed id="); Serial.println(id);
    return 0;
  }
  int pos = s_servoBus.ReadPos(-1);
  Serial.print("motor_driver: REAL encoder id="); Serial.print(id); Serial.print(" pos="); Serial.println(pos);
  return pos;
#else
  (void)id;
  return 0;
#endif
}

bool processSerialCommand(const String &line) {
  if (line.length() == 0) return false;
  String up = line;
  up.toUpperCase();
  char buf[128];
  up.toCharArray(buf, sizeof(buf));
  char *tk = strtok(buf, " \t\r\n");
  if (!tk) return false;
  if (strcmp(tk, "MOTOR") != 0) return false;
  char *cmd = strtok(NULL, " \t\r\n");
  if (!cmd) return false;

  if (strcmp(cmd, "ENABLE") == 0) {
    enableMotors();
    return true;
  }
  if (strcmp(cmd, "DISABLE") == 0) {
    disableMotors();
    return true;
  }
  if (strcmp(cmd, "STATUS") == 0) {
    Serial.print("motor_driver: status enabled="); Serial.println(areMotorsEnabled() ? "YES" : "NO");
    Serial.print("motor_driver: LEFT_ID="); Serial.print(LEFT_MOTOR_ID); Serial.print(" RIGHT_ID="); Serial.println(RIGHT_MOTOR_ID);
    Serial.print("motor_driver: LEFT_INVERT="); Serial.print(LEFT_MOTOR_INVERT); Serial.print(" RIGHT_INVERT="); Serial.println(RIGHT_MOTOR_INVERT);
    return true;
  }
  if (strcmp(cmd, "SET") == 0) {
    char *arg = strtok(NULL, " \t\r\n");
    if (!arg) return false;
    int id = -1;
    if (strcmp(arg, "LEFT") == 0) id = LEFT_MOTOR_ID;
    else if (strcmp(arg, "RIGHT") == 0) id = RIGHT_MOTOR_ID;
    else id = atoi(arg);
    char *v = strtok(NULL, " \t\r\n");
    if (!v) return false;
    // Support RAW mode: `MOTOR SET LEFT RAW 2000` to send raw servo units
    if (strcmp(v, "RAW") == 0) {
      char *rv = strtok(NULL, " \t\r\n");
      if (!rv) return false;
      int16_t rawv = (int16_t)atoi(rv);
      setMotorCommandRaw(id, rawv);
      return true;
    }
    float cmdv = atof(v);
    setMotorCommand(id, cmdv);
    return true;
  }
  if (strcmp(cmd, "READ") == 0) {
    char *arg = strtok(NULL, " \t\r\n");
    if (!arg) return false;
    int id = -1;
    if (strcmp(arg, "LEFT") == 0) id = LEFT_MOTOR_ID;
    else if (strcmp(arg, "RIGHT") == 0) id = RIGHT_MOTOR_ID;
    else id = atoi(arg);
    int32_t val = readEncoder(id);
    Serial.print("motor_driver: encoder id="); Serial.print(id); Serial.print(" val="); Serial.println(val);
    return true;
  }
  if (strcmp(cmd, "DUMP") == 0) {
    Serial.print("motor_driver: config LEFT_ID="); Serial.print(LEFT_MOTOR_ID); Serial.print(" RIGHT_ID="); Serial.println(RIGHT_MOTOR_ID);
    Serial.print("motor_driver: pins S_TXD="); Serial.print(SC_SERVO_TX_PIN); Serial.print(" S_RXD="); Serial.println(SC_SERVO_RX_PIN);
    return true;
  }

  return false;
}

} // namespace motor
} // namespace abbot
