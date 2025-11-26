// motor_driver.cpp
#include "motor_driver.h"
#include "../config/motor_config.h"
#include "logging.h"
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
// Track last normalized motor commands for tuning/telemetry
static float s_last_command_left = 0.0f;
static float s_last_command_right = 0.0f;

void initMotorDriver() {
#if MOTOR_DRIVER_REAL
  // Ensure safe default
  LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "motor_driver: init (motors disabled by default)");
  LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "motor_driver: compiled in REAL mode (SCServo). Initializing bus (Serial1)...");
  // NOTE: Do NOT initialize Serial1 here — some devkits expose multiple USB
  // interfaces and initializing the servo UART at boot can cause a transient
  // re-enumeration or pin contention on some boards. Delay initialization
  // until motors are explicitly enabled (see enableMotors()).
  s_servoInitialized = false;
#else
  LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "motor_driver: compiled in STUB mode");
#endif
}

float getLastMotorCommand(int id) {
  if (id == LEFT_MOTOR_ID) {
    return s_last_command_left;
  }
  if (id == RIGHT_MOTOR_ID) {
    return s_last_command_right;
  }
  return 0.0f;
}

void enableMotors() {
  if (s_motors_enabled) {
    return;
  }
  s_motors_enabled = true;
  LOG_PRINTLN(abbot::log::CHANNEL_MOTOR, "motor_driver: motors ENABLED");

#if MOTOR_DRIVER_REAL
  // Initialize Serial1 and servo bus on-demand to avoid interfering with
  // USB enumeration at boot. This also prevents transient disappearance of
  // the native USB/CDC port on some devkits.
  if (!s_servoInitialized) {
    LOG_PRINTLN(abbot::log::CHANNEL_MOTOR, "motor_driver: initializing servo UART on-demand (Serial1)");
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
  if (!s_motors_enabled) {
    return;
  }
  s_motors_enabled = false;
  LOG_PRINTLN(abbot::log::CHANNEL_MOTOR, "motor_driver: motors DISABLED");

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
  if (command > 1.0f) {
    command = 1.0f;
  }
  if (command < -1.0f) {
    command = -1.0f;
  }

  if (!s_motors_enabled) {
    LOG_PRINTF(abbot::log::CHANNEL_MOTOR, "motor_driver: setMotorCommand ignored (motors disabled) id=%d cmd=%.4f\n", id, command);
    return;
  }

  // Apply per-motor inversion from config
  int invert = 0;
  if (id == LEFT_MOTOR_ID) {
    invert = LEFT_MOTOR_INVERT;
  } else if (id == RIGHT_MOTOR_ID) {
    invert = RIGHT_MOTOR_INVERT;
  }
  if (invert) {
    command = -command;
  }

  // Record last command for telemetry/tuning
  if (id == LEFT_MOTOR_ID) {
    s_last_command_left = command;
  } else if (id == RIGHT_MOTOR_ID) {
    s_last_command_right = command;
  }
#if MOTOR_DRIVER_REAL
  if (!s_servoInitialized) {
    LOG_PRINTLN(abbot::log::CHANNEL_MOTOR, "motor_driver: REAL set called but servo bus not initialized");
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
    LOG_PRINTF(abbot::log::CHANNEL_MOTOR, "motor_driver: REAL set (POSITION) id=%d pos=%d speed=%u rc=%d\n", id, pos, speed, rc);
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
    LOG_PRINTF(abbot::log::CHANNEL_MOTOR, "motor_driver: REAL set (VELOCITY) id=%d speed=%d rc=%d\n", id, speed, rc);
  } else if (MOTOR_CONTROL_MODE == MOTOR_CONTROL_MOTOR) {
    // Motor (low-level) mode: switch to wheel mode (constant-speed) and send speed
    s_servoBus.WheelMode((uint8_t)id);
    int16_t pwm = (int16_t)(command * 1000.0f);
    int rc = s_servoBus.WriteSpe((uint8_t)id, (s16)pwm, 0);
    LOG_PRINTF(abbot::log::CHANNEL_MOTOR, "motor_driver: REAL set (MOTOR/WHEEL) id=%d pwm=%d rc=%d\n", id, pwm, rc);
  } else {
    LOG_PRINTF(abbot::log::CHANNEL_MOTOR, "motor_driver: REAL set unknown MOTOR_CONTROL_MODE=%d\n", (int)MOTOR_CONTROL_MODE);
  }
#else
  LOG_PRINTF(abbot::log::CHANNEL_MOTOR, "motor_driver: STUB set id=%d cmd=%.4f\n", id, command);
#endif
}

void setMotorCommandRaw(int id, int16_t rawSpeed) {
  if (!s_motors_enabled) {
    LOG_PRINTF(abbot::log::CHANNEL_MOTOR, "motor_driver: setMotorCommandRaw ignored (motors disabled) id=%d raw=%d\n", id, rawSpeed);
    return;
  }
#if MOTOR_DRIVER_REAL
  if (!s_servoInitialized) {
    LOG_PRINTLN(abbot::log::CHANNEL_MOTOR, "motor_driver: REAL raw set called but servo bus not initialized");
    return;
  }
  // Force wheel/motor mode and write raw speed
  s_servoBus.WheelMode((uint8_t)id);
  delay(5);
  int rc = s_servoBus.WriteSpe((uint8_t)id, (s16)rawSpeed, 0);
  LOG_PRINTF(abbot::log::CHANNEL_MOTOR, "motor_driver: REAL set (RAW) id=%d raw=%d rc=%d\n", id, rawSpeed, rc);
#else
  (void)rawSpeed;
  LOG_PRINTF(abbot::log::CHANNEL_MOTOR, "motor_driver: STUB raw set id=%d raw=%d\n", id, rawSpeed);
#endif
}

int32_t readEncoder(int id) {
#if MOTOR_DRIVER_REAL
  if (!s_servoInitialized) {
    LOG_PRINTF(abbot::log::CHANNEL_MOTOR, "motor_driver: REAL readEncoder but servo bus not initialized id=%d\n", id);
    return 0;
  }
  // Request feedback and return position
  int fb = s_servoBus.FeedBack(id);
  if (fb <= 0) {
    // feedback failed
    LOG_PRINTF(abbot::log::CHANNEL_MOTOR, "motor_driver: REAL FeedBack failed id=%d\n", id);
    return 0;
  }
  int pos = s_servoBus.ReadPos(-1);
  LOG_PRINTF(abbot::log::CHANNEL_MOTOR, "motor_driver: REAL encoder id=%d pos=%d\n", id, pos);
  return pos;
#else
  (void)id;
  return 0;
#endif
}

bool processSerialCommand(const String &line) {
  if (line.length() == 0) {
    return false;
  }
  String up = line;
  up.toUpperCase();
  char buf[128];
  up.toCharArray(buf, sizeof(buf));
  char *tk = strtok(buf, " \t\r\n");
  if (!tk) {
    return false;
  }
  if (strcmp(tk, "MOTOR") != 0) {
    return false;
  }
  char *cmd = strtok(NULL, " \t\r\n");
  if (!cmd) {
    return false;
  }

  if (strcmp(cmd, "ENABLE") == 0) {
    LOG_PRINT(abbot::log::CHANNEL_MOTOR, "motor_driver: status enabled="); LOG_PRINTLN(abbot::log::CHANNEL_MOTOR, areMotorsEnabled() ? "YES" : "NO");
    LOG_PRINTF(abbot::log::CHANNEL_MOTOR, "motor_driver: LEFT_ID=%d RIGHT_ID=%d\n", LEFT_MOTOR_ID, RIGHT_MOTOR_ID);
  }
  if (strcmp(cmd, "DISABLE") == 0) {
    disableMotors();
    return true;
  }
  if (strcmp(cmd, "STATUS") == 0) {
    LOG_PRINT(abbot::log::CHANNEL_MOTOR, "motor_driver: status enabled="); LOG_PRINTLN(abbot::log::CHANNEL_MOTOR, areMotorsEnabled() ? "YES" : "NO");
    LOG_PRINTF(abbot::log::CHANNEL_MOTOR, "motor_driver: LEFT_ID=%d RIGHT_ID=%d\n", LEFT_MOTOR_ID, RIGHT_MOTOR_ID);
    LOG_PRINTF(abbot::log::CHANNEL_MOTOR, "motor_driver: LEFT_INVERT=%d RIGHT_INVERT=%d\n", LEFT_MOTOR_INVERT, RIGHT_MOTOR_INVERT);
    return true;
  }
  if (strcmp(cmd, "SET") == 0) {
    char *arg = strtok(NULL, " \t\r\n");
    if (!arg) {
      return false;
    }
    int id = -1;
    if (strcmp(arg, "LEFT") == 0) id = LEFT_MOTOR_ID;
    else if (strcmp(arg, "RIGHT") == 0) id = RIGHT_MOTOR_ID;
    else id = atoi(arg);
    char *v = strtok(NULL, " \t\r\n");
    if (!v) {
      return false;
    }
    // Support RAW mode: `MOTOR SET LEFT RAW 2000` to send raw servo units
    if (strcmp(v, "RAW") == 0) {
      char *rv = strtok(NULL, " \t\r\n");
      if (!rv) {
        return false;
      }
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
    if (!arg) {
      return false;
    }
    int id = -1;
    if (strcmp(arg, "LEFT") == 0) id = LEFT_MOTOR_ID;
    else if (strcmp(arg, "RIGHT") == 0) id = RIGHT_MOTOR_ID;
    else id = atoi(arg);
    int32_t val = readEncoder(id);
    LOG_PRINTF(abbot::log::CHANNEL_MOTOR, "motor_driver: encoder id=%d val=%ld\n", id, val);
    return true;
  }
  if (strcmp(cmd, "PARAMS") == 0) {
    char *arg = strtok(NULL, " \t\r\n");
    if (!arg) {
      return false;
    }
    int id = -1;
    if (strcmp(arg, "LEFT") == 0) id = LEFT_MOTOR_ID;
    else if (strcmp(arg, "RIGHT") == 0) id = RIGHT_MOTOR_ID;
    else id = atoi(arg);
#if MOTOR_DRIVER_REAL
    if (!s_servoInitialized) {
      LOG_PRINTLN(abbot::log::CHANNEL_MOTOR, "motor_driver: PARAMS requested but servo bus not initialized");
      return true;
    }
    LOG_PRINTF(abbot::log::CHANNEL_MOTOR, "motor_driver: params id=%d\n", id);
    // Read a selection of EEPROM/SRAM registers defined by SMS_STS
    int model = s_servoBus.readWord(id, SMS_STS_MODEL_L);
    LOG_PRINTF(abbot::log::CHANNEL_MOTOR, "  model=0x%X\n", model);
    int vid = s_servoBus.readByte(id, SMS_STS_ID);
    LOG_PRINTF(abbot::log::CHANNEL_MOTOR, "  id=%d\n", vid);
    int min_angle = s_servoBus.readWord(id, SMS_STS_MIN_ANGLE_LIMIT_L);
    int max_angle = s_servoBus.readWord(id, SMS_STS_MAX_ANGLE_LIMIT_L);
    LOG_PRINTF(abbot::log::CHANNEL_MOTOR, "  min_angle=%d max_angle=%d\n", min_angle, max_angle);
    int cw_dead = s_servoBus.readByte(id, SMS_STS_CW_DEAD);
    int ccw_dead = s_servoBus.readByte(id, SMS_STS_CCW_DEAD);
    LOG_PRINTF(abbot::log::CHANNEL_MOTOR, "  cw_dead=%d ccw_dead=%d\n", cw_dead, ccw_dead);
    int ofs = s_servoBus.readWord(id, SMS_STS_OFS_L);
    LOG_PRINTF(abbot::log::CHANNEL_MOTOR, "  offset=%d\n", ofs);
    int mode = s_servoBus.readByte(id, SMS_STS_MODE);
    LOG_PRINTF(abbot::log::CHANNEL_MOTOR, "  mode=%d\n", mode);
    int torque_en = s_servoBus.readByte(id, SMS_STS_TORQUE_ENABLE);
    LOG_PRINTF(abbot::log::CHANNEL_MOTOR, "  torque_enable=%d\n", torque_en);
    int acc = s_servoBus.readByte(id, SMS_STS_ACC);
    LOG_PRINTF(abbot::log::CHANNEL_MOTOR, "  acc=%d\n", acc);
    int torque_limit = s_servoBus.readWord(id, SMS_STS_TORQUE_LIMIT_L);
    LOG_PRINTF(abbot::log::CHANNEL_MOTOR, "  torque_limit=%d\n", torque_limit);
    int lock = s_servoBus.readByte(id, SMS_STS_LOCK);
    LOG_PRINTF(abbot::log::CHANNEL_MOTOR, "  lock=%d\n", lock);
    // Present status
    int pos = s_servoBus.readWord(id, SMS_STS_PRESENT_POSITION_L);
    int speed = s_servoBus.readWord(id, SMS_STS_PRESENT_SPEED_L);
    int load = s_servoBus.readWord(id, SMS_STS_PRESENT_LOAD_L);
    int volt = s_servoBus.readByte(id, SMS_STS_PRESENT_VOLTAGE);
    int temp = s_servoBus.readByte(id, SMS_STS_PRESENT_TEMPERATURE);
    int moving = s_servoBus.readByte(id, SMS_STS_MOVING);
    int current = s_servoBus.readWord(id, SMS_STS_PRESENT_CURRENT_L);
    LOG_PRINTF(abbot::log::CHANNEL_MOTOR, "  present_pos=%d speed=%d load=%d volt=%d temp=%d moving=%d current=%d\n", pos, speed, load, volt, temp, moving, current);
    return true;
#else
    LOG_PRINTLN(abbot::log::CHANNEL_MOTOR, "motor_driver: PARAMS only supported in real mode");
    return true;
#endif
  }
  if (strcmp(cmd, "DUMP") == 0) {
    LOG_PRINTF(abbot::log::CHANNEL_MOTOR, "motor_driver: config LEFT_ID=%d RIGHT_ID=%d\n", LEFT_MOTOR_ID, RIGHT_MOTOR_ID);
    LOG_PRINTF(abbot::log::CHANNEL_MOTOR, "motor_driver: pins S_TXD=%d S_RXD=%d\n", SC_SERVO_TX_PIN, SC_SERVO_RX_PIN);
    return true;
  }

  return false;
}

} // namespace motor
} // namespace abbot
