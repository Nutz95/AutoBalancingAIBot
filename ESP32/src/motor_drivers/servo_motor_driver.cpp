// servo_motor_driver.cpp
// Velocity Closed-Loop Mode: ESP-side position control with encoder unwrapping
// Servos run in Wheel mode, position is tracked as int64_t (unlimited range)

#include "motor_drivers/servo_motor_driver.h"
#include "../config/motor_configs/servo_motor_config.h"
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

// =============================================================================
// VELOCITY CLOSED-LOOP: Position tracking with encoder unwrap
// =============================================================================
// Accumulated positions (int64_t = virtually unlimited)
static int64_t s_accumulated_pos_left = 0;
static int64_t s_accumulated_pos_right = 0;

// Target positions (where we want to be)
static int64_t s_target_pos_left = 0;
static int64_t s_target_pos_right = 0;

// Last raw encoder readings (0-4095) for unwrap detection
static int16_t s_last_raw_pos_left = 0;
static int16_t s_last_raw_pos_right = 0;

// Flag to indicate if we've read initial encoder positions
static bool s_encoder_initialized = false;

// Update accumulated position with unwrap detection
// Returns true if successful, false if feedback failed
static bool updateEncoderPosition(int id, int16_t raw_pos) {
  int16_t *last_raw;
  int64_t *accumulated;

  if (id == LEFT_MOTOR_ID) {
    last_raw = &s_last_raw_pos_left;
    accumulated = &s_accumulated_pos_left;
  } else if (id == RIGHT_MOTOR_ID) {
    last_raw = &s_last_raw_pos_right;
    accumulated = &s_accumulated_pos_right;
  } else {
    return false;
  }

  // Calculate delta with unwrap
  int16_t delta = raw_pos - *last_raw;

  // Unwrap: if |delta| > 2048, we crossed the 0/4095 boundary
  if (delta > 2048) {
    delta -= SERVO_ENCODER_RESOLUTION; // Wrapped backwards (4095 -> 0)
  } else if (delta < -2048) {
    delta += SERVO_ENCODER_RESOLUTION; // Wrapped forwards (0 -> 4095)
  }

  *accumulated += delta;
  *last_raw = raw_pos;

  return true;
}

// Read encoder and update accumulated position
static bool readAndUpdateEncoder(int id) {
#if MOTOR_DRIVER_REAL
  if (!s_servoInitialized)
    return false;

  int fb = s_servoBus.FeedBack(id);
  if (fb <= 0) {
    LOG_PRINTF(abbot::log::CHANNEL_MOTOR,
               "motor_driver: FeedBack failed id=%d fb=%d\n", id, fb);
    return false;
  }

  int raw_pos = s_servoBus.ReadPos(-1);
  if (raw_pos < 0) {
    LOG_PRINTF(abbot::log::CHANNEL_MOTOR,
               "motor_driver: ReadPos failed id=%d\n", id);
    return false;
  }

  // Mask to 0-4095 range (in wheel mode, position is single-turn)
  raw_pos = raw_pos & 0x0FFF;

  return updateEncoderPosition(id, (int16_t)raw_pos);
#else
  return false;
#endif
}

void initMotorDriver() {
#if MOTOR_DRIVER_REAL
  LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT,
              "motor_driver: init (motors disabled by default)");
  LOG_PRINTLN(
      abbot::log::CHANNEL_DEFAULT,
      "motor_driver: VELOCITY CLOSED-LOOP mode (ESP-side position control)");
  s_servoInitialized = false;
  s_encoder_initialized = false;

#if MOTOR_SERVO_RESET_ON_BOOT
  LOG_PRINTLN(abbot::log::CHANNEL_MOTOR,
              "motor_driver: performing servo bus soft-reset");
  Serial1.begin(SC_SERVO_BAUD, SERIAL_8N1, SC_SERVO_RX_PIN, SC_SERVO_TX_PIN);
  s_servoBus.pSerial = &Serial1;
  s_servoBus.IOTimeOut = 200;
  delay(30);
  while (Serial1.available()) {
    (void)Serial1.read();
  }
  // Broadcast torque disable
  s_servoBus.EnableTorque(0xFE, 0);
  delay(20);
  Serial1.end();

  // Clear command/target state after reset
  s_last_command_left = 0.0f;
  s_last_command_right = 0.0f;
  s_target_pos_left = s_accumulated_pos_left;
  s_target_pos_right = s_accumulated_pos_right;
  LOG_PRINTLN(abbot::log::CHANNEL_MOTOR, "motor_driver: command state cleared");
#endif
#else
  LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT,
              "motor_driver: compiled in STUB mode");
#endif
}

void clearCommandState() {
  // Reset tracked logical commands
  s_last_command_left = 0.0f;
  s_last_command_right = 0.0f;
  // Reset targets to current accumulated positions to avoid latent motion on
  // re-enable
  s_target_pos_left = s_accumulated_pos_left;
  s_target_pos_right = s_accumulated_pos_right;

#if MOTOR_DRIVER_REAL
  // If the servo bus is initialized, push an explicit zero-speed command to
  // both motors to clear any residual velocity register contents, even if
  // torque is off.
  if (s_servoInitialized) {
    // Send zero twice to maximize chance the bus applies it even if torque is
    // off
    s_servoBus.WriteSpe(LEFT_MOTOR_ID, 0, 0);
    delayMicroseconds(MOTOR_INTER_COMMAND_DELAY_US);
    s_servoBus.WriteSpe(RIGHT_MOTOR_ID, 0, 0);
    delayMicroseconds(MOTOR_INTER_COMMAND_DELAY_US);
    s_servoBus.WriteSpe(LEFT_MOTOR_ID, 0, 0);
    delayMicroseconds(MOTOR_INTER_COMMAND_DELAY_US);
    s_servoBus.WriteSpe(RIGHT_MOTOR_ID, 0, 0);
  }
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
  // Ensure any prior command state is cleared before re-enable
  clearCommandState();
  s_motors_enabled = true;
  LOG_PRINTLN(abbot::log::CHANNEL_MOTOR, "motor_driver: motors ENABLED");

#if MOTOR_DRIVER_REAL
  if (!s_servoInitialized) {
    LOG_PRINTLN(abbot::log::CHANNEL_MOTOR,
                "motor_driver: initializing servo UART (Serial1)");
    Serial1.begin(SC_SERVO_BAUD, SERIAL_8N1, SC_SERVO_RX_PIN, SC_SERVO_TX_PIN);
    s_servoBus.pSerial = &Serial1;
    s_servoBus.IOTimeOut = 200;
    delay(50);
    s_servoInitialized = true;

    // Ensure torque off before switching mode
    s_servoBus.EnableTorque(LEFT_MOTOR_ID, 0);
    s_servoBus.EnableTorque(RIGHT_MOTOR_ID, 0);
    delay(10);

    // Switch both servos to Wheel mode (velocity control)
    s_servoBus.WheelMode(LEFT_MOTOR_ID);
    delay(10);
    s_servoBus.WheelMode(RIGHT_MOTOR_ID);
    delay(10);

    // Set acceleration to MAX (0) for immediate response
    // This is critical for balancing to avoid phase lag
    s_servoBus.writeByte(LEFT_MOTOR_ID, SMS_STS_ACC, 0);
    delay(10);
    s_servoBus.writeByte(RIGHT_MOTOR_ID, SMS_STS_ACC, 0);
    delay(10);

    LOG_PRINTLN(abbot::log::CHANNEL_MOTOR,
                "motor_driver: servos set to Wheel mode with MAX acceleration");
  }

  // Read initial encoder positions
  if (!s_encoder_initialized) {
    int fb_l = s_servoBus.FeedBack(LEFT_MOTOR_ID);
    if (fb_l > 0) {
      int pos_l = s_servoBus.ReadPos(-1) & 0x0FFF;
      s_last_raw_pos_left = pos_l;
      s_accumulated_pos_left = 0; // Start at zero
      s_target_pos_left = 0;
    } else {
      LOG_PRINTF(abbot::log::CHANNEL_MOTOR,
                 "motor_driver: WARNING FeedBack LEFT failed fb=%d\n", fb_l);
    }

    int fb_r = s_servoBus.FeedBack(RIGHT_MOTOR_ID);
    if (fb_r > 0) {
      int pos_r = s_servoBus.ReadPos(-1) & 0x0FFF;
      s_last_raw_pos_right = pos_r;
      s_accumulated_pos_right = 0; // Start at zero
      s_target_pos_right = 0;
    } else {
      LOG_PRINTF(abbot::log::CHANNEL_MOTOR,
                 "motor_driver: WARNING FeedBack RIGHT failed fb=%d\n", fb_r);
    }

    s_encoder_initialized = true;
    LOG_PRINTF(abbot::log::CHANNEL_MOTOR,
               "motor_driver: encoder init raw L=%d R=%d\n",
               s_last_raw_pos_left, s_last_raw_pos_right);
  }

  // Enable torque
  s_servoBus.EnableTorque(LEFT_MOTOR_ID, 1);
  s_servoBus.EnableTorque(RIGHT_MOTOR_ID, 1);

  // Send zero velocity to start stationary
  s_servoBus.WriteSpe(LEFT_MOTOR_ID, 0, 0);
  delayMicroseconds(MOTOR_INTER_COMMAND_DELAY_US);
  s_servoBus.WriteSpe(RIGHT_MOTOR_ID, 0, 0);

  // Log status
  int mode_l = s_servoBus.readByte(LEFT_MOTOR_ID, SMS_STS_MODE);
  int torque_l = s_servoBus.readByte(LEFT_MOTOR_ID, SMS_STS_TORQUE_ENABLE);
  int mode_r = s_servoBus.readByte(RIGHT_MOTOR_ID, SMS_STS_MODE);
  int torque_r = s_servoBus.readByte(RIGHT_MOTOR_ID, SMS_STS_TORQUE_ENABLE);
  LOG_PRINTF(abbot::log::CHANNEL_MOTOR,
             "motor_driver: LEFT mode=%d torque=%d | RIGHT mode=%d torque=%d\n",
             mode_l, torque_l, mode_r, torque_r);
#endif
}

void disableMotors() {
  // Always clear command state and send explicit zeros to avoid residual speed
  clearCommandState();

  if (!s_motors_enabled) {
    return;
  }
  s_motors_enabled = false;
  LOG_PRINTLN(abbot::log::CHANNEL_MOTOR, "motor_driver: motors DISABLED");

#if MOTOR_DRIVER_REAL
  if (s_servoInitialized) {
    // Stop motion first
    s_servoBus.WriteSpe(LEFT_MOTOR_ID, 0, 0);
    delayMicroseconds(MOTOR_INTER_COMMAND_DELAY_US);
    s_servoBus.WriteSpe(RIGHT_MOTOR_ID, 0, 0);
    delayMicroseconds(MOTOR_INTER_COMMAND_DELAY_US);
    // Second zero just before torque-off in case first was ignored
    s_servoBus.WriteSpe(LEFT_MOTOR_ID, 0, 0);
    delayMicroseconds(MOTOR_INTER_COMMAND_DELAY_US);
    s_servoBus.WriteSpe(RIGHT_MOTOR_ID, 0, 0);
    delay(10);
    // Disable torque
    s_servoBus.EnableTorque(LEFT_MOTOR_ID, 0);
    s_servoBus.EnableTorque(RIGHT_MOTOR_ID, 0);
  }
#endif
}

bool areMotorsEnabled() {
  return s_motors_enabled;
}

void printStatus() {
  LOG_PRINT(abbot::log::CHANNEL_MOTOR, "motor_driver: status enabled=");
  LOG_PRINTLN(abbot::log::CHANNEL_MOTOR, areMotorsEnabled() ? "YES" : "NO");
  LOG_PRINTF(abbot::log::CHANNEL_MOTOR,
             "motor_driver: LEFT_ID=%d RIGHT_ID=%d\n", LEFT_MOTOR_ID,
             RIGHT_MOTOR_ID);
  LOG_PRINTF(abbot::log::CHANNEL_MOTOR,
             "motor_driver: LEFT_INVERT=%d RIGHT_INVERT=%d\n",
             LEFT_MOTOR_INVERT, RIGHT_MOTOR_INVERT);
}

void dumpConfig() {
  LOG_PRINTF(abbot::log::CHANNEL_MOTOR,
             "motor_driver: config LEFT_ID=%d RIGHT_ID=%d\n", LEFT_MOTOR_ID,
             RIGHT_MOTOR_ID);
  LOG_PRINTF(abbot::log::CHANNEL_MOTOR,
             "motor_driver: pins S_TXD=%d S_RXD=%d\n", SC_SERVO_TX_PIN,
             SC_SERVO_RX_PIN);
  LOG_PRINTF(abbot::log::CHANNEL_MOTOR,
             "motor_driver: VELOCITY_POSITION_KP=%.2f INCREMENT_SCALE=%.1f\n",
             (double)VELOCITY_POSITION_KP,
             (double)VELOCITY_TARGET_INCREMENT_SCALE);
}

// =============================================================================
// DIRECT VELOCITY CONTROL
// =============================================================================
// Each cycle:
// 1. Convert normalized command [-1..1] to speed units
// 2. Send speed command directly to servo (bypassing ESP-side position loop)
// 3. Read encoders for telemetry only (not for control)

void setMotorCommandBoth(float left_command, float right_command) {
  // Clamp individually
  if (left_command > 1.0f)
    left_command = 1.0f;
  if (left_command < -1.0f)
    left_command = -1.0f;
  if (right_command > 1.0f)
    right_command = 1.0f;
  if (right_command < -1.0f)
    right_command = -1.0f;

  // Record last commands for telemetry (BEFORE inversion - logical commands)
  s_last_command_left = left_command;
  s_last_command_right = right_command;

  if (!s_motors_enabled) {
    LOG_PRINTF(abbot::log::CHANNEL_MOTOR,
               "motor_driver: setMotorCommandBoth ignored (motors disabled) "
               "L=%.4f R=%.4f\n",
               left_command, right_command);
    return;
  }

  // Apply inversion per motor (for physical motor direction)
  float left_cmd = LEFT_MOTOR_INVERT ? -left_command : left_command;
  float right_cmd = RIGHT_MOTOR_INVERT ? -right_command : right_command;

#if MOTOR_DRIVER_REAL
  if (!s_servoInitialized) {
    LOG_PRINTLN(abbot::log::CHANNEL_MOTOR,
                "motor_driver: setMotorCommandBoth called but servo bus not "
                "initialized");
    return;
  }

  // Convert to raw speed units
  int16_t vel_left = (int16_t)(left_cmd * VELOCITY_MAX_SPEED);
  int16_t vel_right = (int16_t)(right_cmd * VELOCITY_MAX_SPEED);

  // Use standard WriteSpe to ensure compatibility and correct register usage
  // (Reverted optimization to rule out register addressing issues)
  s_servoBus.WriteSpe(LEFT_MOTOR_ID, vel_left, 0);
  // Small delay to prevent bus collision
  delayMicroseconds(MOTOR_INTER_COMMAND_DELAY_US);
  s_servoBus.WriteSpe(RIGHT_MOTOR_ID, vel_right, 0);

  // Read encoders for telemetry (optional, can be skipped if too slow)
  // We do this AFTER writing to ensure minimum latency for control
  readAndUpdateEncoder(LEFT_MOTOR_ID);
  readAndUpdateEncoder(RIGHT_MOTOR_ID);
#endif
}

void setMotorCommand(int id, float command) {
  // For single motor commands, call the dual version with zero for the other
  if (id == LEFT_MOTOR_ID) {
    setMotorCommandBoth(command, 0.0f);
  } else if (id == RIGHT_MOTOR_ID) {
    setMotorCommandBoth(0.0f, command);
  }
}

void setMotorCommandRaw(int id, int16_t rawSpeed) {
  if (!s_motors_enabled) {
    LOG_PRINTF(abbot::log::CHANNEL_MOTOR,
               "motor_driver: setMotorCommandRaw ignored (motors disabled) "
               "id=%d raw=%d\n",
               id, rawSpeed);
    return;
  }
#if MOTOR_DRIVER_REAL
  if (!s_servoInitialized) {
    LOG_PRINTLN(
        abbot::log::CHANNEL_MOTOR,
        "motor_driver: REAL raw set called but servo bus not initialized");
    return;
  }
  // Send raw velocity command directly (respecting inversion)
  int16_t speed = rawSpeed;
  if (id == LEFT_MOTOR_ID && LEFT_MOTOR_INVERT) {
    speed = -speed;
  } else if (id == RIGHT_MOTOR_ID && RIGHT_MOTOR_INVERT) {
    speed = -speed;
  }
  int rc = s_servoBus.WriteSpe((uint8_t)id, (s16)speed, 0);
  LOG_PRINTF(abbot::log::CHANNEL_MOTOR,
             "motor_driver: REAL set (RAW) id=%d raw=%d (sent=%d) rc=%d\n", id, rawSpeed,
             speed, rc);
#else
  (void)rawSpeed;
  LOG_PRINTF(abbot::log::CHANNEL_MOTOR,
             "motor_driver: STUB raw set id=%d raw=%d\n", id, rawSpeed);
#endif
}

int32_t readEncoder(int id) {
#if MOTOR_DRIVER_REAL
  if (!s_servoInitialized) {
    LOG_PRINTF(
        abbot::log::CHANNEL_MOTOR,
        "motor_driver: REAL readEncoder but servo bus not initialized id=%d\n",
        id);
    return 0;
  }
  // Return accumulated position (with unwrap)
  int32_t logicalPosition = 0;
  if (id == LEFT_MOTOR_ID) {
    logicalPosition = (int32_t)s_accumulated_pos_left;
    if (LEFT_MOTOR_INVERT) {
      logicalPosition = -logicalPosition;
    }
  } else if (id == RIGHT_MOTOR_ID) {
    logicalPosition = (int32_t)s_accumulated_pos_right;
    if (RIGHT_MOTOR_INVERT) {
      logicalPosition = -logicalPosition;
    }
  }
  return logicalPosition;
#else
  (void)id;
  return 0;
#endif
}

// Get accumulated position (full 64-bit range)
int64_t getAccumulatedPosition(int id) {
  if (id == LEFT_MOTOR_ID) {
    return s_accumulated_pos_left;
  } else if (id == RIGHT_MOTOR_ID) {
    return s_accumulated_pos_right;
  }
  return 0;
}

// Get target position
int64_t getTargetPosition(int id) {
  if (id == LEFT_MOTOR_ID) {
    return s_target_pos_left;
  } else if (id == RIGHT_MOTOR_ID) {
    return s_target_pos_right;
  }
  return 0;
}

// Reset position tracking (sets accumulated = target = 0)
void resetPositionTracking() {
  s_accumulated_pos_left = 0;
  s_accumulated_pos_right = 0;
  s_target_pos_left = 0;
  s_target_pos_right = 0;
  LOG_PRINTLN(abbot::log::CHANNEL_MOTOR,
              "motor_driver: position tracking RESET to zero");
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
    // Enable motors on request
    enableMotors();
    printStatus();
    return true;
  }
  if (strcmp(cmd, "DISABLE") == 0) {
    disableMotors();
    return true;
  }
  if (strcmp(cmd, "STATUS") == 0) {
    printStatus();
    return true;
  }
  if (strcmp(cmd, "SET") == 0) {
    char *arg = strtok(NULL, " \t\r\n");
    if (!arg) {
      return false;
    }
    int id = -1;
    if (strcmp(arg, "LEFT") == 0)
      id = LEFT_MOTOR_ID;
    else if (strcmp(arg, "RIGHT") == 0)
      id = RIGHT_MOTOR_ID;
    else
      id = atoi(arg);
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
    // Accept user-friendly percent-style values: if the supplied value is
    // outside [-1..1] treat it as percentage (e.g. 10 => 0.10, 100 => 1.0).
    if (cmdv > 1.0f || cmdv < -1.0f) {
      float orig = cmdv;
      cmdv = cmdv / 100.0f;
      LOG_PRINTF(abbot::log::CHANNEL_MOTOR,
                 "motor_driver: interpreted provided value %.3f as percent -> "
                 "%.3f normalized\n",
                 orig, cmdv);
    }
    setMotorCommand(id, cmdv);
    return true;
  }
  if (strcmp(cmd, "READ") == 0) {
    char *arg = strtok(NULL, " \t\r\n");
    if (!arg) {
      return false;
    }
    int id = -1;
    if (strcmp(arg, "LEFT") == 0)
      id = LEFT_MOTOR_ID;
    else if (strcmp(arg, "RIGHT") == 0)
      id = RIGHT_MOTOR_ID;
    else
      id = atoi(arg);
    int32_t val = readEncoder(id);
    LOG_PRINTF(abbot::log::CHANNEL_MOTOR,
               "motor_driver: encoder id=%d val=%ld\n", id, val);
    return true;
  }
  if (strcmp(cmd, "PARAMS") == 0) {
    char *arg = strtok(NULL, " \t\r\n");
    if (!arg) {
      return false;
    }
    int id = -1;
    if (strcmp(arg, "LEFT") == 0)
      id = LEFT_MOTOR_ID;
    else if (strcmp(arg, "RIGHT") == 0)
      id = RIGHT_MOTOR_ID;
    else
      id = atoi(arg);
#if MOTOR_DRIVER_REAL
    if (!s_servoInitialized) {
      LOG_PRINTLN(
          abbot::log::CHANNEL_MOTOR,
          "motor_driver: PARAMS requested but servo bus not initialized");
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
    LOG_PRINTF(abbot::log::CHANNEL_MOTOR, "  min_angle=%d max_angle=%d\n",
               min_angle, max_angle);
    int cw_dead = s_servoBus.readByte(id, SMS_STS_CW_DEAD);
    int ccw_dead = s_servoBus.readByte(id, SMS_STS_CCW_DEAD);
    LOG_PRINTF(abbot::log::CHANNEL_MOTOR, "  cw_dead=%d ccw_dead=%d\n", cw_dead,
               ccw_dead);
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
    LOG_PRINTF(abbot::log::CHANNEL_MOTOR,
               "  present_pos=%d speed=%d load=%d volt=%d temp=%d moving=%d "
               "current=%d\n",
               pos, speed, load, volt, temp, moving, current);
    return true;
#else
    LOG_PRINTLN(abbot::log::CHANNEL_MOTOR,
                "motor_driver: PARAMS only supported in real mode");
    return true;
#endif
  }
  if (strcmp(cmd, "DUMP") == 0) {
    dumpConfig();
    return true;
  }
  if (strcmp(cmd, "RESETPOS") == 0) {
    resetPositionTracking();
    return true;
  }
  if (strcmp(cmd, "POS") == 0) {
    // Show current position tracking state
    LOG_PRINTF(abbot::log::CHANNEL_MOTOR,
               "motor_driver: LEFT  target=%lld accumulated=%lld error=%lld\n",
               s_target_pos_left, s_accumulated_pos_left,
               s_target_pos_left - s_accumulated_pos_left);
    LOG_PRINTF(abbot::log::CHANNEL_MOTOR,
               "motor_driver: RIGHT target=%lld accumulated=%lld error=%lld\n",
               s_target_pos_right, s_accumulated_pos_right,
               s_target_pos_right - s_accumulated_pos_right);
    return true;
  }
  if (strcmp(cmd, "ACC") == 0) {
    // Runtime acceleration setter: MOTOR ACC <LEFT|RIGHT|id> <value>
    char *arg = strtok(NULL, " \t\r\n");
    if (!arg) {
      LOG_PRINTLN(abbot::log::CHANNEL_MOTOR,
                  "Usage: MOTOR ACC <LEFT|RIGHT|id> <value>");
      return true;
    }
    int id = -1;
    if (strcmp(arg, "LEFT") == 0)
      id = LEFT_MOTOR_ID;
    else if (strcmp(arg, "RIGHT") == 0)
      id = RIGHT_MOTOR_ID;
    else
      id = atoi(arg);
    char *v = strtok(NULL, " \t\r\n");
    if (!v) {
      LOG_PRINTLN(abbot::log::CHANNEL_MOTOR,
                  "Usage: MOTOR ACC <LEFT|RIGHT|id> <value>");
      return true;
    }
    int acc = atoi(v);
#if MOTOR_DRIVER_REAL
    if (!s_servoInitialized) {
      LOG_PRINTLN(abbot::log::CHANNEL_MOTOR,
                  "motor_driver: ACC requested but servo bus not initialized");
      return true;
    }
    int rc = s_servoBus.writeByte((uint8_t)id, SMS_STS_ACC, (uint8_t)acc);
    LOG_PRINTF(abbot::log::CHANNEL_MOTOR,
               "motor_driver: ACC id=%d acc=%d rc=%d\n", id, acc, rc);
#else
    LOG_PRINTF(abbot::log::CHANNEL_MOTOR,
               "motor_driver: STUB ACC id=%d acc=%d\n", id, acc);
#endif
    return true;
  }
  if (strcmp(cmd, "VEL") == 0) {
    // Direct velocity command: MOTOR VEL <LEFT|RIGHT> <speed>
    // Sends velocity directly via WriteSpe (bypasses closed-loop but applies
    // inversion)
    char *arg = strtok(NULL, " \t\r\n");
    if (!arg) {
      LOG_PRINTLN(abbot::log::CHANNEL_MOTOR,
                  "Usage: MOTOR VEL <LEFT|RIGHT> <speed>");
      return true;
    }
    int id = -1;
    if (strcmp(arg, "LEFT") == 0) {
      id = LEFT_MOTOR_ID;
    } else if (strcmp(arg, "RIGHT") == 0) {
      id = RIGHT_MOTOR_ID;
    } else {
      id = atoi(arg);
    }

    char *v = strtok(NULL, " \t\r\n");
    if (!v) {
      LOG_PRINTLN(abbot::log::CHANNEL_MOTOR,
                  "Usage: MOTOR VEL <LEFT|RIGHT> <speed>");
      return true;
    }
    int16_t speed = (int16_t)atoi(v);
    setMotorCommandRaw(id, speed);
    
    // Immediately attempt to update the firmware-side accumulated encoder so
    // that subsequent "MOTOR READ" calls reflect recent motion even when the
    // command was issued via the direct VEL console path.
    // A small inter-command delay helps the servo update its registers first.
    delayMicroseconds(MOTOR_INTER_COMMAND_DELAY_US);
    (void)readAndUpdateEncoder(id);
    return true;
  }

  return false;
}

} // namespace motor
} // namespace abbot
