// serial_commands.cpp
#include "serial_commands.h"
#include "imu_calibration.h"
#include "motor_driver.h"
#include "../config/motor_config.h"
#include "BMI088Driver.h"
#include "logging.h"
#include "tuning_capture.h"
#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "SystemTasks.h"
#include "serial_menu.h"
#include "../include/balancer_controller.h"

namespace abbot {
namespace serialcmds {

// forward-declare startInteractiveMenu (implemented after ensureMenus)
void startInteractiveMenu(abbot::BMI088Driver *driver);

// Globals for interactive menu state (previously in this file)
static SerialMenu *g_rootMenu = nullptr;
static SerialMenu *g_currentMenu = nullptr;
static bool g_menuActive = false;
static bool g_building_menu = false;
static portMUX_TYPE g_menu_mux = portMUX_INITIALIZER_UNLOCKED;
// Per-submenu globals (used by menu callbacks)
static SerialMenu *g_calibMenu = nullptr;
static SerialMenu *g_motorMenu = nullptr;
static SerialMenu *g_tuningMenu = nullptr;
static SerialMenu *g_logMenu = nullptr;

// Forward-declare helper builders and ensureMenus
static SerialMenu* buildCalibrationMenu(abbot::BMI088Driver *driver);
static SerialMenu* buildMotorMenu();
static SerialMenu* buildTuningMenu();
static SerialMenu* buildLogMenu();
static SerialMenu* buildBalancerMenu();
static void ensureMenus(abbot::BMI088Driver *driver);
static void onCaptureCompleteRefreshMenu();

// Forward-declare extracted menu action helpers
static void calibStartGyro(abbot::BMI088Driver *driver, const String &p);
static void calibStartAccel(abbot::BMI088Driver *driver, const String &p);
static void motorSetHandler(const String &p);
static void tuningCustomHandler(const String &p);
static void tuningStreamStartHandler(const String &p);
static void tuningStreamStopHandler(const String &p);
static void logMenuOnEnter();
static void balancerSetGainsHandler(const String &p);
static void balancerTuningStartHandler(const String &p);
static void balancerTuningStopHandler(const String &p);
static void balancerDeadbandSetHandler(const String &p);
static void autotuneRelayHandler(const String &p);
static void autotuneDeadbandHandler(const String &p);
static void autotuneMaxAngleHandler(const String &p);

// Submenu builder helpers to reduce size of ensureMenus
static SerialMenu* buildCalibrationMenu(abbot::BMI088Driver *driver) {
  SerialMenu *m = new SerialMenu("Calibration Commands");
  m->addEntry(1, "CALIB START GYRO [N]", [driver](const String& p){ calibStartGyro(driver, p); });
  m->addEntry(2, "CALIB START ACCEL [N]", [driver](const String& p){
    calibStartAccel(driver, p);
  });
  m->addEntry(3, "CALIB DUMP", [](const String&){ abbot::imu_cal::dumpCalibration(); });
  m->addEntry(4, "CALIB RESET", [](const String&){ abbot::imu_cal::resetCalibration(); });
  return m;
}

static SerialMenu* buildMotorMenu() {
  SerialMenu *m = new SerialMenu("Motor Commands");
  m->addEntry(1, "MOTOR ENABLE", [](const String&){ abbot::motor::enableMotors(); });
  m->addEntry(2, "MOTOR DISABLE", [](const String&){ abbot::motor::disableMotors(); });
  m->addEntry(3, "MOTOR STATUS", [](const String&){ abbot::motor::printStatus(); });
  m->addEntry(4, "MOTOR DUMP", [](const String&){ abbot::motor::dumpConfig(); });
  m->addEntry(5, "MOTOR RESETPOS", [](const String&){ abbot::motor::resetPositionTracking(); });
  m->addEntry(6, "MOTOR POS", [](const String&){ abbot::motor::processSerialCommand("POS"); });
  m->addEntry(7, "MOTOR VEL <LEFT|RIGHT> <speed>", [](const String& p){
    String cmd = "VEL " + p;
    abbot::motor::processSerialCommand(cmd);
  });
  m->addEntry(8, "MOTOR SET <LEFT|RIGHT|ID> <v>", [](const String& p){ motorSetHandler(p); });
  m->addEntry(9, "MOTOR PARAMS <LEFT|RIGHT>", [](const String& p){ String cmd = "PARAMS " + p; abbot::motor::processSerialCommand(cmd); });
  m->addEntry(10, "MOTOR ACC <LEFT|RIGHT> <value>", [](const String& p){ String cmd = "ACC " + p; abbot::motor::processSerialCommand(cmd); });
  return m;
}

static SerialMenu* buildTuningMenu() {
  SerialMenu *m = new SerialMenu("Tuning (Madgwick)");
  m->addEntry(1, "TUNING STATIC (2000)", [](const String&){
    abbot::log::pushChannelMask(static_cast<uint32_t>(abbot::log::CHANNEL_DEFAULT) | static_cast<uint32_t>(abbot::log::CHANNEL_TUNING));
    abbot::tuning::startCapture(2000, true);
  });
  m->addEntry(2, "TUNING DYNAMIC (20000)", [](const String&){
    abbot::log::pushChannelMask(static_cast<uint32_t>(abbot::log::CHANNEL_DEFAULT) | static_cast<uint32_t>(abbot::log::CHANNEL_TUNING));
    abbot::tuning::startCapture(20000, true);
  });
  m->addEntry(3, "TUNING CUSTOM <N>", [](const String& p){
    tuningCustomHandler(p);
  });
  m->addEntry(6, "TUNING STATIC STATS (2000)", [](const String&){
    abbot::log::pushChannelMask(static_cast<uint32_t>(abbot::log::CHANNEL_DEFAULT) | static_cast<uint32_t>(abbot::log::CHANNEL_TUNING));
    abbot::tuning::startCapture(2000, false, true);
  });
  m->addEntry(7, "TUNING DYNAMIC STATS (20000)", [](const String&){
    abbot::log::pushChannelMask(static_cast<uint32_t>(abbot::log::CHANNEL_DEFAULT) | static_cast<uint32_t>(abbot::log::CHANNEL_TUNING));
    abbot::tuning::startCapture(20000, false, true);
  });
  m->addEntry(8, "TUNING CUSTOM STATS <N>", [](const String& p){
    String s = p; s.trim(); if (s.length() == 0) { LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "Usage: 8 <N> (number of samples)"); return; }
    int n = s.toInt(); if (n <= 0) { LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "Invalid sample count"); return; }
    abbot::log::pushChannelMask(static_cast<uint32_t>(abbot::log::CHANNEL_DEFAULT) | static_cast<uint32_t>(abbot::log::CHANNEL_TUNING));
    abbot::tuning::startCapture((uint32_t)n, false, true);
  });
  m->addEntry(4, "TUNING STREAM START", [](const String&){
    tuningStreamStartHandler(String());
  });
  m->addEntry(5, "TUNING STREAM STOP", [](const String&){
    tuningStreamStopHandler(String());
  });
  return m;
}

static SerialMenu* buildLogMenu() {
  SerialMenu *m = new SerialMenu("Log Channels");
  m->setOnEnter([](){ logMenuOnEnter(); });
  return m;
}

static SerialMenu* buildBalancerMenu() {
  SerialMenu *bal = new SerialMenu("Balancer (PID)");
  bal->addEntry(1, "BALANCE START", [](const String&){ abbot::balancer::controller::start(abbot::getFusedPitch()); });
  bal->addEntry(2, "BALANCE STOP", [](const String&){ abbot::balancer::controller::stop(); });
  bal->addEntry(3, "BALANCE GET_GAINS", [](const String&){
    float kp,ki,kd; abbot::balancer::controller::getGains(kp,ki,kd);
    char buf[128]; snprintf(buf, sizeof(buf), "BALANCER: Kp=%.6f Ki=%.6f Kd=%.6f", (double)kp, (double)ki, (double)kd);
    LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, buf);
  });
  // Group: gains
  bal->addEntry(4, "BALANCE SET GAINS <kp> <ki> <kd>", [](const String& p){ balancerSetGainsHandler(p); });
  bal->addEntry(5, "BALANCE RESET GAINS", [](const String&){ abbot::balancer::controller::resetGainsToDefaults(); });
  // Group: deadband
  bal->addEntry(6, "BALANCE DEADBAND GET", [](const String&){ float db = abbot::balancer::controller::getDeadband(); char buf[128]; snprintf(buf, sizeof(buf), "BALANCER: deadband=%.6f", (double)db); LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, buf); });
  bal->addEntry(7, "BALANCE DEADBAND SET <v>", [](const String& p){ balancerDeadbandSetHandler(p); });
  bal->addEntry(8, "BALANCE DEADBAND CALIBRATE", [](const String&){ abbot::balancer::controller::calibrateDeadband(); });
  // Group: autotune
  bal->addEntry(9, "AUTOTUNE START", [](const String&){ abbot::balancer::controller::startAutotune(); });
  bal->addEntry(10, "AUTOTUNE STOP", [](const String&){ abbot::balancer::controller::stopAutotune(); });
  bal->addEntry(11, "AUTOTUNE STATUS", [](const String&){ const char* status = abbot::balancer::controller::getAutotuneStatus(); char buf[128]; snprintf(buf, sizeof(buf), "AUTOTUNE: %s", status); LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, buf); });
  bal->addEntry(12, "AUTOTUNE APPLY", [](const String&){ abbot::balancer::controller::applyAutotuneGains(); });
  bal->addEntry(13, "AUTOTUNE RELAY <amp>", [](const String& p){ autotuneRelayHandler(p); });
  bal->addEntry(14, "AUTOTUNE DEADBAND <deg>", [](const String& p){ autotuneDeadbandHandler(p); });
  bal->addEntry(15, "AUTOTUNE MAXANGLE <deg>", [](const String& p){ autotuneMaxAngleHandler(p); });
  return bal;
}

static void onCaptureCompleteRefreshMenu() {
  if (g_menuActive && g_currentMenu) {
    g_currentMenu->enter();
  }
}

static void ensureMenus(abbot::BMI088Driver *driver) {
  // Avoid rebuilding concurrently
  if (g_rootMenu || g_building_menu) return;
  g_building_menu = true;

  // Build root locally and only assign to global after fully populated
  SerialMenu *root = new SerialMenu("Main Menu");
  g_calibMenu = buildCalibrationMenu(driver);
  g_motorMenu = buildMotorMenu();
  g_tuningMenu = buildTuningMenu();
  g_logMenu = buildLogMenu();
  SerialMenu *bal = buildBalancerMenu();

  // Root menu entries link to submenus
  root->addSubmenu(1, "Calibration (BMI088)", g_calibMenu);
  root->addSubmenu(2, "Motor control", g_motorMenu);
  root->addSubmenu(3, "Madgwick tuning", g_tuningMenu);
  root->addSubmenu(4, "Log channels", g_logMenu);
  root->addSubmenu(5, "Balancer (PID)", bal);

  // Commit to globals after fully-built to avoid races
  portENTER_CRITICAL(&g_menu_mux);
  g_rootMenu = root;
  g_building_menu = false;
  portEXIT_CRITICAL(&g_menu_mux);

  // Register capture-complete callback so the interactive menu can be re-shown
  // when an auto-capture finishes.
  abbot::tuning::setOnCaptureComplete(onCaptureCompleteRefreshMenu);
}

// Utility: tokenize an input string into uppercase tokens using a buffer.
// Returns number of tokens (<= maxTokens). `buf` must have size bufsize.
static int tokenizeUpper(const String &src, char *buf, size_t bufsize, char *tokens[], int maxTokens) {
  String tmp = src;
  tmp.toUpperCase();
  tmp.toCharArray(buf, (int)bufsize);
  char *tk = strtok(buf, " \t\r\n");
  int i = 0;
  while (tk && i < maxTokens) {
    tokens[i++] = tk;
    tk = strtok(NULL, " \t\r\n");
  }
  return i;
}

// Tuning helpers: centralize repeated push/pop and stream header printing
static void pushTuningMask() {
  abbot::log::pushChannelMask(static_cast<uint32_t>(abbot::log::CHANNEL_DEFAULT) | static_cast<uint32_t>(abbot::log::CHANNEL_TUNING));
}

static void startTuningStream() {
  pushTuningMask();
  abbot::log::enableChannel(abbot::log::CHANNEL_TUNING);
  LOG_PRINTLN(abbot::log::CHANNEL_TUNING, "timestamp_ms,pitch_deg,pitch_rad,pitch_rate_deg,pitch_rate_rad,ax,ay,az,gx,gy,gz,temp_C,left_cmd,right_cmd");
  LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "TUNING: started");
}

static void stopTuningStream() {
  abbot::log::disableChannel(abbot::log::CHANNEL_TUNING);
  abbot::log::popChannelMask();
  LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "TUNING: stopped");
  if (g_menuActive && g_currentMenu) g_currentMenu->enter();
}

static void startTuningCapture(uint32_t samples, bool csv, bool statsOnly=false) {
  pushTuningMask();
  if (statsOnly) {
    abbot::tuning::startCapture(samples, false, true);
  } else {
    abbot::tuning::startCapture(samples, csv);
  }
}

// Extracted action handlers
static void calibStartGyro(abbot::BMI088Driver *driver, const String &p) {
  int n = 2000;
  if (p.length()) n = p.toInt();
  if (!abbot::imu_cal::isCalibrating()) {
    abbot::log::pushChannelMask(static_cast<uint32_t>(abbot::log::CHANNEL_DEFAULT) | static_cast<uint32_t>(abbot::log::CHANNEL_IMU));
    bool ok = abbot::imu_cal::startGyroCalibration(*driver, n);
    (void)ok;
    abbot::log::popChannelMask();
    if (g_menuActive && g_currentMenu) g_currentMenu->enter();
  }
}

static void calibStartAccel(abbot::BMI088Driver *driver, const String &p) {
  int n = 2000;
  if (p.length()) n = p.toInt();
  if (!abbot::imu_cal::isCalibrating()) {
    abbot::log::pushChannelMask(static_cast<uint32_t>(abbot::log::CHANNEL_DEFAULT) | static_cast<uint32_t>(abbot::log::CHANNEL_IMU));
    bool ok = abbot::imu_cal::startAccelCalibration(*driver, n);
    (void)ok;
    abbot::log::popChannelMask();
    if (g_menuActive && g_currentMenu) g_currentMenu->enter();
  }
}

static void motorSetHandler(const String &p) {
  String s = p; s.trim();
  int sp = s.indexOf(' ');
  if (sp == -1) { LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "Usage: <side> <value>"); return; }
  String side = s.substring(0, sp); String val = s.substring(sp+1);
  side.toUpperCase();
  if (side == "LEFT") {
    abbot::motor::setMotorCommand(LEFT_MOTOR_ID, val.toFloat());
  } else if (side == "RIGHT") {
    abbot::motor::setMotorCommand(RIGHT_MOTOR_ID, val.toFloat());
  } else {
    int id = side.toInt(); abbot::motor::setMotorCommandRaw(id, (int)val.toInt());
  }
}

static void tuningCustomHandler(const String &p) {
  String s = p; s.trim(); if (s.length() == 0) { LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "Usage: <N> (number of samples)"); return; }
  int n = s.toInt(); if (n <= 0) { LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "Invalid sample count"); return; }
  startTuningCapture((uint32_t)n, true);
}

static void tuningStreamStartHandler(const String & /*unused*/) {
  startTuningStream();
}

static void tuningStreamStopHandler(const String & /*unused*/) {
  stopTuningStream();
}

static void logMenuOnEnter() {
  // clear and rebuild entries
  if (!g_logMenu) return;
  g_logMenu->clearEntries();
  struct C { abbot::log::Channel ch; const char* name; } channels[] = {
    { abbot::log::CHANNEL_TUNING,  "TUNING" },
    { abbot::log::CHANNEL_BLE,     "BLE" },
    { abbot::log::CHANNEL_IMU,     "IMU" },
    { abbot::log::CHANNEL_MOTOR,   "MOTOR" },
    { abbot::log::CHANNEL_BALANCER, "BALANCER" },
    { abbot::log::CHANNEL_DEFAULT, "DEFAULT" },
  };
  int id = 1;
  for (auto &c : channels) {
    const char* name = abbot::log::channelName(c.ch);
    bool on = abbot::log::isChannelEnabled(c.ch);
    char lbl[64];
    snprintf(lbl, sizeof(lbl), "%s: %s", name, on ? "ON" : "OFF");
    abbot::log::Channel chcopy = c.ch;
    g_logMenu->addEntry(id++, lbl, [chcopy](const String&){
      bool newState = abbot::log::toggleChannel(chcopy);
      char msg[64];
      snprintf(msg, sizeof(msg), "%s %s", abbot::log::channelName(chcopy), newState ? "enabled" : "disabled");
      LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, msg);
      if (g_logMenu) { g_logMenu->enter(); }
    });
  }
}

static void balancerSetGainsHandler(const String &p) {
  String s = p; s.trim(); float kp=0, ki=0, kd=0; int matched = sscanf(s.c_str(), "%f %f %f", &kp, &ki, &kd);
  (void)matched; abbot::balancer::controller::setGains(kp, ki, kd);
}

static void balancerTuningStartHandler(const String &p) {
  String s = p; s.trim(); int samples = 2000; if (s.length() > 0) samples = s.toInt(); if (samples < 100) samples = 100;
  startTuningCapture((uint32_t)samples, true);
}

static void balancerTuningStopHandler(const String & /*unused*/) {
  abbot::tuning::stopCapture();
}

static void balancerDeadbandSetHandler(const String &p) {
  String s = p; s.trim(); if (s.length() == 0) { LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "Usage: BALANCE DEADBAND SET <value>"); return; }
  float v = s.toFloat(); abbot::balancer::controller::setDeadband(v);
}

static void autotuneRelayHandler(const String &p) {
  String s = p; s.trim(); if (s.length() == 0) { LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "Usage: AUTOTUNE RELAY <amplitude>"); return; }
  float amp = s.toFloat(); abbot::balancer::controller::setAutotuneRelay(amp);
}

static void autotuneDeadbandHandler(const String &p) {
  String s = p; s.trim(); if (s.length() == 0) { LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "Usage: AUTOTUNE DEADBAND <degrees>"); return; }
  float db = s.toFloat(); abbot::balancer::controller::setAutotuneDeadband(db);
}

static void autotuneMaxAngleHandler(const String &p) {
  String s = p; s.trim(); if (s.length() == 0) { LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "Usage: AUTOTUNE MAXANGLE <degrees>"); return; }
  float maxa = s.toFloat(); abbot::balancer::controller::setAutotuneMaxAngle(maxa);
}

// Command handlers: return true if the command was handled
static bool handleCalib(abbot::BMI088Driver *driver, const String& line, const String& up) {
  if (!up.startsWith("CALIB")) return false;
  char buf[128]; char *tok[8]; int nt = tokenizeUpper(up, buf, sizeof(buf), tok, 8);
  if (nt < 2) return true;
  char *t2 = tok[1];
  if (strcmp(t2, "START") == 0) {
    char *what = (nt >= 3) ? tok[2] : nullptr;
    if (!what) return true;
    int sampleCount = 2000;
    if (nt >= 4) { int v = atoi(tok[3]); if (v > 0) sampleCount = v; }
    if (strcmp(what, "GYRO") == 0) {
      if (!abbot::imu_cal::isCalibrating()) {
        abbot::log::pushChannelMask(static_cast<uint32_t>(abbot::log::CHANNEL_DEFAULT) | static_cast<uint32_t>(abbot::log::CHANNEL_IMU));
        bool ok = abbot::imu_cal::startGyroCalibration(*driver, sampleCount);
        (void)ok;
        abbot::log::popChannelMask();
        if (g_menuActive && g_currentMenu) g_currentMenu->enter();
      }
      return true;
    } else if (strcmp(what, "ACCEL") == 0) {
      if (!abbot::imu_cal::isCalibrating()) {
        abbot::log::pushChannelMask(static_cast<uint32_t>(abbot::log::CHANNEL_DEFAULT) | static_cast<uint32_t>(abbot::log::CHANNEL_IMU));
        bool ok = abbot::imu_cal::startAccelCalibration(*driver, sampleCount);
        (void)ok;
        abbot::log::popChannelMask();
        if (g_menuActive && g_currentMenu) g_currentMenu->enter();
      }
      return true;
    }
  } else if (strcmp(t2, "DUMP") == 0) {
    abbot::imu_cal::dumpCalibration();
    return true;
  } else if (strcmp(t2, "RESET") == 0) {
    abbot::imu_cal::resetCalibration();
    return true;
  }
  return true;
}

static bool handleTuning(const String& line, const String& up) {
  if (!up.startsWith("TUNING")) return false;
  char buf2[64]; char *tok[4]; int nt = tokenizeUpper(up, buf2, sizeof(buf2), tok, 4);
  char *arg = (nt>=2)?tok[1]:nullptr; char *arg2 = (nt>=3)?tok[2]:nullptr;
  if (arg && strcmp(arg, "START") == 0) {
    if (arg2) {
      int n = atoi(arg2);
      if (n > 0) {
        startTuningCapture((uint32_t)n, true);
        LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "TUNING: auto-capture started");
        return true;
      }
    }
    startTuningStream();
    return true;
  } else if (arg && strcmp(arg, "STOP") == 0) {
    stopTuningStream();
    return true;
  } else if (arg && strcmp(arg, "START_STATS") == 0) {
    int sampleCount = 2000; if (arg2) { int v = atoi(arg2); if (v > 0) sampleCount = v; }
    startTuningCapture((uint32_t)sampleCount, false, true);
    LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "TUNING: auto-capture (stats-only) started");
    return true;
  } else {
    LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "TUNING usage: TUNING START | TUNING STOP");
    return true;
  }
  return true;
}

static bool handleLog(const String& line, const String& up) {
  if (!up.startsWith("LOG")) return false;
  char buf3[128]; char *tok[4]; int nt = tokenizeUpper(up, buf3, sizeof(buf3), tok, 4);
  char *cmd = (nt>=2)?tok[1]:nullptr;
  if (!cmd) { LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "LOG usage: LOG ENABLE|DISABLE <CHANNEL> | LOG LIST"); return true; }
  if (strcmp(cmd, "LIST") == 0) { char out[128]; abbot::log::listEnabledChannels(out, sizeof(out)); LOG_PRINTF(abbot::log::CHANNEL_DEFAULT, "LOG enabled: %s\n", out); return true; }
  char *argch = (nt>=3)?tok[2]:nullptr; if (!argch) { LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "LOG usage: LOG ENABLE|DISABLE <CHANNEL>"); return true; }
  auto ch = abbot::log::channelFromString(argch);
  if (ch == static_cast<abbot::log::Channel>(0)) { LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "Unknown channel. Known: TUNING,BLE,IMU,MOTOR,DEFAULT"); return true; }
  if (strcmp(cmd, "ENABLE") == 0) { abbot::log::enableChannel(ch); LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "LOG: enabled"); return true; }
  else if (strcmp(cmd, "DISABLE") == 0) { abbot::log::disableChannel(ch); LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "LOG: disabled"); return true; }
  LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "LOG usage: LOG ENABLE|DISABLE <CHANNEL> | LOG LIST");
  return true;
}

static bool handleBalance(const String& line, const String& up) {
  if (!up.startsWith("BALANCE")) return false;
  char buf4[128]; char *tok[8]; int nt = tokenizeUpper(up, buf4, sizeof(buf4), tok, 8);
  if (nt < 2) { LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "BALANCE usage: BALANCE START | BALANCE STOP | BALANCE GAINS [<kp> <ki> <kd>] | BALANCE RESET | BALANCE GET_GAINS | BALANCE DEADBAND GET|SET|CALIBRATE"); return true; }
  char *arg = tok[1];
  if (strcmp(arg, "START") == 0) { abbot::balancer::controller::start(abbot::getFusedPitch()); return true; }
  else if (strcmp(arg, "STOP") == 0) { abbot::balancer::controller::stop(); return true; }
  else if (strcmp(arg, "RESET") == 0) { abbot::balancer::controller::resetGainsToDefaults(); return true; }
  else if (strcmp(arg, "GAINS") == 0) {
    if (nt < 5) {
      float kp,ki,kd; abbot::balancer::controller::getGains(kp,ki,kd);
      char buf[128]; snprintf(buf, sizeof(buf), "BALANCER: Kp=%.6f Ki=%.6f Kd=%.6f", (double)kp, (double)ki, (double)kd);
      LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, buf);
      return true;
    }
    float kp = atof(tok[2]); float ki = atof(tok[3]); float kd = atof(tok[4]);
    abbot::balancer::controller::setGains(kp, ki, kd);
    return true;
  } else if (strcmp(arg, "GET_GAINS") == 0) { float kp,ki,kd; abbot::balancer::controller::getGains(kp,ki,kd); char buf[128]; snprintf(buf, sizeof(buf), "BALANCER: Kp=%.6f Ki=%.6f Kd=%.6f", (double)kp, (double)ki, (double)kd); LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, buf); return true; }
  else if (strcmp(arg, "DEADBAND") == 0) {
    if (nt < 3) { LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "BALANCE DEADBAND usage: DEADBAND GET | DEADBAND SET <v> | DEADBAND CALIBRATE"); return true; }
    char *sub = tok[2];
    if (strcmp(sub, "GET") == 0) { float db = abbot::balancer::controller::getDeadband(); char buf[128]; snprintf(buf, sizeof(buf), "BALANCER: deadband=%.6f", (double)db); LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, buf); return true; }
    else if (strcmp(sub, "SET") == 0) { if (nt < 4) { LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "Usage: BALANCE DEADBAND SET <value>"); return true; } float v = atof(tok[3]); abbot::balancer::controller::setDeadband(v); return true; }
    else if (strcmp(sub, "CALIBRATE") == 0) { abbot::balancer::controller::calibrateDeadband(); return true; }
  }
  LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "BALANCE usage: BALANCE START | BALANCE STOP | BALANCE GAINS [<kp> <ki> <kd>] | BALANCE RESET | BALANCE DEADBAND GET|SET|CALIBRATE");
  return true;
}

static bool handleAutotune(const String& line, const String& up) {
  if (!up.startsWith("AUTOTUNE")) return false;
  char buf5[128]; char *tok[6]; int nt = tokenizeUpper(up, buf5, sizeof(buf5), tok, 6);
  if (nt < 2) { LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "AUTOTUNE usage: AUTOTUNE START | AUTOTUNE STOP | AUTOTUNE STATUS | AUTOTUNE APPLY | AUTOTUNE RELAY <amp> | AUTOTUNE DEADBAND <deg> | AUTOTUNE MAXANGLE <deg>"); return true; }
  char *arg = tok[1];
  if (strcmp(arg, "START") == 0) { abbot::balancer::controller::startAutotune(); return true; }
  else if (strcmp(arg, "SAFE") == 0) { abbot::balancer::controller::setAutotuneRelay(0.15f); abbot::balancer::controller::setAutotuneDeadband(0.5f); abbot::balancer::controller::setAutotuneMaxAngle(8.0f); LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "AUTOTUNE: starting in SAFE mode (relay=0.15 deadband=0.5° maxangle=8°)"); abbot::balancer::controller::startAutotune(); return true; }
  else if (strcmp(arg, "STOP") == 0) { abbot::balancer::controller::stopAutotune(); return true; }
  else if (strcmp(arg, "STATUS") == 0) { const char* status = abbot::balancer::controller::getAutotuneStatus(); char buf[128]; snprintf(buf, sizeof(buf), "AUTOTUNE: %s", status); LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, buf); return true; }
  else if (strcmp(arg, "APPLY") == 0) { abbot::balancer::controller::applyAutotuneGains(); return true; }
  else if (strcmp(arg, "RELAY") == 0) { if (nt < 3) { LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "Usage: AUTOTUNE RELAY <amplitude>"); return true; } float amp = atof(tok[2]); abbot::balancer::controller::setAutotuneRelay(amp); return true; }
  else if (strcmp(arg, "DEADBAND") == 0) { if (nt < 3) { LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "Usage: AUTOTUNE DEADBAND <degrees>"); return true; } float db = atof(tok[2]); abbot::balancer::controller::setAutotuneDeadband(db); return true; }
  else if (strcmp(arg, "MAXANGLE") == 0) { if (nt < 3) { LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "Usage: AUTOTUNE MAXANGLE <degrees>"); return true; } float maxa = atof(tok[2]); abbot::balancer::controller::setAutotuneMaxAngle(maxa); return true; }
  LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "AUTOTUNE usage: AUTOTUNE START | AUTOTUNE STOP | AUTOTUNE STATUS | AUTOTUNE APPLY | AUTOTUNE RELAY <amp> | AUTOTUNE DEADBAND <deg> | AUTOTUNE MAXANGLE <deg>");
  return true;
}

// Start the interactive menu programmatically (prints the menu and makes it active)
void startInteractiveMenu(abbot::BMI088Driver *driver) {
  // Ensure menus exist. Another task may be building them concurrently
  // (serialTaskEntry runs in a different task); wait briefly if so.
  ensureMenus(driver);
  // If another task is building the menu, wait for it to finish so that
  // g_rootMenu is non-null and we can safely enter it. Timeout to avoid
  // blocking setup for too long.
  int attempts = 0;
  while (!g_rootMenu && g_building_menu && attempts < 20) {
    // 10 ms per attempt -> up to ~200 ms total
    vTaskDelay(pdMS_TO_TICKS(10));
    ++attempts;
  }
  // Final ensure in case it wasn't built yet
  ensureMenus(driver);
  g_menuActive = true;
  g_currentMenu = g_rootMenu;
  if (g_currentMenu) {
    g_currentMenu->enter();
  }
}


void processSerialOnce(class abbot::BMI088Driver *driver) {
  if (!driver) {
    return;
  }
  ensureMenus(driver);
  if (!Serial || Serial.available() == 0) {
    return;
  }
  String line = Serial.readStringUntil('\n');
  // Optional RX echo for debugging user-typed commands. Define
  // `ENABLE_RX_ECHO` at compile time to enable this (see platformio.ini
  // build_flags). Disabled by default to keep serial output clean.
#ifdef ENABLE_RX_ECHO
  {
    String tline = line;
    tline.trim();
    if (tline.length() > 0) {
      char dbg2[128];
      tline.toCharArray(dbg2, sizeof(dbg2));
      char outbuf[160];
      snprintf(outbuf, sizeof(outbuf), "RX: %s", dbg2);
      LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, outbuf);
    }
  }
#endif
  line.trim();
  if (line.length() == 0) {
    return;
  }

  // If interactive menu is active, only feed the menu when the input
  // looks like a numeric selection. Otherwise fall through so textual
  // commands (e.g. "TUNING START 2000") are still processed while the
  // menu is visible.
  if (g_menuActive && g_currentMenu) {
    // determine if first token is numeric
    String s = line;
    s.trim();
    int sp = s.indexOf(' ');
    String first = (sp == -1) ? s : s.substring(0, sp);
    bool firstIsNum = true;
    if (first.length() == 0) firstIsNum = false;
    for (unsigned int i = 0; i < first.length() && firstIsNum; ++i) {
      if (!isDigit(first.charAt(i))) firstIsNum = false;
    }
    if (firstIsNum) {
      SerialMenu* next = g_currentMenu->handleInput(line);
      if (next == nullptr) {
        // exit menu
        g_menuActive = false;
        g_currentMenu = nullptr;
      } else {
        g_currentMenu = next;
      }
      return;
    }
    // else: not a numeric selection - fall through to command parsing
  }
  // First try imu_cal commands
  // imu_cal functions expect upper-case tokens; we forward the raw line but
  // the imu_cal module uppercases internally when parsing.
  // Try to handle CALIB commands first
  String up = line;
  up.toUpperCase();
  // Try handlers for known command groups in order. Each returns true
  // when it handled the command and the caller should return early.
  if (handleCalib(driver, line, up)) return;

  // HELP command: enter interactive menu (numeric selections)
  if (up == "HELP" || up == "?" || up.startsWith("HELP ")) {
    ensureMenus(driver);
    g_menuActive = true;
    g_currentMenu = g_rootMenu;
    g_currentMenu->enter();
    return;
  }
  if (handleTuning(line, up)) return;
  if (handleLog(line, up)) return;
  if (handleBalance(line, up)) return;
  if (handleAutotune(line, up)) return;


  // Forward to motor driver processor
  if (abbot::motor::processSerialCommand(line)) {
    return;
  }

  LOG_PRINT(abbot::log::CHANNEL_DEFAULT, "Unknown command: "); LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, line);
}

void serialTaskEntry(void *pvParameters) {
  abbot::BMI088Driver *driver = reinterpret_cast<abbot::BMI088Driver*>(pvParameters);
  for (;;) {
    processSerialOnce(driver);
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

} // namespace serialcmds
} // namespace abbot
