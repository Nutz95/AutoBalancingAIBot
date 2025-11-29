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

// Interactive menu instance (constructed on first use)
static SerialMenu *g_rootMenu = nullptr;
static SerialMenu *g_calibMenu = nullptr;
static SerialMenu *g_motorMenu = nullptr;
static SerialMenu *g_tuningMenu = nullptr;
static SerialMenu *g_logMenu = nullptr;
static bool g_menuActive = false;
static SerialMenu* g_currentMenu = nullptr;
// Guard to prevent concurrent menu construction (race after upload)
static volatile bool g_building_menu = false;
// Mutex for protecting menu construction
static portMUX_TYPE g_menu_mux = portMUX_INITIALIZER_UNLOCKED;

// Called by tuning capture when it finishes so we can re-enter the current menu
static void onCaptureCompleteRefreshMenu() {
  if (g_menuActive && g_currentMenu) {
    g_currentMenu->enter();
  }
}

static void ensureMenus(abbot::BMI088Driver *driver) {
  // Fast-path: already built
  portENTER_CRITICAL(&g_menu_mux);
  if (g_rootMenu) {
    portEXIT_CRITICAL(&g_menu_mux);
    return;
  }
  // If another task is building the menus, wait until it's done
  if (g_building_menu) {
    portEXIT_CRITICAL(&g_menu_mux);
    // wait until builder finishes (should be quick)
    while (!g_rootMenu) {
      vTaskDelay(pdMS_TO_TICKS(1));
    }
    return;
  }
  // mark that we are building
  g_building_menu = true;
  portEXIT_CRITICAL(&g_menu_mux);

  // Build root locally and only assign to global after fully populated
  SerialMenu *root = new SerialMenu("Main Menu");

  // Calibration submenu
  g_calibMenu = new SerialMenu("Calibration Commands");
  g_calibMenu->addEntry(1, "CALIB START GYRO [N]", [driver](const String& p){
    int n = 2000;
    if (p.length()) {
      n = p.toInt();
    }
    if (!abbot::imu_cal::isCalibrating()) {
      // Temporarily restrict logs to DEFAULT + IMU while calibrating
      abbot::log::pushChannelMask(static_cast<uint32_t>(abbot::log::CHANNEL_DEFAULT) | static_cast<uint32_t>(abbot::log::CHANNEL_IMU));
      bool ok = abbot::imu_cal::startGyroCalibration(*driver, n);
      (void)ok;
      abbot::log::popChannelMask();
      // If interactive menu is active, re-show the current menu
      if (g_menuActive && g_currentMenu) {
        g_currentMenu->enter();
      }
    }
  });
  g_calibMenu->addEntry(2, "CALIB START ACCEL [N]", [driver](const String& p){
    int n = 2000;
    if (p.length()) {
      n = p.toInt();
    }
    if (!abbot::imu_cal::isCalibrating()) {
      // Temporarily restrict logs to DEFAULT + IMU while calibrating
      abbot::log::pushChannelMask(static_cast<uint32_t>(abbot::log::CHANNEL_DEFAULT) | static_cast<uint32_t>(abbot::log::CHANNEL_IMU));
      bool ok = abbot::imu_cal::startAccelCalibration(*driver, n);
      (void)ok;
      abbot::log::popChannelMask();
      // If interactive menu is active, re-show the current menu
      if (g_menuActive && g_currentMenu) {
        g_currentMenu->enter();
      }
    }
  });
  g_calibMenu->addEntry(3, "CALIB DUMP", [](const String&){ abbot::imu_cal::dumpCalibration(); });
  g_calibMenu->addEntry(4, "CALIB RESET", [](const String&){ abbot::imu_cal::resetCalibration(); });

  // Motor submenu
  g_motorMenu = new SerialMenu("Motor Commands");
  g_motorMenu->addEntry(1, "MOTOR ENABLE", [](const String&){ abbot::motor::enableMotors(); });
  g_motorMenu->addEntry(2, "MOTOR DISABLE", [](const String&){ abbot::motor::disableMotors(); });
  g_motorMenu->addEntry(3, "MOTOR STATUS", [](const String&){ abbot::motor::printStatus(); });
  g_motorMenu->addEntry(4, "MOTOR DUMP", [](const String&){ abbot::motor::dumpConfig(); });
  g_motorMenu->addEntry(5, "MOTOR SET <LEFT|RIGHT|ID> <v>", [](const String& p){
    // Expect: <side> <value>
    String s = p; s.trim();
    int sp = s.indexOf(' ');
    if (sp == -1) {
      LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "Usage: <side> <value>");
      return;
    }
    String side = s.substring(0, sp); String val = s.substring(sp+1);
    side.toUpperCase(); float v = val.toFloat();
    if (side == "LEFT") {
      abbot::motor::setMotorCommand(LEFT_MOTOR_ID, v);
    } else if (side == "RIGHT") {
      abbot::motor::setMotorCommand(RIGHT_MOTOR_ID, v);
    }
    else { int id = side.toInt(); abbot::motor::setMotorCommandRaw(id, (int)val.toInt()); }
  });

  // Tuning submenu
  g_tuningMenu = new SerialMenu("Tuning (Madgwick)");
  g_tuningMenu->addEntry(1, "TUNING STATIC (2000)", [](const String&){
    // Capture 2000 samples (static noise measurement)
    abbot::log::pushChannelMask(static_cast<uint32_t>(abbot::log::CHANNEL_DEFAULT) | static_cast<uint32_t>(abbot::log::CHANNEL_TUNING));
    abbot::tuning::startCapture(2000, true);
    if (g_menuActive && g_currentMenu) { g_currentMenu->enter(); }
  });
  g_tuningMenu->addEntry(2, "TUNING DYNAMIC (20000)", [](const String&){
    // Capture 20000 samples (dynamic maneuver)
    abbot::log::pushChannelMask(static_cast<uint32_t>(abbot::log::CHANNEL_DEFAULT) | static_cast<uint32_t>(abbot::log::CHANNEL_TUNING));
    abbot::tuning::startCapture(20000, true);
    if (g_menuActive && g_currentMenu) { g_currentMenu->enter(); }
  });
  g_tuningMenu->addEntry(3, "TUNING CUSTOM <N>", [](const String& p){
    String s = p; s.trim();
    if (s.length() == 0) {
      LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "Usage: 3 <N> (number of samples)");
      return;
    }
    int n = s.toInt();
    if (n <= 0) {
      LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "Invalid sample count");
      return;
    }
    abbot::log::pushChannelMask(static_cast<uint32_t>(abbot::log::CHANNEL_DEFAULT) | static_cast<uint32_t>(abbot::log::CHANNEL_TUNING));
    abbot::tuning::startCapture((uint32_t)n, true);
    if (g_menuActive && g_currentMenu) { g_currentMenu->enter(); }
  });
  // Stats-only menu entries (periodic progress + final summary, no CSV)
  g_tuningMenu->addEntry(6, "TUNING STATIC STATS (2000)", [](const String&){
    abbot::log::pushChannelMask(static_cast<uint32_t>(abbot::log::CHANNEL_DEFAULT) | static_cast<uint32_t>(abbot::log::CHANNEL_TUNING));
    abbot::tuning::startCapture(2000, false, true);
    if (g_menuActive && g_currentMenu) { g_currentMenu->enter(); }
  });
  g_tuningMenu->addEntry(7, "TUNING DYNAMIC STATS (20000)", [](const String&){
    abbot::log::pushChannelMask(static_cast<uint32_t>(abbot::log::CHANNEL_DEFAULT) | static_cast<uint32_t>(abbot::log::CHANNEL_TUNING));
    abbot::tuning::startCapture(20000, false, true);
    if (g_menuActive && g_currentMenu) { g_currentMenu->enter(); }
  });
  g_tuningMenu->addEntry(8, "TUNING CUSTOM STATS <N>", [](const String& p){
    String s = p; s.trim();
    if (s.length() == 0) {
      LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "Usage: 8 <N> (number of samples)");
      return;
    }
    int n = s.toInt();
    if (n <= 0) {
      LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "Invalid sample count");
      return;
    }
    abbot::log::pushChannelMask(static_cast<uint32_t>(abbot::log::CHANNEL_DEFAULT) | static_cast<uint32_t>(abbot::log::CHANNEL_TUNING));
    abbot::tuning::startCapture((uint32_t)n, false, true);
    if (g_menuActive && g_currentMenu) { g_currentMenu->enter(); }
  });
  g_tuningMenu->addEntry(4, "TUNING STREAM START", [](const String&){
    abbot::log::pushChannelMask(static_cast<uint32_t>(abbot::log::CHANNEL_DEFAULT) | static_cast<uint32_t>(abbot::log::CHANNEL_TUNING));
    abbot::startTuningStream();
    LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "TUNING: started (stream)");
    if (g_menuActive && g_currentMenu) { g_currentMenu->enter(); }
  });
  g_tuningMenu->addEntry(5, "TUNING STREAM STOP", [](const String&){
    abbot::stopTuningStream();
    abbot::log::popChannelMask();
    LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "TUNING: stopped (stream)");
    if (g_menuActive && g_currentMenu) {
      g_currentMenu->enter();
    }
  });

  // Log channels submenu: dynamically rebuild entries on enter so each
  // channel appears with its current ON/OFF state and selecting an item
  // toggles the channel.
  g_logMenu = new SerialMenu("Log Channels");
  g_logMenu->setOnEnter([](){
    // clear and rebuild entries
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
      // capture channel value by copy for the action
      abbot::log::Channel chcopy = c.ch;
      g_logMenu->addEntry(id++, lbl, [chcopy](const String&){
        bool newState = abbot::log::toggleChannel(chcopy);
        char msg[64];
        snprintf(msg, sizeof(msg), "%s %s", abbot::log::channelName(chcopy), newState ? "enabled" : "disabled");
        LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, msg);
        // re-enter menu to refresh labels
        if (g_logMenu) {
          g_logMenu->enter();
        }
      });
    }
  });

  // Root menu entries link to submenus
  root->addSubmenu(1, "Calibration (BMI088)", g_calibMenu);
  root->addSubmenu(2, "Motor control", g_motorMenu);
  root->addSubmenu(3, "Madgwick tuning", g_tuningMenu);
  // Balancer submenu
  SerialMenu *bal = new SerialMenu("Balancer (PID)");
  bal->addEntry(1, "BALANCE START", [](const String&){ abbot::balancer::controller::start(abbot::getFusedPitch()); });
  bal->addEntry(2, "BALANCE STOP", [](const String&){ abbot::balancer::controller::stop(); });
  bal->addEntry(3, "BALANCE GET_GAINS", [](const String&){
    float kp,ki,kd; abbot::balancer::controller::getGains(kp,ki,kd);
    char buf[128]; snprintf(buf, sizeof(buf), "BALANCER: Kp=%.6f Ki=%.6f Kd=%.6f", (double)kp, (double)ki, (double)kd);
    LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, buf);
  });
  bal->addEntry(4, "BALANCE SET GAINS <kp> <ki> <kd>", [](const String& p){
    String s = p; s.trim();
    float kp=0, ki=0, kd=0;
    int sp = s.indexOf(' ');
    // parse three floats
    float a=0,b=0,c=0;
    int matched = sscanf(s.c_str(), "%f %f %f", &a, &b, &c);
    if (matched >= 1) kp = a;
    if (matched >= 2) ki = b;
    if (matched >= 3) kd = c;
    abbot::balancer::controller::setGains(kp, ki, kd);
  });
  bal->addEntry(5, "BALANCE DEADBAND GET", [](const String&){
    float db = abbot::balancer::controller::getDeadband();
    char buf[128]; snprintf(buf, sizeof(buf), "BALANCER: deadband=%.6f", (double)db);
    LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, buf);
  });
  bal->addEntry(6, "BALANCE DEADBAND SET <v>", [](const String& p){
    String s = p; s.trim(); if (s.length()==0) { LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "Usage: BALANCE DEADBAND SET <value>"); return; }
    float v = s.toFloat(); abbot::balancer::controller::setDeadband(v);
  });
  bal->addEntry(7, "BALANCE DEADBAND CALIBRATE", [](const String&){ abbot::balancer::controller::calibrateDeadband(); });
  bal->addEntry(8, "AUTOTUNE START", [](const String&){ abbot::balancer::controller::startAutotune(); });
  bal->addEntry(9, "AUTOTUNE STOP", [](const String&){ abbot::balancer::controller::stopAutotune(); });
  bal->addEntry(10, "AUTOTUNE STATUS", [](const String&){
    const char* status = abbot::balancer::controller::getAutotuneStatus();
    char buf[128]; snprintf(buf, sizeof(buf), "AUTOTUNE: %s", status);
    LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, buf);
  });
  bal->addEntry(11, "AUTOTUNE APPLY", [](const String&){ abbot::balancer::controller::applyAutotuneGains(); });
  root->addSubmenu(5, "Balancer (PID)", bal);
  root->addSubmenu(4, "Log channels", g_logMenu);

  // Commit to globals after fully-built to avoid races
  portENTER_CRITICAL(&g_menu_mux);
  g_rootMenu = root;
  g_building_menu = false;
  portEXIT_CRITICAL(&g_menu_mux);

  // Register capture-complete callback so the interactive menu can be re-shown
  // when an auto-capture finishes.
  abbot::tuning::setOnCaptureComplete(onCaptureCompleteRefreshMenu);
}

// Start the interactive menu programmatically (prints the menu and makes it active)
void startInteractiveMenu(abbot::BMI088Driver *driver) {
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
  if (up.startsWith("CALIB")) {
    // reuse imu_cal parsing by calling its start/reset/dump functions
    // Copy-paste of previous parsing logic kept minimal here for clarity
    char buf[128];
    up.toCharArray(buf, sizeof(buf));
    char *tk = strtok(buf, " \t\r\n");
    if (!tk) {
      return;
    }
    char *t2 = strtok(NULL, " \t\r\n");
    if (!t2) {
      return;
    }
    if (strcmp(t2, "START") == 0) {
      char *what = strtok(NULL, " \t\r\n");
      if (!what) {
        return;
      }
      int sampleCount = 2000;
      char *nstr = strtok(NULL, " \t\r\n");
      if (nstr) {
        int v = atoi(nstr);
        if (v > 0) {
          sampleCount = v;
        }
      }
      if (strcmp(what, "GYRO") == 0) {
        if (!abbot::imu_cal::isCalibrating()) {
          abbot::log::pushChannelMask(static_cast<uint32_t>(abbot::log::CHANNEL_DEFAULT) | static_cast<uint32_t>(abbot::log::CHANNEL_IMU));
          bool ok = abbot::imu_cal::startGyroCalibration(*driver, sampleCount);
          (void)ok;
          abbot::log::popChannelMask();
          // If interactive menu is active, re-show the current menu
          if (g_menuActive && g_currentMenu) {
            g_currentMenu->enter();
          }
        }
        return;
      } else if (strcmp(what, "ACCEL") == 0) {
        if (!abbot::imu_cal::isCalibrating()) {
          abbot::log::pushChannelMask(static_cast<uint32_t>(abbot::log::CHANNEL_DEFAULT) | static_cast<uint32_t>(abbot::log::CHANNEL_IMU));
          bool ok = abbot::imu_cal::startAccelCalibration(*driver, sampleCount);
          (void)ok;
          abbot::log::popChannelMask();
          // If interactive menu is active, re-show the current menu
          if (g_menuActive && g_currentMenu) {
            g_currentMenu->enter();
          }
        }
        return;
      }
    } else if (strcmp(t2, "DUMP") == 0) {
      abbot::imu_cal::dumpCalibration();
      return;
    } else if (strcmp(t2, "RESET") == 0) {
      abbot::imu_cal::resetCalibration();
      return;
    }
  }

  // HELP command: enter interactive menu (numeric selections)
  if (up == "HELP" || up == "?" || up.startsWith("HELP ")) {
    ensureMenus(driver);
    g_menuActive = true;
    g_currentMenu = g_rootMenu;
    g_currentMenu->enter();
    return;
  }

  // TUNING commands: start/stop CSV stream
  if (up.startsWith("TUNING")) {
    char buf2[32];
    up.toCharArray(buf2, sizeof(buf2));
    char *tk = strtok(buf2, " \t\r\n");
    char *arg = strtok(NULL, " \t\r\n");
    char *arg2 = strtok(NULL, " \t\r\n");
    if (arg && strcmp(arg, "START") == 0) {
      // Optional sample count after START
      if (arg2) {
        int n = atoi(arg2);
        if (n > 0) {
          abbot::log::pushChannelMask(static_cast<uint32_t>(abbot::log::CHANNEL_DEFAULT) | static_cast<uint32_t>(abbot::log::CHANNEL_TUNING));
          abbot::tuning::startCapture((uint32_t)n, true);
          LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "TUNING: auto-capture started");
          return;
        }
      }
      abbot::log::pushChannelMask(static_cast<uint32_t>(abbot::log::CHANNEL_DEFAULT) | static_cast<uint32_t>(abbot::log::CHANNEL_TUNING));
      abbot::startTuningStream();
      LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "TUNING: started");
      return;
    } else if (arg && strcmp(arg, "STOP") == 0) {
      abbot::stopTuningStream();
      abbot::log::popChannelMask();
      LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "TUNING: stopped");
      // If interactive menu is active, re-show the current menu
      if (g_menuActive && g_currentMenu) {
        g_currentMenu->enter();
      }
      return;
    } else if (arg && strcmp(arg, "START_STATS") == 0) {
      // Start capture in stats-only mode (periodic progress + final summary)
      int sampleCount = 2000;
      if (arg2) {
        int v = atoi(arg2);
        if (v > 0) sampleCount = v;
      }
      abbot::log::pushChannelMask(static_cast<uint32_t>(abbot::log::CHANNEL_DEFAULT) | static_cast<uint32_t>(abbot::log::CHANNEL_TUNING));
      abbot::tuning::startCapture((uint32_t)sampleCount, false, true);
      LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "TUNING: auto-capture (stats-only) started");
      return;
    } else {
      LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "TUNING usage: TUNING START | TUNING STOP");
      return;
    }
  }

  // LOG commands: enable/disable/list channels
  if (up.startsWith("LOG")) {
    char buf3[64];
    up.toCharArray(buf3, sizeof(buf3));
    char *tk = strtok(buf3, " \t\r\n");
    char *cmd = strtok(NULL, " \t\r\n");
    if (!cmd) {
      LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "LOG usage: LOG ENABLE|DISABLE <CHANNEL> | LOG LIST");
      return;
    }
    if (strcmp(cmd, "LIST") == 0) {
      char out[128];
      abbot::log::listEnabledChannels(out, sizeof(out));
      LOG_PRINTF(abbot::log::CHANNEL_DEFAULT, "LOG enabled: %s\n", out);
      return;
    }
    char *argch = strtok(NULL, " \t\r\n");
    if (!argch) {
      LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "LOG usage: LOG ENABLE|DISABLE <CHANNEL>");
      return;
    }
    auto ch = abbot::log::channelFromString(argch);
    if (ch == static_cast<abbot::log::Channel>(0)) {
      LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "Unknown channel. Known: TUNING,BLE,IMU,MOTOR,DEFAULT");
      return;
    }
    if (strcmp(cmd, "ENABLE") == 0) {
      abbot::log::enableChannel(ch);
      LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "LOG: enabled");
      return;
    } else if (strcmp(cmd, "DISABLE") == 0) {
      abbot::log::disableChannel(ch);
      LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "LOG: disabled");
      return;
    }
    LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "LOG usage: LOG ENABLE|DISABLE <CHANNEL> | LOG LIST");
    return;
  }

  // BALANCE commands: BALANCE START | BALANCE STOP | BALANCE GAINS <kp> <ki> <kd>
  //                 | BALANCE GET_GAINS
  //                 | BALANCE DEADBAND GET|SET|CALIBRATE
  if (up.startsWith("BALANCE")) {
    char buf4[64];
    up.toCharArray(buf4, sizeof(buf4));
    char *tk = strtok(buf4, " \t\r\n");
    char *arg = strtok(NULL, " \t\r\n");
    if (!arg) {
      LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "BALANCE usage: BALANCE START | BALANCE STOP | BALANCE GAINS <kp> <ki> <kd> | BALANCE GET_GAINS | BALANCE DEADBAND GET|SET|CALIBRATE");
      return;
    }
    if (strcmp(arg, "START") == 0) {
      abbot::startBalancer();
      return;
    } else if (strcmp(arg, "STOP") == 0) {
      abbot::stopBalancer();
      return;
    } else if (strcmp(arg, "GAINS") == 0) {
      // remaining tokens are kp ki kd
      char *kp_s = strtok(NULL, " \t\r\n");
      char *ki_s = strtok(NULL, " \t\r\n");
      char *kd_s = strtok(NULL, " \t\r\n");
      float kp = kp_s ? atof(kp_s) : 0.0f;
      float ki = ki_s ? atof(ki_s) : 0.0f;
      float kd = kd_s ? atof(kd_s) : 0.0f;
      abbot::setBalancerGains(kp, ki, kd);
      return;
    } else if (strcmp(arg, "GET_GAINS") == 0) {
      float kp,ki,kd; abbot::getBalancerGains(kp,ki,kd);
      char buf[128]; snprintf(buf, sizeof(buf), "BALANCER: Kp=%.6f Ki=%.6f Kd=%.6f", (double)kp, (double)ki, (double)kd);
      LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, buf);
      return;
    } else if (strcmp(arg, "DEADBAND") == 0) {
      char *sub = strtok(NULL, " \t\r\n");
      if (!sub) { LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "BALANCE DEADBAND usage: DEADBAND GET | DEADBAND SET <v> | DEADBAND CALIBRATE"); return; }
      if (strcmp(sub, "GET") == 0) {
        float db = abbot::balancer::controller::getDeadband();
        char buf[128]; snprintf(buf, sizeof(buf), "BALANCER: deadband=%.6f", (double)db);
        LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, buf);
        return;
      } else if (strcmp(sub, "SET") == 0) {
        char *val = strtok(NULL, " \t\r\n");
        if (!val) { LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "Usage: BALANCE DEADBAND SET <value>"); return; }
        float v = atof(val);
        abbot::balancer::controller::setDeadband(v);
        return;
      } else if (strcmp(sub, "CALIBRATE") == 0) {
        abbot::balancer::controller::calibrateDeadband();
        return;
      }
    }
    LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "BALANCE usage: BALANCE START | BALANCE STOP | BALANCE GAINS <kp> <ki> <kd>");
    return;
  }

  // AUTOTUNE commands: AUTOTUNE START | AUTOTUNE STOP | AUTOTUNE STATUS | AUTOTUNE APPLY | AUTOTUNE RELAY <amp> | AUTOTUNE DEADBAND <deg> | AUTOTUNE MAXANGLE <deg>
  if (up.startsWith("AUTOTUNE")) {
    char buf5[64];
    up.toCharArray(buf5, sizeof(buf5));
    char *tk = strtok(buf5, " \t\r\n");
    char *arg = strtok(NULL, " \t\r\n");
    if (!arg) {
      LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "AUTOTUNE usage: AUTOTUNE START | AUTOTUNE STOP | AUTOTUNE STATUS | AUTOTUNE APPLY | AUTOTUNE RELAY <amp> | AUTOTUNE DEADBAND <deg> | AUTOTUNE MAXANGLE <deg>");
      return;
    }
    if (strcmp(arg, "START") == 0) {
      abbot::balancer::controller::startAutotune();
      return;
    } else if (strcmp(arg, "STOP") == 0) {
      abbot::balancer::controller::stopAutotune();
      return;
    } else if (strcmp(arg, "STATUS") == 0) {
      const char* status = abbot::balancer::controller::getAutotuneStatus();
      char buf[128]; snprintf(buf, sizeof(buf), "AUTOTUNE: %s", status);
      LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, buf);
      return;
    } else if (strcmp(arg, "APPLY") == 0) {
      abbot::balancer::controller::applyAutotuneGains();
      return;
    } else if (strcmp(arg, "RELAY") == 0) {
      char *val = strtok(NULL, " \t\r\n");
      if (!val) { LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "Usage: AUTOTUNE RELAY <amplitude>"); return; }
      float amp = atof(val);
      abbot::balancer::controller::setAutotuneRelay(amp);
      return;
    } else if (strcmp(arg, "DEADBAND") == 0) {
      char *val = strtok(NULL, " \t\r\n");
      if (!val) { LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "Usage: AUTOTUNE DEADBAND <degrees>"); return; }
      float db = atof(val);
      abbot::balancer::controller::setAutotuneDeadband(db);
      return;
    } else if (strcmp(arg, "MAXANGLE") == 0) {
      char *val = strtok(NULL, " \t\r\n");
      if (!val) { LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "Usage: AUTOTUNE MAXANGLE <degrees>"); return; }
      float maxa = atof(val);
      abbot::balancer::controller::setAutotuneMaxAngle(maxa);
      return;
    }
    LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "AUTOTUNE usage: AUTOTUNE START | AUTOTUNE STOP | AUTOTUNE STATUS | AUTOTUNE APPLY | AUTOTUNE RELAY <amp> | AUTOTUNE DEADBAND <deg> | AUTOTUNE MAXANGLE <deg>");
    return;
  }


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
