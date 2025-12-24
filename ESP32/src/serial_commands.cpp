// serial_commands.cpp
#include "serial_commands.h"
#include "../config/motor_configs/servo_motor_config.h"
#include "../include/balancer_controller.h"
#include "BMI088Driver.h"
#include "SystemTasks.h"
#include "esp_wifi_console.h"
#include "filter_manager.h"
#include "imu_calibration.h"
#include "logging.h"
#include "motor_drivers/driver_manager.h"
#include "motor_drivers/critical_guard.h"
#include "serial_commands/motor_telemetry_manager.h"
#include "serial_menu.h"
#include "tuning_capture.h"
#include <Arduino.h>
#include <Preferences.h>
#include <WiFi.h>
#include <cstdio>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

namespace abbot {
namespace serialcmds {

// forward-declare startInteractiveMenu (implemented after ensureMenus)
void startInteractiveMenu(abbot::BMI088Driver *driver);

// Module-local copy of the driver pointer used by the serial task so other
// modules (wifi console) can inject lines without needing the driver handle.
static abbot::BMI088Driver *g_serial_task_driver = nullptr;

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
static SerialMenu *g_filterMenu = nullptr;
static SerialMenu *g_wifiMenu = nullptr;

// Forward-declare helper builders and ensureMenus
static SerialMenu *buildCalibrationMenu(abbot::BMI088Driver *driver);
static SerialMenu *buildMotorMenu();
static SerialMenu *buildTuningMenu();
static SerialMenu *buildLogMenu();
static SerialMenu *buildBalancerMenu();
static SerialMenu *buildFilterMenu();
static void ensureMenus(abbot::BMI088Driver *driver);
static void onCaptureCompleteRefreshMenu();

static bool handleFilter(const String &line, const String &up);

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
static void balancerStartHandler(const String &p);
static void autotuneRelayHandler(const String &p);
static void autotuneDeadbandHandler(const String &p);
static void autotuneMaxAngleHandler(const String &p);
static bool handleFusion(const String &line, const String &up);

static SerialMenu *buildCalibrationMenu(abbot::BMI088Driver *driver) {
  SerialMenu *m = new SerialMenu("Calibration Commands");
  m->addEntry(1, "CALIB START GYRO [N]",
              [driver](const String &p) { calibStartGyro(driver, p); });
  m->addEntry(2, "CALIB START ACCEL [N]",
              [driver](const String &p) { calibStartAccel(driver, p); });
  m->addEntry(3, "CALIB DUMP",
              [](const String &) { abbot::imu_cal::dumpCalibration(); });
  m->addEntry(4, "CALIB RESET",
              [](const String &) { abbot::imu_cal::resetCalibration(); });
  return m;
}

static SerialMenu *buildMotorMenu() {
  SerialMenu *m = new SerialMenu("Motor Commands");
  m->addEntry(1, "MOTOR ENABLE", [](const String &) {
    auto d = abbot::motor::getActiveMotorDriver();
    if (d) {
      d->enableMotors();
    } else {
      LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "No active motor driver");
    }
  });
  m->addEntry(2, "MOTOR DISABLE", [](const String &) {
    auto d = abbot::motor::getActiveMotorDriver();
    if (d) {
      d->disableMotors();
    } else {
      LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "No active motor driver");
    }
  });
  m->addEntry(3, "MOTOR STATUS", [](const String &) {
    auto d = abbot::motor::getActiveMotorDriver();
    if (d) {
      d->printStatus();
    } else {
      LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "No active motor driver");
    }
  });
  m->addEntry(4, "MOTOR DUMP", [](const String &) {
    auto d = abbot::motor::getActiveMotorDriver();
    if (d) {
      d->dumpConfig();
    } else {
      LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "No active motor driver");
    }
  });
  m->addEntry(5, "MOTOR RESETPOS", [](const String &) {
    auto d = abbot::motor::getActiveMotorDriver();
    if (d) {
      d->resetPositionTracking();
    } else {
      LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "No active motor driver");
    }
  });
  m->addEntry(6, "MOTOR POS", [](const String &) {
    auto d = abbot::motor::getActiveMotorDriver();
    if (d) {
      d->processSerialCommand("POS");
    } else {
      LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "No active motor driver");
    }
  });
  m->addEntry(7, "MOTOR VEL <LEFT|RIGHT> <speed>", [](const String &p) {
    String cmd = "VEL " + p;
    auto d = abbot::motor::getActiveMotorDriver();
    if (d) {
      d->processSerialCommand(cmd);
    } else {
      LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "No active motor driver");
    }
  });
  m->addEntry(8, "MOTOR SET <LEFT|RIGHT|ID> <v>",
              [](const String &p) { motorSetHandler(p); });
  m->addEntry(9, "MOTOR PARAMS <LEFT|RIGHT>", [](const String &p) {
    String cmd = "PARAMS " + p;
    auto d = abbot::motor::getActiveMotorDriver();
    if (d) {
      d->processSerialCommand(cmd);
    } else {
      LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "No active motor driver");
    }
  });
  m->addEntry(10, "MOTOR ACC <LEFT|RIGHT> <value>", [](const String &p) {
    String cmd = "ACC " + p;
    auto d = abbot::motor::getActiveMotorDriver();
    if (d) {
      d->processSerialCommand(cmd);
    } else {
      LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "No active motor driver");
    }
  });
  m->addEntry(11, "MOTOR INVERT <LEFT|RIGHT|ID> [0|1]", [](const String &p) {
    String cmd = "INVERT " + p;
    auto d = abbot::motor::getActiveMotorDriver();
    if (d) {
      d->processSerialCommand(cmd);
    } else {
      LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "No active motor driver");
    }
  });
  m->addEntry(12, "MOTOR GET ENCODER <LEFT|RIGHT|ID>", [](const String &p) {
    abbot::motor::EncoderReport rep;
    bool ok = abbot::motor::getEncoderReportFromArg(p.c_str(), rep);
    if (!ok) {
      LOG_PRINTLN(abbot::log::CHANNEL_MOTOR,
                  "Usage: MOTOR GET ENCODER <LEFT|RIGHT|ID>");
      return;
    }
    if (rep.both) {
      LOG_PRINTF(abbot::log::CHANNEL_MOTOR,
                 "MOTOR: encoder L(id=%d)=%ld R(id=%d)=%ld\n", rep.leftId,
                 (long)rep.leftVal, rep.rightId, (long)rep.rightVal);
    } else {
      if (rep.requestedId == rep.leftId) {
        LOG_PRINTF(abbot::log::CHANNEL_MOTOR,
                   "MOTOR: encoder id=%d value=%ld\n", rep.requestedId,
                   (long)rep.leftVal);
      } else if (rep.requestedId == rep.rightId) {
        LOG_PRINTF(abbot::log::CHANNEL_MOTOR,
                   "MOTOR: encoder id=%d value=%ld\n", rep.requestedId,
                   (long)rep.rightVal);
      } else {
        // Numeric id case: values are returned in leftVal/rightVal (same)
        LOG_PRINTF(abbot::log::CHANNEL_MOTOR,
                   "MOTOR: encoder id=%d value=%ld\n", rep.requestedId,
                   (long)rep.leftVal);
      }
    }
  });
  m->addEntry(13, "MOTOR SPEED <LEFT|RIGHT|ID>", [](const String &p) {
    abbot::motor::EncoderReport rep;
    bool ok = abbot::motor::getEncoderReportFromArg(p.c_str(), rep);
    if (!ok) {
      LOG_PRINTLN(abbot::log::CHANNEL_MOTOR,
                  "Usage: MOTOR SPEED <LEFT|RIGHT|ID>");
      return;
    }
    auto drv = abbot::motor::getActiveMotorDriver();
    if (!drv) {
      LOG_PRINTLN(abbot::log::CHANNEL_MOTOR, "No active motor driver");
      return;
    }
    if (rep.both) {
      float lsp = drv->readSpeed(abbot::motor::IMotorDriver::MotorSide::LEFT);
      float rsp = drv->readSpeed(abbot::motor::IMotorDriver::MotorSide::RIGHT);
      LOG_PRINTF(abbot::log::CHANNEL_MOTOR,
                 "MOTOR: speed L(id=%d)=%.2f R(id=%d)=%.2f\n", rep.leftId,
                 (double)lsp, rep.rightId, (double)rsp);
    } else {
      if (rep.requestedId == rep.leftId) {
        float lsp = drv->readSpeed(abbot::motor::IMotorDriver::MotorSide::LEFT);
        LOG_PRINTF(abbot::log::CHANNEL_MOTOR,
                   "MOTOR: speed id=%d value=%.2f\n", rep.requestedId,
                   (double)lsp);
      } else if (rep.requestedId == rep.rightId) {
        float rsp = drv->readSpeed(abbot::motor::IMotorDriver::MotorSide::RIGHT);
        LOG_PRINTF(abbot::log::CHANNEL_MOTOR,
                   "MOTOR: speed id=%d value=%.2f\n", rep.requestedId,
                   (double)rsp);
      } else {
        // numeric id case: map numeric id -> MotorSide and query by side
        abbot::motor::IMotorDriver::MotorSide side;
        if (abbot::motor::getSideForId(rep.requestedId, side)) {
          float s = abbot::motor::readSpeedBySide(side);
          LOG_PRINTF(abbot::log::CHANNEL_MOTOR,
                     "MOTOR: speed id=%d value=%.2f\n", rep.requestedId,
                     (double)s);
        } else {
          LOG_PRINTF(abbot::log::CHANNEL_MOTOR,
                     "MOTOR: speed id=%d unknown\n", rep.requestedId);
        }
      }
    }
  });
  m->addEntry(14, "MOTOR TELEMETRY <LEFT|RIGHT|ALL> <ms> (0 to stop)", 
              [](const String &p) {
    String args = p;
    args.trim();
    if (args.length() == 0) {
      LOG_PRINTLN(abbot::log::CHANNEL_MOTOR,
                  "Usage: MOTOR TELEMETRY <LEFT|RIGHT|ALL> <ms> (0 to stop)");
      return;
    }
    int spaceIndex = args.indexOf(' ');
    String sideToken;
    String msToken;
    if (spaceIndex < 0) {
      // missing ms
      LOG_PRINTLN(abbot::log::CHANNEL_MOTOR,
                  "Usage: MOTOR TELEMETRY <LEFT|RIGHT|ALL> <ms> (0 to stop)");
      return;
    } else {
      sideToken = args.substring(0, spaceIndex);
      msToken = args.substring(spaceIndex + 1);
      sideToken.trim();
      msToken.trim();
    }
    int ms = msToken.toInt();
    if (ms < 0) {
      LOG_PRINTLN(abbot::log::CHANNEL_MOTOR, "Invalid interval");
      return;
    }
    bool reportBoth = false;
    bool leftSelected = true;
    String up = sideToken;
    up.toUpperCase();
    if (up == "ALL") {
      reportBoth = true;
    } else if (up == "LEFT") {
      reportBoth = false;
      leftSelected = true;
    } else if (up == "RIGHT") {
      reportBoth = false;
      leftSelected = false;
    } else {
      LOG_PRINTLN(abbot::log::CHANNEL_MOTOR,
                  "Usage: MOTOR TELEMETRY <LEFT|RIGHT|ALL> <ms> (0 to stop)");
      return;
    }
    if (ms == 0) {
      g_telemetryManager.stop();
      LOG_PRINTLN(abbot::log::CHANNEL_MOTOR, "MOTOR: telemetry stopped");
      return;
    }
    // configure single-side selection
    // access private field via start(): we cheat by stopping then starting
    // with desired mode. The manager defaults singleLeft=true; to support
    // RIGHT we restart and flip the internal flag via a simple start for
    // now.
    g_telemetryManager.stop();
    g_telemetryManager.start(reportBoth, ms, leftSelected);
    LOG_PRINTF(abbot::log::CHANNEL_MOTOR,
           "MOTOR: telemetry started mode=%s interval=%dms\n",
           (reportBoth ? "ALL" : (leftSelected ? "LEFT" : "RIGHT")), ms);
  });
  return m;
}



static SerialMenu *buildTuningMenu() {
  SerialMenu *m = new SerialMenu("Tuning (Madgwick)");
  m->addEntry(1, "TUNING STATIC (2000)", [](const String &) {
    abbot::log::pushChannelMask(
        static_cast<uint32_t>(abbot::log::CHANNEL_DEFAULT) |
        static_cast<uint32_t>(abbot::log::CHANNEL_TUNING));
    abbot::tuning::startCapture(2000, true);
  });
  m->addEntry(2, "TUNING DYNAMIC (20000)", [](const String &) {
    abbot::log::pushChannelMask(
        static_cast<uint32_t>(abbot::log::CHANNEL_DEFAULT) |
        static_cast<uint32_t>(abbot::log::CHANNEL_TUNING));
    abbot::tuning::startCapture(20000, true);
  });
  m->addEntry(3, "TUNING CUSTOM <N>",
              [](const String &p) { tuningCustomHandler(p); });
  m->addEntry(6, "TUNING STATIC STATS (2000)", [](const String &) {
    abbot::log::pushChannelMask(
        static_cast<uint32_t>(abbot::log::CHANNEL_DEFAULT) |
        static_cast<uint32_t>(abbot::log::CHANNEL_TUNING));
    abbot::tuning::startCapture(2000, false, true);
  });
  m->addEntry(7, "TUNING DYNAMIC STATS (20000)", [](const String &) {
    abbot::log::pushChannelMask(
        static_cast<uint32_t>(abbot::log::CHANNEL_DEFAULT) |
        static_cast<uint32_t>(abbot::log::CHANNEL_TUNING));
    abbot::tuning::startCapture(20000, false, true);
  });
  m->addEntry(8, "TUNING CUSTOM STATS <N>", [](const String &p) {
    String s = p;
    s.trim();
    if (s.length() == 0) {
      LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT,
                  "Usage: 8 <N> (number of samples)");
      return;
    }
    int n = s.toInt();
    if (n <= 0) {
      LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "Invalid sample count");
      return;
    }
    abbot::log::pushChannelMask(
        static_cast<uint32_t>(abbot::log::CHANNEL_DEFAULT) |
        static_cast<uint32_t>(abbot::log::CHANNEL_TUNING));
    abbot::tuning::startCapture((uint32_t)n, false, true);
  });
  m->addEntry(4, "TUNING STREAM START",
              [](const String &) { tuningStreamStartHandler(String()); });
  m->addEntry(5, "TUNING STREAM STOP",
              [](const String &) { tuningStreamStopHandler(String()); });
  return m;
}

static SerialMenu *buildLogMenu() {
  SerialMenu *m = new SerialMenu("Log Channels");
  m->setOnEnter([]() { logMenuOnEnter(); });
  return m;
}

static SerialMenu *buildFilterMenu() {
  SerialMenu *m = new SerialMenu("Filter Selection");
  // Use onEnter to rebuild dynamic entries each time the menu is shown so
  // the "(current)" marker always reflects the latest selection.
  m->setOnEnter([m]() {
    // clear and rebuild entries
    m->clearEntries();
    // show current
    m->addEntry(1, "FILTER STATUS", [](const String &) {
      char buf[128];
      snprintf(buf, sizeof(buf), "Current filter: %s",
               abbot::filter::getCurrentFilterName());
      LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, buf);
    });
    // expose ALPHA tuning via menu (shows current value and usage)
    m->addEntry(
        2, "SET ALPHA (usage: FILTER SET ALPHA <v>)", [](const String &) {
          auto a = abbot::filter::getActiveFilter();
          if (a) {
            float val = 0.0f;
            if (a->getParam("ALPHA", val)) {
              char out[128];
              snprintf(out, sizeof(out),
                       "FILTER: ALPHA=%.4f (use: FILTER SET ALPHA <value>)",
                       (double)val);
              LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, out);
            } else {
              LOG_PRINTLN(
                  abbot::log::CHANNEL_DEFAULT,
                  "FILTER: current filter does not support ALPHA parameter");
            }
          } else {
            LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT,
                        "FILTER: no active filter");
          }
          if (g_filterMenu) {
            g_menuActive = true;
            g_currentMenu = g_filterMenu;
            g_filterMenu->enter();
          }
        });
    // dynamic entries for available filters
    int count = abbot::filter::getAvailableFilterCount();
    int id = 3;
    const char *cur = abbot::filter::getCurrentFilterName();
    for (int i = 0; i < count; ++i) {
      const char *name = abbot::filter::getAvailableFilterName(i);
      char lbl[80];
      if (cur && strcmp(name, cur) == 0) {
        snprintf(lbl, sizeof(lbl), "SELECT %s (current)", name);
        // If user selects the already-active filter, just inform them rather
        // than re-selecting
        m->addEntry(id++, lbl, [name](const String &) {
          char b[128];
          snprintf(b, sizeof(b), "FILTER: %s already active", name);
          LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, b);
          if (g_filterMenu) {
            g_menuActive = true;
            g_currentMenu = g_filterMenu;
            g_filterMenu->enter();
          }
        });
      } else {
        snprintf(lbl, sizeof(lbl), "SELECT %s", name);
        m->addEntry(id++, lbl, [name](const String &) {
          bool ok = abbot::filter::setCurrentFilterByName(name);
          if (ok) {
            auto a = abbot::filter::getActiveFilter();
            if (a) {
              unsigned long ms = a->getWarmupDurationMs();
              if (ms > 0) {
                float s = ((float)ms) / 1000.0f;
                requestTuningWarmupSeconds(s);
                char tbuf[128];
                snprintf(tbuf, sizeof(tbuf),
                         "FILTER: requested warmup %.3f s for %s", (double)s,
                         name);
                LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, tbuf);
              }
            }
          }
          if (g_filterMenu) {
            g_menuActive = true;
            g_currentMenu = g_filterMenu;
            g_filterMenu->enter();
          }
        });
      }
    }
  });
  return m;
}

static SerialMenu *buildWifiMenu() {
  SerialMenu *m = new SerialMenu("WiFi Configuration");
  m->addEntry(1, "WIFI SHOW", [](const String &) {
    Preferences p;
    if (p.begin("abbot", true)) {
      String ssid = p.getString("wifi_ssid", "");
      bool has = ssid.length() > 0;
      char out[128];
      snprintf(out, sizeof(out), "WIFI: stored_ssid=%s",
               has ? ssid.c_str() : "(none)");
      LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, out);
      p.end();
    }
  });
  m->addEntry(2, "WIFI SET SSID <ssid>", [](const String &p) {
    String s = p;
    s.trim();
    if (s.length() == 0) {
      LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "Usage: WIFI SET SSID <ssid>");
      return;
    }
    Preferences pref;
    if (pref.begin("abbot", false)) {
      pref.putString("wifi_ssid", s);
      pref.end();
      LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "WIFI: SSID saved");
    }
  });
  m->addEntry(3, "WIFI SET PASS <pwd>", [](const String &p) {
    String s = p;
    s.trim();
    if (s.length() == 0) {
      LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT,
                  "Usage: WIFI SET PASS <password>");
      return;
    }
    Preferences pref;
    if (pref.begin("abbot", false)) {
      pref.putString("wifi_pass", s);
      pref.end();
      LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "WIFI: password saved (hidden)");
    }
  });
  m->addEntry(4, "WIFI CONNECT NOW", [](const String &) {
    abbot::wifi_console::connectNow();
    LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "WIFI: connect requested");
  });
  m->addEntry(5, "WIFI DISCONNECT", [](const String &) {
    abbot::wifi_console::disconnectNow();
    LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "WIFI: disconnect requested");
  });
  m->addEntry(6, "WIFI RESET", [](const String &) {
    Preferences pref;
    if (pref.begin("abbot", false)) {
      pref.remove("wifi_ssid");
      pref.remove("wifi_pass");
      pref.end();
      LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT,
                  "WIFI: cleared stored credentials");
    }
  });
  m->addEntry(7, "WIFI STATUS", [](const String &) {
    if (WiFi.status() == WL_CONNECTED) {
      IPAddress ip = WiFi.localIP();
      char buf[128];
      char ipbuf[32];
      ip.toString().toCharArray(ipbuf, sizeof(ipbuf));
      snprintf(buf, sizeof(buf), "WIFI: connected ssid=%s ip=%s",
               WiFi.SSID().c_str(), ipbuf);
      LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, buf);
    } else {
      LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "WIFI: not connected");
    }
  });
  m->addEntry(8, "WIFI DIAG", [](const String &) {
    char d[256];
    abbot::wifi_console::getDiagnostics(d, sizeof(d));
    char out[300];
    snprintf(out, sizeof(out), "WIFI-DIAG: %s", d);
    LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, out);
  });
  return m;
}

static SerialMenu *buildBalancerMenu() {
  SerialMenu *bal = new SerialMenu("Balancer (PID)");
  bal->addEntry(1, "BALANCE START",
                [](const String &p) { balancerStartHandler(p); });
  bal->addEntry(2, "BALANCE STOP",
                [](const String &) { abbot::balancer::controller::stop(); });
  bal->addEntry(3, "BALANCE GET_GAINS", [](const String &) {
    float kp, ki, kd;
    abbot::balancer::controller::getGains(kp, ki, kd);
    char buf[128];
    snprintf(buf, sizeof(buf), "BALANCER: Kp=%.6f Ki=%.6f Kd=%.6f", (double)kp,
             (double)ki, (double)kd);
    LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, buf);
  });
  // Group: gains
  bal->addEntry(4, "BALANCE SET GAINS <kp> <ki> <kd>",
                [](const String &p) { balancerSetGainsHandler(p); });
  bal->addEntry(5, "BALANCE RESET GAINS", [](const String &) {
    abbot::balancer::controller::resetGainsToDefaults();
  });
  // Group: deadband
  bal->addEntry(6, "BALANCE DEADBAND GET", [](const String &) {
    float db = abbot::balancer::controller::getDeadband();
    char buf[128];
    snprintf(buf, sizeof(buf), "BALANCER: deadband=%.6f", (double)db);
    LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, buf);
  });
  bal->addEntry(7, "BALANCE DEADBAND SET <v>",
                [](const String &p) { balancerDeadbandSetHandler(p); });
  bal->addEntry(8, "BALANCE DEADBAND CALIBRATE", [](const String &) {
    abbot::balancer::controller::calibrateDeadband();
  });
  // Group: autotune
  bal->addEntry(9, "AUTOTUNE START", [](const String &) {
    abbot::balancer::controller::startAutotune();
  });
  bal->addEntry(10, "AUTOTUNE STOP", [](const String &) {
    abbot::balancer::controller::stopAutotune();
  });
  bal->addEntry(11, "AUTOTUNE STATUS", [](const String &) {
    const char *status = abbot::balancer::controller::getAutotuneStatus();
    char buf[128];
    snprintf(buf, sizeof(buf), "AUTOTUNE: %s", status);
    LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, buf);
  });
  bal->addEntry(12, "AUTOTUNE APPLY", [](const String &) {
    abbot::balancer::controller::applyAutotuneGains();
  });
  bal->addEntry(13, "AUTOTUNE RELAY <amp>",
                [](const String &p) { autotuneRelayHandler(p); });
  bal->addEntry(14, "AUTOTUNE DEADBAND <deg>",
                [](const String &p) { autotuneDeadbandHandler(p); });
  bal->addEntry(15, "AUTOTUNE MAXANGLE <deg>",
                [](const String &p) { autotuneMaxAngleHandler(p); });
  return bal;
}

static void onCaptureCompleteRefreshMenu() {
  if (g_menuActive && g_currentMenu) {
    g_currentMenu->enter();
  }
}

static void ensureMenus(abbot::BMI088Driver *driver) {
  // Avoid rebuilding concurrently
  if (g_rootMenu || g_building_menu)
    return;
  g_building_menu = true;

  // Build root locally and only assign to global after fully populated
  SerialMenu *root = new SerialMenu("Main Menu");
  g_calibMenu = buildCalibrationMenu(driver);
  g_motorMenu = buildMotorMenu();
  g_tuningMenu = buildTuningMenu();
  g_logMenu = buildLogMenu();
  g_wifiMenu = buildWifiMenu();
  SerialMenu *bal = buildBalancerMenu();

  // Root menu entries link to submenus
  root->addSubmenu(1, "Calibration (BMI088)", g_calibMenu);
  root->addSubmenu(2, "Motor control", g_motorMenu);
  root->addSubmenu(3, "Madgwick tuning", g_tuningMenu);
  root->addSubmenu(4, "Log channels", g_logMenu);
  root->addSubmenu(5, "Balancer (PID)", bal);
  // Filter selection
  g_filterMenu = buildFilterMenu();
  root->addSubmenu(6, "Filter Selection", g_filterMenu);
  root->addSubmenu(7, "WiFi Configuration", g_wifiMenu);

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
static int tokenizeUpper(const String &src, char *buf, size_t bufsize,
                         char *tokens[], int maxTokens) {
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
  abbot::log::pushChannelMask(
      static_cast<uint32_t>(abbot::log::CHANNEL_DEFAULT) |
      static_cast<uint32_t>(abbot::log::CHANNEL_TUNING));
}

static void startTuningStream() {
  pushTuningMask();
  abbot::log::enableChannel(abbot::log::CHANNEL_TUNING);
  LOG_PRINTLN(abbot::log::CHANNEL_TUNING,
              "timestamp_ms,pitch_deg,pitch_rad,pitch_rate_deg,pitch_rate_rad,"
              "ax,ay,az,gx,gy,gz,temp_C,left_cmd,right_cmd");
  LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "TUNING: started");
}

static void stopTuningStream() {
  abbot::log::disableChannel(abbot::log::CHANNEL_TUNING);
  abbot::log::popChannelMask();
  LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "TUNING: stopped");
  if (g_menuActive && g_currentMenu)
    g_currentMenu->enter();
}

static void startTuningCapture(uint32_t samples, bool csv,
                               bool statsOnly = false) {
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
  if (p.length())
    n = p.toInt();
  if (!abbot::imu_cal::isCalibrating()) {
    abbot::log::pushChannelMask(
        static_cast<uint32_t>(abbot::log::CHANNEL_DEFAULT) |
        static_cast<uint32_t>(abbot::log::CHANNEL_IMU));
    bool ok = abbot::imu_cal::startGyroCalibration(*driver, n);
    (void)ok;
    abbot::log::popChannelMask();
    if (g_menuActive && g_currentMenu)
      g_currentMenu->enter();
  }
}

static void calibStartAccel(abbot::BMI088Driver *driver, const String &p) {
  int n = 2000;
  if (p.length())
    n = p.toInt();
  if (!abbot::imu_cal::isCalibrating()) {
    abbot::log::pushChannelMask(
        static_cast<uint32_t>(abbot::log::CHANNEL_DEFAULT) |
        static_cast<uint32_t>(abbot::log::CHANNEL_IMU));
    bool ok = abbot::imu_cal::startAccelCalibration(*driver, n);
    (void)ok;
    abbot::log::popChannelMask();
    if (g_menuActive && g_currentMenu)
      g_currentMenu->enter();
  }
}

static void motorSetHandler(const String &p) {
  String s = p;
  s.trim();
  int sp = s.indexOf(' ');
  if (sp == -1) {
    LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "Usage: <side> <value>");
    return;
  }
  String side = s.substring(0, sp);
  String val = s.substring(sp + 1);
  side.toUpperCase();
  if (side == "LEFT") {
    if (auto d = abbot::motor::getActiveMotorDriver())
      d->setMotorCommand(abbot::motor::IMotorDriver::MotorSide::LEFT,
                         val.toFloat());
    else
      LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "No active motor driver");
  } else if (side == "RIGHT") {
    if (auto d = abbot::motor::getActiveMotorDriver())
      d->setMotorCommand(abbot::motor::IMotorDriver::MotorSide::RIGHT,
                         val.toFloat());
    else
      LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "No active motor driver");
  } else {
    int id = side.toInt();
    if (auto d = abbot::motor::getActiveMotorDriver()) {
      if (id == LEFT_MOTOR_ID)
        d->setMotorCommandRaw(abbot::motor::IMotorDriver::MotorSide::LEFT,
                              (int16_t)val.toInt());
      else if (id == RIGHT_MOTOR_ID)
        d->setMotorCommandRaw(abbot::motor::IMotorDriver::MotorSide::RIGHT,
                              (int16_t)val.toInt());
      else
        LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "Unknown motor id");
    } else
      LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "No active motor driver");
  }
}

static void tuningCustomHandler(const String &p) {
  String s = p;
  s.trim();
  if (s.length() == 0) {
    LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "Usage: <N> (number of samples)");
    return;
  }
  int n = s.toInt();
  if (n <= 0) {
    LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "Invalid sample count");
    return;
  }
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
  if (!g_logMenu)
    return;
  g_logMenu->clearEntries();
  struct C {
    abbot::log::Channel ch;
    const char *name;
  } channels[] = {
      {abbot::log::CHANNEL_TUNING, "TUNING"},
      {abbot::log::CHANNEL_BLE, "BLE"},
      {abbot::log::CHANNEL_IMU, "IMU"},
      {abbot::log::CHANNEL_MOTOR, "MOTOR"},
      {abbot::log::CHANNEL_BALANCER, "BALANCER"},
      {abbot::log::CHANNEL_DEFAULT, "DEFAULT"},
  };
  int id = 1;
  for (auto &c : channels) {
    const char *name = abbot::log::channelName(c.ch);
    bool on = abbot::log::isChannelEnabled(c.ch);
    char lbl[64];
    snprintf(lbl, sizeof(lbl), "%s: %s", name, on ? "ON" : "OFF");
    abbot::log::Channel chcopy = c.ch;
    g_logMenu->addEntry(id++, lbl, [chcopy](const String &) {
      bool newState = abbot::log::toggleChannel(chcopy);
      char msg[64];
      snprintf(msg, sizeof(msg), "%s %s", abbot::log::channelName(chcopy),
               newState ? "enabled" : "disabled");
      LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, msg);
      if (g_logMenu) {
        g_logMenu->enter();
      }
    });
  }
}

static void balancerSetGainsHandler(const String &p) {
  String s = p;
  s.trim();
  float kp = 0, ki = 0, kd = 0;
  int matched = sscanf(s.c_str(), "%f %f %f", &kp, &ki, &kd);
  (void)matched;
  abbot::balancer::controller::setGains(kp, ki, kd);
}

static void balancerStartHandler(const String &p) {
  // p may contain optional FORCE token (case-insensitive)
  String s = p;
  s.trim();
  s.toUpperCase();
  bool force = false;
  if (s.length() > 0) {
    if (s.indexOf("FORCE") != -1)
      force = true;
  }
  // Print diagnostics for operator visibility
  abbot::printMadgwickDiagnostics();
  if (!force && !abbot::isFusionReady()) {
    unsigned long rem = abbot::getFusionWarmupRemaining();
    char buf[256];
    snprintf(buf, sizeof(buf),
             "BALANCE: fusion not ready (warmup remaining=%lu). Use 'BALANCE "
             "START FORCE' to override.",
             rem);
    LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, buf);
    return;
  }
  // Start balancer using current fused pitch
  abbot::balancer::controller::start(abbot::getFusedPitch());
}

static void balancerTuningStartHandler(const String &p) {
  String s = p;
  s.trim();
  int samples = 2000;
  if (s.length() > 0)
    samples = s.toInt();
  if (samples < 100)
    samples = 100;
  startTuningCapture((uint32_t)samples, true);
}

static void balancerTuningStopHandler(const String & /*unused*/) {
  abbot::tuning::stopCapture();
}

static void balancerDeadbandSetHandler(const String &p) {
  String s = p;
  s.trim();
  if (s.length() == 0) {
    LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT,
                "Usage: BALANCE DEADBAND SET <value>");
    return;
  }
  float v = s.toFloat();
  abbot::balancer::controller::setDeadband(v);
}

static void autotuneRelayHandler(const String &p) {
  String s = p;
  s.trim();
  if (s.length() == 0) {
    LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT,
                "Usage: AUTOTUNE RELAY <amplitude>");
    return;
  }
  float amp = s.toFloat();
  abbot::balancer::controller::setAutotuneRelay(amp);
}

static void autotuneDeadbandHandler(const String &p) {
  String s = p;
  s.trim();
  if (s.length() == 0) {
    LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT,
                "Usage: AUTOTUNE DEADBAND <degrees>");
    return;
  }
  float db = s.toFloat();
  abbot::balancer::controller::setAutotuneDeadband(db);
}

static void autotuneMaxAngleHandler(const String &p) {
  String s = p;
  s.trim();
  if (s.length() == 0) {
    LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT,
                "Usage: AUTOTUNE MAXANGLE <degrees>");
    return;
  }
  float maxa = s.toFloat();
  abbot::balancer::controller::setAutotuneMaxAngle(maxa);
}

// Command handlers: return true if the command was handled
static bool handleCalib(abbot::BMI088Driver *driver, const String &line,
                        const String &up) {
  if (!up.startsWith("CALIB"))
    return false;
  char buf[128];
  char *tok[8];
  int nt = tokenizeUpper(up, buf, sizeof(buf), tok, 8);
  if (nt < 2)
    return true;
  char *t2 = tok[1];
  if (strcmp(t2, "START") == 0) {
    char *what = (nt >= 3) ? tok[2] : nullptr;
    if (!what)
      return true;
    int sampleCount = 2000;
    if (nt >= 4) {
      int v = atoi(tok[3]);
      if (v > 0)
        sampleCount = v;
    }
    if (strcmp(what, "GYRO") == 0) {
      if (!abbot::imu_cal::isCalibrating()) {
        abbot::log::pushChannelMask(
            static_cast<uint32_t>(abbot::log::CHANNEL_DEFAULT) |
            static_cast<uint32_t>(abbot::log::CHANNEL_IMU));
        bool ok = abbot::imu_cal::startGyroCalibration(*driver, sampleCount);
        (void)ok;
        abbot::log::popChannelMask();
        if (g_menuActive && g_currentMenu)
          g_currentMenu->enter();
      }
      return true;
    } else if (strcmp(what, "ACCEL") == 0) {
      if (!abbot::imu_cal::isCalibrating()) {
        abbot::log::pushChannelMask(
            static_cast<uint32_t>(abbot::log::CHANNEL_DEFAULT) |
            static_cast<uint32_t>(abbot::log::CHANNEL_IMU));
        bool ok = abbot::imu_cal::startAccelCalibration(*driver, sampleCount);
        (void)ok;
        abbot::log::popChannelMask();
        if (g_menuActive && g_currentMenu)
          g_currentMenu->enter();
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

static bool handleTuning(const String &line, const String &up) {
  if (!up.startsWith("TUNING"))
    return false;
  char buf2[64];
  char *tok[4];
  int nt = tokenizeUpper(up, buf2, sizeof(buf2), tok, 4);
  char *arg = (nt >= 2) ? tok[1] : nullptr;
  char *arg2 = (nt >= 3) ? tok[2] : nullptr;
  if (arg && strcmp(arg, "START") == 0) {
    if (arg2) {
      int n = atoi(arg2);
      if (n > 0) {
        startTuningCapture((uint32_t)n, true);
        LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT,
                    "TUNING: auto-capture started");
        return true;
      }
    }
    startTuningStream();
    return true;
  } else if (arg && strcmp(arg, "STOP") == 0) {
    stopTuningStream();
    return true;
  } else if (arg && strcmp(arg, "START_STATS") == 0) {
    int sampleCount = 2000;
    if (arg2) {
      int v = atoi(arg2);
      if (v > 0)
        sampleCount = v;
    }
    startTuningCapture((uint32_t)sampleCount, false, true);
    LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT,
                "TUNING: auto-capture (stats-only) started");
    return true;
  } else {
    LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT,
                "TUNING usage: TUNING START | TUNING STOP");
    return true;
  }
  return true;
}

static bool handleLog(const String &line, const String &up) {
  if (!up.startsWith("LOG"))
    return false;
  char buf3[128];
  char *tok[4];
  int nt = tokenizeUpper(up, buf3, sizeof(buf3), tok, 4);
  char *cmd = (nt >= 2) ? tok[1] : nullptr;
  if (!cmd) {
    LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT,
                "LOG usage: LOG ENABLE|DISABLE <CHANNEL> | LOG LIST");
    return true;
  }
  if (strcmp(cmd, "LIST") == 0) {
    char out[128];
    abbot::log::listEnabledChannels(out, sizeof(out));
    LOG_PRINTF(abbot::log::CHANNEL_DEFAULT, "LOG enabled: %s\n", out);
    return true;
  }
  char *argch = (nt >= 3) ? tok[2] : nullptr;
  if (!argch) {
    LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT,
                "LOG usage: LOG ENABLE|DISABLE <CHANNEL>");
    return true;
  }
  auto ch = abbot::log::channelFromString(argch);
  if (ch == static_cast<abbot::log::Channel>(0)) {
    LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT,
                "Unknown channel. Known: TUNING,BLE,IMU,MOTOR,DEFAULT");
    return true;
  }
  if (strcmp(cmd, "ENABLE") == 0) {
    abbot::log::enableChannel(ch);
    LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "LOG: enabled");
    return true;
  } else if (strcmp(cmd, "DISABLE") == 0) {
    abbot::log::disableChannel(ch);
    LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "LOG: disabled");
    return true;
  }
  LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT,
              "LOG usage: LOG ENABLE|DISABLE <CHANNEL> | LOG LIST");
  return true;
}

static bool handleBalance(const String &line, const String &up) {
  if (!up.startsWith("BALANCE"))
    return false;
  char buf4[128];
  char *tok[8];
  int nt = tokenizeUpper(up, buf4, sizeof(buf4), tok, 8);
  if (nt < 2) {
    LOG_PRINTLN(
        abbot::log::CHANNEL_DEFAULT,
        "BALANCE usage: BALANCE START | BALANCE STOP | BALANCE GAINS [<kp> "
        "<ki> <kd>] (persisted for current FILTER) | BALANCE RESET | BALANCE "
        "GET_GAINS | BALANCE DEADBAND GET|SET|CALIBRATE");
    return true;
  }
  char *arg = tok[1];
  if (strcmp(arg, "START") == 0) {
    abbot::printMadgwickDiagnostics();
    abbot::balancer::controller::start(abbot::getFusedPitch());
    return true;
  }
  if (strcmp(arg, "START") == 0) {
    bool force = (nt >= 3 && strcmp(tok[2], "FORCE") == 0);
    abbot::printMadgwickDiagnostics();
    if (!force && !abbot::isFusionReady()) {
      unsigned long rem = abbot::getFusionWarmupRemaining();
      char buf[256];
      snprintf(buf, sizeof(buf),
               "BALANCE: fusion not ready (warmup remaining=%lu). Use 'BALANCE "
               "START FORCE' to override.",
               rem);
      LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, buf);
      return true;
    }
    abbot::balancer::controller::start(abbot::getFusedPitch());
    return true;
  } else if (strcmp(arg, "STOP") == 0) {
    abbot::balancer::controller::stop();
    return true;
  } else if (strcmp(arg, "RESET") == 0) {
    abbot::balancer::controller::resetGainsToDefaults();
    return true;
  } else if (strcmp(arg, "GAINS") == 0) {
    if (nt < 5) {
      float kp, ki, kd;
      abbot::balancer::controller::getGains(kp, ki, kd);
      char buf[128];
      snprintf(buf, sizeof(buf),
               "BALANCER: Kp=%.6f Ki=%.6f Kd=%.6f (persisted for FILTER=%s)",
               (double)kp, (double)ki, (double)kd,
               abbot::filter::getCurrentFilterName());
      LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, buf);
      return true;
    }
    float kp = atof(tok[2]);
    float ki = atof(tok[3]);
    float kd = atof(tok[4]);
    abbot::balancer::controller::setGains(kp, ki, kd);
    // Inform user where the gains are persisted
    char msg[128];
    snprintf(msg, sizeof(msg), "BALANCE: gains saved for FILTER=%s",
             abbot::filter::getCurrentFilterName());
    LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, msg);
    return true;
  } else if (strcmp(arg, "GET_GAINS") == 0) {
    float kp, ki, kd;
    abbot::balancer::controller::getGains(kp, ki, kd);
    char buf[128];
    snprintf(buf, sizeof(buf), "BALANCER: Kp=%.6f Ki=%.6f Kd=%.6f", (double)kp,
             (double)ki, (double)kd);
    LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, buf);
    return true;
  } else if (strcmp(arg, "DEADBAND") == 0) {
    if (nt < 3) {
      LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT,
                  "BALANCE DEADBAND usage: DEADBAND GET | DEADBAND SET <v> | "
                  "DEADBAND CALIBRATE");
      return true;
    }
    char *sub = tok[2];
    if (strcmp(sub, "GET") == 0) {
      float db = abbot::balancer::controller::getDeadband();
      char buf[128];
      snprintf(buf, sizeof(buf), "BALANCER: deadband=%.6f", (double)db);
      LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, buf);
      return true;
    } else if (strcmp(sub, "SET") == 0) {
      if (nt < 4) {
        LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT,
                    "Usage: BALANCE DEADBAND SET <value>");
        return true;
      }
      float v = atof(tok[3]);
      abbot::balancer::controller::setDeadband(v);
      return true;
    } else if (strcmp(sub, "CALIBRATE") == 0) {
      abbot::balancer::controller::calibrateDeadband();
      return true;
    }
  }
  LOG_PRINTLN(
      abbot::log::CHANNEL_DEFAULT,
      "BALANCE usage: BALANCE START | BALANCE STOP | BALANCE GAINS [<kp> <ki> "
      "<kd>] | BALANCE RESET | BALANCE DEADBAND GET|SET|CALIBRATE");
  return true;
}

static bool handleAutotune(const String &line, const String &up) {
  if (!up.startsWith("AUTOTUNE"))
    return false;
  char buf5[128];
  char *tok[6];
  int nt = tokenizeUpper(up, buf5, sizeof(buf5), tok, 6);
  if (nt < 2) {
    LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT,
                "AUTOTUNE usage: AUTOTUNE START | AUTOTUNE STOP | AUTOTUNE "
                "STATUS | AUTOTUNE APPLY | AUTOTUNE RELAY <amp> | AUTOTUNE "
                "DEADBAND <deg> | AUTOTUNE MAXANGLE <deg>");
    return true;
  }
  char *arg = tok[1];
  if (strcmp(arg, "START") == 0) {
    abbot::balancer::controller::startAutotune();
    return true;
  } else if (strcmp(arg, "SAFE") == 0) {
    abbot::balancer::controller::setAutotuneRelay(0.15f);
    abbot::balancer::controller::setAutotuneDeadband(0.5f);
    abbot::balancer::controller::setAutotuneMaxAngle(8.0f);
    LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT,
                "AUTOTUNE: starting in SAFE mode (relay=0.15 deadband=0.5° "
                "maxangle=8°)");
    abbot::balancer::controller::startAutotune();
    return true;
  } else if (strcmp(arg, "STOP") == 0) {
    abbot::balancer::controller::stopAutotune();
    return true;
  } else if (strcmp(arg, "STATUS") == 0) {
    const char *status = abbot::balancer::controller::getAutotuneStatus();
    char buf[128];
    snprintf(buf, sizeof(buf), "AUTOTUNE: %s", status);
    LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, buf);
    return true;
  } else if (strcmp(arg, "APPLY") == 0) {
    abbot::balancer::controller::applyAutotuneGains();
    return true;
  } else if (strcmp(arg, "RELAY") == 0) {
    if (nt < 3) {
      LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT,
                  "Usage: AUTOTUNE RELAY <amplitude>");
      return true;
    }
    float amp = atof(tok[2]);
    abbot::balancer::controller::setAutotuneRelay(amp);
    return true;
  } else if (strcmp(arg, "DEADBAND") == 0) {
    if (nt < 3) {
      LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT,
                  "Usage: AUTOTUNE DEADBAND <degrees>");
      return true;
    }
    float db = atof(tok[2]);
    abbot::balancer::controller::setAutotuneDeadband(db);
    return true;
  } else if (strcmp(arg, "MAXANGLE") == 0) {
    if (nt < 3) {
      LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT,
                  "Usage: AUTOTUNE MAXANGLE <degrees>");
      return true;
    }
    float maxa = atof(tok[2]);
    abbot::balancer::controller::setAutotuneMaxAngle(maxa);
    return true;
  }
  LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT,
              "AUTOTUNE usage: AUTOTUNE START | AUTOTUNE STOP | AUTOTUNE "
              "STATUS | AUTOTUNE APPLY | AUTOTUNE RELAY <amp> | AUTOTUNE "
              "DEADBAND <deg> | AUTOTUNE MAXANGLE <deg>");
  return true;
}

static bool handleFusion(const String &line, const String &up) {
  if (!up.startsWith("FUSION"))
    return false;
  char buf[128];
  char *tok[4];
  int nt = tokenizeUpper(up, buf, sizeof(buf), tok, 4);
  char *cmd = (nt >= 2) ? tok[1] : nullptr;
  if (!cmd) {
    LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT,
                "FUSION usage: FUSION STATUS | FUSION WARMUP <secs>");
    return true;
  }
  if (strcmp(cmd, "STATUS") == 0) {
    abbot::printMadgwickDiagnostics();
    bool ready = abbot::isFusionReady();
    unsigned long rem = abbot::getFusionWarmupRemaining();
    char out[128];
    snprintf(out, sizeof(out), "FUSION: ready=%d warmup_remaining=%lu",
             ready ? 1 : 0, rem);
    LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, out);
    return true;
  } else if (strcmp(cmd, "WARMUP") == 0) {
    if (nt < 3) {
      LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT,
                  "Usage: FUSION WARMUP <seconds>");
      return true;
    }
    float s = atof(tok[2]);
    if (s <= 0.0f) {
      LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "Invalid warmup seconds");
      return true;
    }
    requestTuningWarmupSeconds(s);
    char out[128];
    snprintf(out, sizeof(out), "FUSION: warmup requested ~%.1f s", (double)s);
    LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, out);
    return true;
  }
  LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT,
              "FUSION usage: FUSION STATUS | FUSION WARMUP <secs>");
  return true;
}

static bool handleFilter(const String &line, const String &up) {
  if (!up.startsWith("FILTER"))
    return false;
  char buf[128];
  char *tok[8];
  int nt = tokenizeUpper(up, buf, sizeof(buf), tok, 8);
  char *cmd = (nt >= 2) ? tok[1] : nullptr;
  if (!cmd) {
    LOG_PRINTLN(
        abbot::log::CHANNEL_DEFAULT,
        "FILTER usage: FILTER STATUS | FILTER LIST | FILTER SELECT <name>");
    return true;
  }
  if (strcmp(cmd, "STATUS") == 0) {
    char out[128];
    snprintf(out, sizeof(out), "FILTER: current=%s",
             abbot::filter::getCurrentFilterName());
    LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, out);
    return true;
  } else if (strcmp(cmd, "LIST") == 0) {
    int count = abbot::filter::getAvailableFilterCount();
    for (int i = 0; i < count; ++i) {
      char out[64];
      snprintf(out, sizeof(out), "FILTER: %s",
               abbot::filter::getAvailableFilterName(i));
      LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, out);
    }
    return true;
  } else if (strcmp(cmd, "SELECT") == 0) {
    if (nt < 3) {
      LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "Usage: FILTER SELECT <name>");
      return true;
    }
    const char *name = tok[2];
    bool ok = abbot::filter::setCurrentFilterByName(name);
    if (!ok) {
      LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "FILTER: unknown filter");
    } else {
      auto a = abbot::filter::getActiveFilter();
      if (a) {
        unsigned long ms = a->getWarmupDurationMs();
        if (ms > 0) {
          float s = ((float)ms) / 1000.0f;
          requestTuningWarmupSeconds(s);
          char tbuf[128];
          snprintf(tbuf, sizeof(tbuf), "FILTER: requested warmup %.3f s for %s",
                   (double)s, name);
          LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, tbuf);
        }
      }
    }
    return true;
  } else if (strcmp(cmd, "SET") == 0) {
    // FILTER SET ALPHA <value>
    if (nt < 4) {
      LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT,
                  "Usage: FILTER SET ALPHA <value>");
      return true;
    }
    char *param = tok[2];
    if (strcmp(param, "ALPHA") == 0) {
      char *valtok = tok[3];
      float v = atof(valtok);
      auto a = abbot::filter::getActiveFilter();
      if (!a) {
        LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "FILTER: no active filter");
        return true;
      }
      bool ok = a->setParam("ALPHA", v);
      if (ok) {
        char out[128];
        snprintf(out, sizeof(out), "FILTER: ALPHA set to %.6f", (double)v);
        LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, out);
      } else {
        LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT,
                    "FILTER: active filter does not support ALPHA or value out "
                    "of range");
      }
      return true;
    }
    LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT,
                "FILTER SET usage: FILTER SET ALPHA <value>");
    return true;
  }
  LOG_PRINTLN(
      abbot::log::CHANNEL_DEFAULT,
      "FILTER usage: FILTER STATUS | FILTER LIST | FILTER SELECT <name>");
  return true;
}

static bool handleWifi(const String &line, const String &up) {
  if (!up.startsWith("WIFI"))
    return false;

  // Preserve original case for SSID/PASS values
  String orig = line;
  orig.trim();

  // Extract second token (command)
  int p1 = orig.indexOf(' ');
  if (p1 == -1) {
    LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT,
                "WIFI usage: WIFI SHOW | WIFI SET SSID <ssid> | WIFI SET PASS "
                "<pwd> | WIFI CONNECT | WIFI DISCONNECT | WIFI RESET | WIFI "
                "STATUS | WIFI DIAG");
    return true;
  }

  int p2 = orig.indexOf(' ', p1 + 1);
  String cmd2 =
      (p2 == -1) ? orig.substring(p1 + 1) : orig.substring(p1 + 1, p2);
  cmd2.trim();
  String cmd2up = cmd2;
  cmd2up.toUpperCase();

  if (cmd2up == "SHOW") {
    Preferences p;
    if (p.begin("abbot", true)) {
      String ssid = p.getString("wifi_ssid", "");
      char out[128];
      snprintf(out, sizeof(out), "WIFI: stored_ssid=%s",
               ssid.length() ? ssid.c_str() : "(none)");
      LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, out);
      p.end();
    }
    return true;
  }

  if (cmd2up == "SET") {
    if (p2 == -1) {
      LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT,
                  "WIFI SET usage: WIFI SET SSID <ssid> | WIFI SET PASS <pwd>");
      return true;
    }
    int p3 = orig.indexOf(' ', p2 + 1);
    String which =
        (p3 == -1) ? orig.substring(p2 + 1) : orig.substring(p2 + 1, p3);
    which.trim();
    String whichUp = which;
    whichUp.toUpperCase();
    String value = (p3 == -1) ? String("") : orig.substring(p3 + 1);
    value.trim();

    if (whichUp == "SSID") {
      if (value.length() == 0) {
        LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "Usage: WIFI SET SSID <ssid>");
        return true;
      }
      Preferences pref;
      if (pref.begin("abbot", false)) {
        pref.putString("wifi_ssid", value);
        pref.end();
        LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "WIFI: SSID saved");
      }
      return true;
    } else if (whichUp == "PASS" || whichUp == "PASSWORD") {
      if (value.length() == 0) {
        LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT,
                    "Usage: WIFI SET PASS <password>");
        return true;
      }
      Preferences pref;
      if (pref.begin("abbot", false)) {
        pref.putString("wifi_pass", value);
        pref.end();
        LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT,
                    "WIFI: password saved (hidden)");
      }
      return true;
    }

    LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT,
                "WIFI SET usage: WIFI SET SSID <ssid> | WIFI SET PASS <pwd>");
    return true;
  }

  if (cmd2up == "CONNECT" || cmd2up == "CONNECTNOW" ||
      cmd2up == "CONNECT_NOW") {
    abbot::wifi_console::connectNow();
    LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "WIFI: connect requested");
    return true;
  }

  if (cmd2up == "DISCONNECT") {
    abbot::wifi_console::disconnectNow();
    LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "WIFI: disconnect requested");
    return true;
  }

  if (cmd2up == "RESET") {
    Preferences pref;
    if (pref.begin("abbot", false)) {
      pref.remove("wifi_ssid");
      pref.remove("wifi_pass");
      pref.end();
      LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT,
                  "WIFI: cleared stored credentials");
    }
    return true;
  }

  if (cmd2up == "STATUS") {
    if (WiFi.status() == WL_CONNECTED) {
      IPAddress ip = WiFi.localIP();
      char ipbuf[32];
      ip.toString().toCharArray(ipbuf, sizeof(ipbuf));
      char buf[128];
      snprintf(buf, sizeof(buf), "WIFI: connected ssid=%s ip=%s",
               WiFi.SSID().c_str(), ipbuf);
      LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, buf);
    } else {
      LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "WIFI: not connected");
    }
    return true;
  }

  if (cmd2up == "DIAG") {
    char d[256];
    abbot::wifi_console::getDiagnostics(d, sizeof(d));
    char out[300];
    snprintf(out, sizeof(out), "WIFI-DIAG: %s", d);
    LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, out);
    return true;
  }

  LOG_PRINTLN(
      abbot::log::CHANNEL_DEFAULT,
      "WIFI usage: WIFI SHOW | WIFI SET SSID <ssid> | WIFI SET PASS <pwd> | "
      "WIFI CONNECT | WIFI DISCONNECT | WIFI RESET | WIFI STATUS | WIFI DIAG");
  return true;
}

// Handle textual form: "MOTOR GET ENCODER <LEFT|RIGHT|ID>" (used by remote
// clients). The interactive menu already supports this, but textual remote
// commands went unhandled; add a small parser here so WiFi clients see the
// same output as the menu.
/**
 * Helper: print encoder(s) for a textual argument.
 *
 * Accepts the same argument forms as the interactive menu: empty -> print
 * both encoders, "LEFT"/"RIGHT" -> print the selected side, or a numeric
 * motor id. Numeric tokens are validated to avoid accidental interpretation
 * of non-numeric strings as id 0.
 *
 * Returns true if a response was emitted (including usage message), false
 * if the argument didn't match the expected syntax.
 */
// Encoder printing handled by `abbot::motor::printEncoderForArg` in
// `driver_manager.cpp` to keep motor-related logic inside the motor module.

// Thin wrapper used by the main parser: detect the textual form
// "MOTOR GET ENCODER [arg]" and delegate to printEncoderForArg.
static bool handleMotorGetEncoder(const String &line, const String &up) {
  if (!up.startsWith("MOTOR"))
    return false;
  int p1 = up.indexOf(' ');
  if (p1 == -1)
    return false;
  int p2 = up.indexOf(' ', p1 + 1);
  if (p2 == -1)
    return false;
  String cmd2 = (p2 == -1) ? up.substring(p1 + 1) : up.substring(p1 + 1, p2);
  cmd2.trim();
  if (cmd2 != "GET")
    return false;
  int p3 = up.indexOf(' ', p2 + 1);
  String cmd3 = (p3 == -1) ? up.substring(p2 + 1) : up.substring(p2 + 1, p3);
  cmd3.trim();
  if (cmd3 != "ENCODER")
    return false;
  String arg = (p3 == -1) ? String("") : line.substring(p3 + 1);
  arg.trim();
  abbot::motor::EncoderReport rep;
  bool ok = abbot::motor::getEncoderReportFromArg(arg.c_str(), rep);
  if (!ok) {
    LOG_PRINTLN(abbot::log::CHANNEL_MOTOR,
                "Usage: MOTOR GET ENCODER <LEFT|RIGHT|ID>");
    return false;
  }
  if (rep.both) {
    LOG_PRINTF(abbot::log::CHANNEL_MOTOR,
               "MOTOR: encoder L(id=%d)=%ld R(id=%d)=%ld\n", rep.leftId,
               (long)rep.leftVal, rep.rightId, (long)rep.rightVal);
  } else {
    if (rep.requestedId == rep.leftId) {
      LOG_PRINTF(abbot::log::CHANNEL_MOTOR,
                 "MOTOR: encoder id=%d value=%ld\n", rep.requestedId,
                 (long)rep.leftVal);
    } else if (rep.requestedId == rep.rightId) {
      LOG_PRINTF(abbot::log::CHANNEL_MOTOR,
                 "MOTOR: encoder id=%d value=%ld\n", rep.requestedId,
                 (long)rep.rightVal);
    } else {
      LOG_PRINTF(abbot::log::CHANNEL_MOTOR,
                 "MOTOR: encoder id=%d value=%ld\n", rep.requestedId,
                 (long)rep.leftVal);
    }
  }
  return true;
}

// Handle textual form: "MOTOR TELEMETRY <LEFT|RIGHT|ALL> <ms>"
// Returns true if handled (including printing usage or start/stop messages).
static bool handleMotorTelemetry(const String &line, const String &up) {
  if (!up.startsWith("MOTOR")) {
    return false;
  }
  int p1 = up.indexOf(' ');
  if (p1 == -1) {
    return false;
  }
  int p2 = up.indexOf(' ', p1 + 1);
  if (p2 == -1) {
    return false;
  }
  String cmd2 = (p2 == -1) ? up.substring(p1 + 1) : up.substring(p1 + 1, p2);
  cmd2.trim();
  if (cmd2 != "TELEMETRY") {
    return false;
  }
  String arg = (p2 == -1) ? String("") : line.substring(p2 + 1);
  arg.trim();
  if (arg.length() == 0) {
    LOG_PRINTLN(abbot::log::CHANNEL_MOTOR,
                "Usage: MOTOR TELEMETRY <LEFT|RIGHT|ALL> <ms> (0 to stop)");
    return true;
  }
  int sp = arg.indexOf(' ');
  if (sp < 0) {
    LOG_PRINTLN(abbot::log::CHANNEL_MOTOR,
                "Usage: MOTOR TELEMETRY <LEFT|RIGHT|ALL> <ms> (0 to stop)");
    return true;
  }
  String sideTok = arg.substring(0, sp);
  String msTok = arg.substring(sp + 1);
  sideTok.trim();
  msTok.trim();
  int ms = msTok.toInt();
  if (ms < 0) {
    LOG_PRINTLN(abbot::log::CHANNEL_MOTOR, "Invalid interval");
    return true;
  }
  bool reportBoth = false;
  bool leftSelected = true;
  String upside = sideTok;
  upside.toUpperCase();
  if (upside == "ALL") {
    reportBoth = true;
  } else if (upside == "LEFT") {
    reportBoth = false;
    leftSelected = true;
  } else if (upside == "RIGHT") {
    reportBoth = false;
    leftSelected = false;
  } else {
    LOG_PRINTLN(abbot::log::CHANNEL_MOTOR,
                "Usage: MOTOR TELEMETRY <LEFT|RIGHT|ALL> <ms> (0 to stop)");
    return true;
  }
  if (ms == 0) {
    g_telemetryManager.stop();
    LOG_PRINTLN(abbot::log::CHANNEL_MOTOR, "MOTOR: telemetry stopped");
    return true;
  }
  g_telemetryManager.stop();
  g_telemetryManager.start(reportBoth, ms, leftSelected);
  LOG_PRINTF(abbot::log::CHANNEL_MOTOR,
             "MOTOR: telemetry started mode=%s interval=%dms\n",
             (reportBoth ? "ALL" : (leftSelected ? "LEFT" : "RIGHT")), ms);
  return true;
}

// Start the interactive menu programmatically (prints the menu and makes it
// active)
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

  // Delegate to shared processor so remote callers can reuse the same logic.
  processSerialLine(driver, line);
}

void processSerialLine(abbot::BMI088Driver *driver, const String &line) {
  String sline = line;
  String up = sline;
  up.toUpperCase();

  // If interactive menu is active, only feed the menu when the input
  // looks like a numeric selection. Otherwise fall through so textual
  // commands are still processed while the menu is visible.
  if (g_menuActive && g_currentMenu) {
    String s = sline;
    s.trim();
    int sp = s.indexOf(' ');
    String first = (sp == -1) ? s : s.substring(0, sp);
    bool firstIsNum = true;
    if (first.length() == 0)
      firstIsNum = false;
    for (unsigned int i = 0; i < first.length() && firstIsNum; ++i) {
      if (!isDigit(first.charAt(i)))
        firstIsNum = false;
    }
    if (firstIsNum) {
      SerialMenu *next = g_currentMenu->handleInput(sline);
      if (next == nullptr) {
        g_menuActive = false;
        g_currentMenu = nullptr;
      } else {
        g_currentMenu = next;
      }
      return;
    }
    // else fall through to textual command parsing
  }

  // Now the same sequence of handlers as the original implementation.
  if (handleCalib(driver, sline, up))
    return;
  if (up == "HELP" || up == "?" || up.startsWith("HELP ")) {
    ensureMenus(driver);
    g_menuActive = true;
    g_currentMenu = g_rootMenu;
    if (g_currentMenu)
      g_currentMenu->enter();
    return;
  }
  if (handleTuning(sline, up))
    return;
  if (handleLog(sline, up))
    return;
  if (handleWifi(sline, up))
    return;
  // Support textual "MOTOR GET ENCODER ..." from remote clients
  // Parse minimal form here and delegate to the shared helper. This keeps
  // behaviour identical between interactive menu and remote textual input.
  if (handleMotorGetEncoder(sline, up)) {
    // handleMotorGetEncoder will call printEncoderForArg; it returns true on
    // handled input (including usage), so we simply return.
    return;
  }
  if (handleMotorTelemetry(sline, up))
    return;
  if (handleFusion(sline, up))
    return;
  if (handleFilter(sline, up))
    return;
  if (handleBalance(sline, up))
    return;
  if (handleAutotune(sline, up))
    return;
  if (auto d = abbot::motor::getActiveMotorDriver()) {
    if (d->processSerialCommand(sline))
      return;
  }
  LOG_PRINT(abbot::log::CHANNEL_DEFAULT, "Unknown command: ");
  LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, sline);
}

void receiveRemoteLine(const String &line) {
  processSerialLine(g_serial_task_driver, line);
}

void serialTaskEntry(void *pvParameters) {
  abbot::BMI088Driver *driver =
      reinterpret_cast<abbot::BMI088Driver *>(pvParameters);
  g_serial_task_driver = driver;
  for (;;) {
    processSerialOnce(driver);
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

} // namespace serialcmds
} // namespace abbot
