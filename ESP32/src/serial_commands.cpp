// serial_commands.cpp
#include "serial_commands.h"
#include "serial_commands/CommandRegistry.h"
#include "serial_commands/WifiCommandHandler.h"
#include "serial_commands/MotorCommandHandler.h"
#include "serial_commands/BalancerCommandHandler.h"
#include "serial_commands/AutotuneCommandHandler.h"
#include "serial_commands/CalibrationCommandHandler.h"
#include "FusionService.h"
#include "serial_commands/FilterCommandHandler.h"
#include "serial_commands/TuningCommandHandler.h"
#include "serial_commands/LogCommandHandler.h"
#include "serial_commands/FusionCommandHandler.h"
#include "motor_drivers/driver_manager.h"
#include "BMI088Driver.h"
#include "logging.h"
#include "serial_menu.h"
#include "tuning_capture.h"
#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <memory>

namespace abbot {
namespace serialcmds {

// Module-local copy of the driver pointer
static abbot::BMI088Driver *g_serial_task_driver = nullptr;

// Globals for interactive menu state
static SerialMenu *g_rootMenu = nullptr;
static SerialMenu *g_currentMenu = nullptr;
static bool g_menuActive = false;
static bool g_building_menu = false;
static portMUX_TYPE g_menu_mux = portMUX_INITIALIZER_UNLOCKED;

// Command Registry
static abbot::serialcmds::CommandRegistry g_registry;

// Fusion Service instance
static abbot::FusionService g_fusionService;

static void onCaptureCompleteRefreshMenu() {
  if (g_menuActive && g_currentMenu) {
    g_currentMenu->enter();
  }
}

static void ensureMenus(abbot::BMI088Driver *driver) {
  if (g_rootMenu || g_building_menu)
    return;
  g_building_menu = true;

  // 1. Register all handlers first (registry takes ownership)
  g_registry.registerHandler(std::unique_ptr<ICommandHandler>(new WifiCommandHandler()));
  g_registry.registerHandler(std::unique_ptr<ICommandHandler>(new MotorCommandHandler()));
  g_registry.registerHandler(std::unique_ptr<ICommandHandler>(new BalancerCommandHandler(&g_fusionService)));
  g_registry.registerHandler(std::unique_ptr<ICommandHandler>(new AutotuneCommandHandler()));
  g_registry.registerHandler(std::unique_ptr<ICommandHandler>(new CalibrationCommandHandler(driver)));
  g_registry.registerHandler(std::unique_ptr<ICommandHandler>(new FilterCommandHandler()));
  g_registry.registerHandler(std::unique_ptr<ICommandHandler>(new TuningCommandHandler()));
  g_registry.registerHandler(std::unique_ptr<ICommandHandler>(new LogCommandHandler()));
  g_registry.registerHandler(std::unique_ptr<ICommandHandler>(new FusionCommandHandler(&g_fusionService)));

  // 2. Build root menu from registered handlers
  SerialMenu *root = new SerialMenu("Main Menu");
  int menuId = 1;

  for (auto& handler : g_registry.getHandlers()) {
    SerialMenu* sub = handler->buildMenu();
    if (sub) {
      root->addSubmenu(menuId++, handler->getPrefix(), sub);
    }
  }

  portENTER_CRITICAL(&g_menu_mux);
  g_rootMenu = root;
  g_building_menu = false;
  portEXIT_CRITICAL(&g_menu_mux);

  abbot::tuning::setOnCaptureComplete(onCaptureCompleteRefreshMenu);
}

void startInteractiveMenu(abbot::BMI088Driver *driver) {
  ensureMenus(driver);
  int attempts = 0;
  while (!g_rootMenu && g_building_menu && attempts < 20) {
    vTaskDelay(pdMS_TO_TICKS(10));
    ++attempts;
  }
  ensureMenus(driver);
  g_menuActive = true;
  g_currentMenu = g_rootMenu;
  if (g_currentMenu) {
    g_currentMenu->enter();
  }
}

void processSerialLine(abbot::BMI088Driver *driver, const String &line) {
  String sline = line;
  sline.trim();
  if (sline.length() == 0) return;

  String up = sline;
  up.toUpperCase();

  // Interactive menu handling
  if (g_menuActive && g_currentMenu) {
    int sp = sline.indexOf(' ');
    String first = (sp == -1) ? sline : sline.substring(0, sp);
    bool firstIsNum = (first.length() > 0);
    for (unsigned int i = 0; i < first.length() && firstIsNum; ++i) {
      if (!isDigit(first.charAt(i))) firstIsNum = false;
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
  }

  // HELP command
  if (up == "HELP" || up == "?" || up.startsWith("HELP ")) {
    ensureMenus(driver);
    g_menuActive = true;
    g_currentMenu = g_rootMenu;
    if (g_currentMenu) g_currentMenu->enter();
    return;
  }

  // Dispatch to registry
  if (g_registry.dispatch(sline, up)) {
    return;
  }

  // Fallback for motor driver specific commands
  if (auto d = abbot::motor::getActiveMotorDriver()) {
    if (d->processSerialCommand(sline)) return;
  }

  LOG_PRINT(abbot::log::CHANNEL_DEFAULT, "Unknown command: ");
  LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, sline);
}

void processSerialOnce(abbot::BMI088Driver *driver) {
  if (!driver) return;
  ensureMenus(driver);
  if (!Serial || Serial.available() == 0) return;
  String line = Serial.readStringUntil('\n');
  processSerialLine(driver, line);
}

void receiveRemoteLine(const String &line) {
  processSerialLine(g_serial_task_driver, line);
}

void serialTaskEntry(void *pvParameters) {
  abbot::BMI088Driver *driver = reinterpret_cast<abbot::BMI088Driver *>(pvParameters);
  g_serial_task_driver = driver;
  for (;;) {
    processSerialOnce(driver);
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

} // namespace serialcmds
} // namespace abbot
