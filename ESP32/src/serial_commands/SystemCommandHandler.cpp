#include "serial_commands/SystemCommandHandler.h"
#include "logging.h"
#include <Arduino.h>

namespace abbot {
namespace serialcmds {

bool SystemCommandHandler::handleCommand(const String& line, const String& lineUpper) {
    if (lineUpper == "REBOOT") {
        LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "Rebooting...");
        delay(100);
        ESP.restart();
        return true;
    }
    if (lineUpper == "HEAP") {
        LOG_PRINT(abbot::log::CHANNEL_DEFAULT, "Free heap: ");
        LOG_PRINT(abbot::log::CHANNEL_DEFAULT, String(ESP.getFreeHeap()));
        LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, " bytes");
        return true;
    }
    if (lineUpper == "TASKS") {
        LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "Task List (vTaskList disabled in sdkconfig)");
        LOG_PRINT(abbot::log::CHANNEL_DEFAULT, "Number of tasks: ");
        LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, String(uxTaskGetNumberOfTasks()));
        /*
        char buffer[1024];
        vTaskList(buffer);
        LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "Task List:");
        LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, buffer);
        */
        return true;
    }
    return false;
}

SerialMenu* SystemCommandHandler::buildMenu() {
    if (!m_menu) {
        m_menu.reset(new SerialMenu("System Commands"));
        int id = 1;
        m_menu->addEntry(id++, "Reboot", [](const String&) { ESP.restart(); });
        m_menu->addEntry(id++, "Show Heap", [](const String&) {
            LOG_PRINT(abbot::log::CHANNEL_DEFAULT, "Free heap: ");
            LOG_PRINT(abbot::log::CHANNEL_DEFAULT, String(ESP.getFreeHeap()));
            LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, " bytes");
        });
        m_menu->addEntry(id++, "Show Tasks", [](const String&) {
            LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "Task List (vTaskList disabled in sdkconfig)");
            LOG_PRINT(abbot::log::CHANNEL_DEFAULT, "Number of tasks: ");
            LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, String(uxTaskGetNumberOfTasks()));
            /*
            char buffer[1024];
            vTaskList(buffer);
            LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "Task List:");
            LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, buffer);
            */
        });
    }
    return m_menu.get();
}

} // namespace serialcmds
} // namespace abbot
