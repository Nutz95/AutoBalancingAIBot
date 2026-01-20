#include "serial_commands/SystemCommandHandler.h"
#include "TelemetryService.h"
#include "esp_wifi_console.h"
#include "imu_drivers/imu_manager.h"
#include "logging.h"
#include <Arduino.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <esp_freertos_hooks.h>

namespace abbot {
namespace serialcmds {

static TaskHandle_t g_cpu_monitor_task = nullptr;
static uint32_t g_cpu_monitor_period_ms = 1000;

static volatile uint32_t g_cpu_total_ticks_core0 = 0;
static volatile uint32_t g_cpu_idle_ticks_core0 = 0;
static volatile uint32_t g_cpu_total_ticks_core1 = 0;
static volatile uint32_t g_cpu_idle_ticks_core1 = 0;

static uint32_t g_cpu_prev_total_ticks_core0 = 0;
static uint32_t g_cpu_prev_idle_ticks_core0 = 0;
static uint32_t g_cpu_prev_total_ticks_core1 = 0;
static uint32_t g_cpu_prev_idle_ticks_core1 = 0;

static TaskHandle_t g_idle_task_core0 = nullptr;
static TaskHandle_t g_idle_task_core1 = nullptr;
static bool g_cpu_tick_hooks_installed = false;

static void cpuTickHookCore0() {
    g_cpu_total_ticks_core0++;
    if (g_idle_task_core0 && xTaskGetCurrentTaskHandle() == g_idle_task_core0) {
        g_cpu_idle_ticks_core0++;
    }
}

static void cpuTickHookCore1() {
    g_cpu_total_ticks_core1++;
    if (g_idle_task_core1 && xTaskGetCurrentTaskHandle() == g_idle_task_core1) {
        g_cpu_idle_ticks_core1++;
    }
}

static bool ensureCpuTickHooksInstalled() {
    if (g_cpu_tick_hooks_installed) {
        return true;
    }

    g_idle_task_core0 = xTaskGetIdleTaskHandleForCPU(0);
#if (portNUM_PROCESSORS >= 2)
    g_idle_task_core1 = xTaskGetIdleTaskHandleForCPU(1);
#else
    g_idle_task_core1 = nullptr;
#endif

    esp_err_t err0 = esp_register_freertos_tick_hook_for_cpu(cpuTickHookCore0, 0);
    if (err0 != ESP_OK) {
        return false;
    }

#if (portNUM_PROCESSORS >= 2)
    esp_err_t err1 = esp_register_freertos_tick_hook_for_cpu(cpuTickHookCore1, 1);
    if (err1 != ESP_OK) {
        return false;
    }
#endif

    g_cpu_tick_hooks_installed = true;
    return true;
}

bool tryGetCpuLoadPercent(float& cpu0_percent, float& cpu1_percent) {
    if (!ensureCpuTickHooksInstalled()) {
        return false;
    }

    const uint32_t total0 = g_cpu_total_ticks_core0;
    const uint32_t idle0 = g_cpu_idle_ticks_core0;
    const uint32_t total1 = g_cpu_total_ticks_core1;
    const uint32_t idle1 = g_cpu_idle_ticks_core1;

    const uint32_t total0_delta = total0 - g_cpu_prev_total_ticks_core0;
    const uint32_t idle0_delta = idle0 - g_cpu_prev_idle_ticks_core0;
    const uint32_t total1_delta = total1 - g_cpu_prev_total_ticks_core1;
    const uint32_t idle1_delta = idle1 - g_cpu_prev_idle_ticks_core1;

    g_cpu_prev_total_ticks_core0 = total0;
    g_cpu_prev_idle_ticks_core0 = idle0;
    g_cpu_prev_total_ticks_core1 = total1;
    g_cpu_prev_idle_ticks_core1 = idle1;

    if (total0_delta == 0) {
        return false;
    }

    const float idle0_frac = (float)idle0_delta / (float)total0_delta;
    cpu0_percent = 100.0f * (1.0f - idle0_frac);
    if (cpu0_percent < 0.0f) {
        cpu0_percent = 0.0f;
    }
    if (cpu0_percent > 100.0f) {
        cpu0_percent = 100.0f;
    }

#if (portNUM_PROCESSORS >= 2)
    if (total1_delta == 0) {
        return false;
    }
    const float idle1_frac = (float)idle1_delta / (float)total1_delta;
    cpu1_percent = 100.0f * (1.0f - idle1_frac);
    if (cpu1_percent < 0.0f) {
        cpu1_percent = 0.0f;
    }
    if (cpu1_percent > 100.0f) {
        cpu1_percent = 100.0f;
    }
#else
    cpu1_percent = 0.0f;
#endif

    return true;
}

static void cpuMonitorTaskEntry(void* pv) {
    (void)pv;
    if (!ensureCpuTickHooksInstalled()) {
        LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "CPU: unable to install tick hooks");
        g_cpu_monitor_task = nullptr;
        vTaskDelete(nullptr);
    }

    for (;;) {
        float cpu0 = 0.0f;
        float cpu1 = 0.0f;
        if (tryGetCpuLoadPercent(cpu0, cpu1)) {
            LOG_PRINTF(abbot::log::CHANNEL_DEFAULT, "CPU: t=%lums core0=%.1f%% core1=%.1f%%\n",
                       (unsigned long)millis(), (double)cpu0, (double)cpu1);
        }
        vTaskDelay(pdMS_TO_TICKS(g_cpu_monitor_period_ms));
    }

    g_cpu_monitor_task = nullptr;
    vTaskDelete(nullptr);
}

bool SystemCommandHandler::handleCommand(const String& line, const String& lineUpper) {
    String s = lineUpper;
    s.trim();

    if (s == "SYS" || s == "SYS HELP") {
        LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "SYS usage: SYS REBOOT | SYS HEAP | SYS TASKS | SYS IMU REINIT | SYS CPU [STREAM <ms>|STOP]");
        return true;
    }

    if (s == "SYS REBOOT") {
        LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "Rebooting...");
        delay(100);
        ESP.restart();
        return true;
    }
    if (s == "SYS IMU REINIT") {
        LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "SYSTEM: Re-initializing IMU...");
        auto drv = abbot::imu::getActiveIMUDriver();
        if (drv) {
            if (drv->begin()) {
                LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "SYSTEM: IMU re-initialized successfully");
            } else {
                LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "SYSTEM: IMU re-initialization FAILED");
            }
        } else {
            LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "SYSTEM: No active IMU driver found");
        }
        return true;
    }
    if (s == "SYS HEAP") {
        LOG_PRINT(abbot::log::CHANNEL_DEFAULT, "Free heap: ");
        LOG_PRINT(abbot::log::CHANNEL_DEFAULT, String(ESP.getFreeHeap()));
        LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, " bytes");
        return true;
    }
    if (s == "SYS TASKS") {
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

    if (s == "SYS CPU") {
        if (!ensureCpuTickHooksInstalled()) {
            LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "CPU: unable to install tick hooks");
            return true;
        }

        float cpu0 = 0.0f;
        float cpu1 = 0.0f;
        // Take a short sampling window so the result is meaningful.
        (void)tryGetCpuLoadPercent(cpu0, cpu1);
        vTaskDelay(pdMS_TO_TICKS(120));

        if (tryGetCpuLoadPercent(cpu0, cpu1)) {
            LOG_PRINTF(abbot::log::CHANNEL_DEFAULT, "CPU: t=%lums core0=%.1f%% core1=%.1f%%\n",
                       (unsigned long)millis(), (double)cpu0, (double)cpu1);
        } else {
            LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "CPU: not enough samples yet");
        }
        return true;
    }

    if (s.startsWith("SYS TELEM UDP")) {
        if (s == "SYS TELEM UDP STOP") {
            // No real stop implemented yet, but we can set invalid target
            abbot::telemetry::TelemetryService::getInstance().begin(INADDR_NONE);
            LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "UDP Telemetry stopped");
            return true;
        }

        char ip_str[32] = {0};
        IPAddress targetIP;
        if (sscanf(s.c_str(), "SYS TELEM UDP %s", ip_str) == 1) {
            String sip(ip_str);
            if (sip == "AUTO") {
                targetIP = abbot::wifi_console::getClientIP();
                if (targetIP == INADDR_NONE) {
                    LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "UDP Telemetry: AUTO failed, no client connected via WiFi Console");
                    return true;
                }
            } else {
                targetIP.fromString(ip_str);
            }

            if (targetIP != INADDR_NONE) {
                abbot::telemetry::TelemetryService::getInstance().begin(targetIP);
                LOG_PRINTF(abbot::log::CHANNEL_DEFAULT, "UDP Telemetry started -> %s:8888\n", targetIP.toString().c_str());
            } else {
                LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "UDP Telemetry: invalid IP");
            }
        } else {
            LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "Usage: SYS TELEM UDP <IP|AUTO|STOP>");
        }
        return true;
    }

    if (s.startsWith("SYS CPU STREAM")) {
        uint32_t period_ms = 1000;
        if (sscanf(s.c_str(), "SYS CPU STREAM %u", &period_ms) == 1) {
            if (period_ms < 100) {
                period_ms = 100;
            }
            g_cpu_monitor_period_ms = period_ms;
        }

        if (!g_cpu_monitor_task) {
            BaseType_t target_core = 0;
#if (portNUM_PROCESSORS >= 2)
            target_core = 1;
#endif
            xTaskCreatePinnedToCore(cpuMonitorTaskEntry, "cpuMon", 4096, nullptr, 2, &g_cpu_monitor_task, target_core);
        }
        LOG_PRINTF(abbot::log::CHANNEL_DEFAULT, "CPU: streaming every %u ms\n", (unsigned)g_cpu_monitor_period_ms);
        return true;
    }

    if (s == "SYS CPU STOP") {
        if (g_cpu_monitor_task) {
            vTaskDelete(g_cpu_monitor_task);
            g_cpu_monitor_task = nullptr;
        }
        LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "CPU: streaming stopped");
        return true;
    }
    return false;
}

SerialMenu* SystemCommandHandler::buildMenu() {
    if (!m_menu) {
        m_menu.reset(new SerialMenu("System Commands"));
        int id = 1;
        m_menu->addEntry(id++, "Reboot", [](const String&) { ESP.restart(); });
        m_menu->addEntry(id++, "IMU Reinit", [](const String&) {
            LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "SYSTEM: Re-initializing IMU...");
            auto drv = abbot::imu::getActiveIMUDriver();
            if (drv && drv->begin()) {
                LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "SYSTEM: IMU re-initialized successfully");
            } else {
                LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "SYSTEM: IMU re-initialization FAILED");
            }
        });
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

        m_menu->addEntry(id++, "CPU Load (once)", [](const String&) {
            if (!ensureCpuTickHooksInstalled()) {
                LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "CPU: unable to install tick hooks");
                return;
            }

            float cpu0 = 0.0f;
            float cpu1 = 0.0f;
            (void)tryGetCpuLoadPercent(cpu0, cpu1);
            vTaskDelay(pdMS_TO_TICKS(120));

            if (tryGetCpuLoadPercent(cpu0, cpu1)) {
                LOG_PRINTF(abbot::log::CHANNEL_DEFAULT, "CPU: t=%lums core0=%.1f%% core1=%.1f%%\n",
                           (unsigned long)millis(), (double)cpu0, (double)cpu1);
            } else {
                LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "CPU: not enough samples yet");
            }
        });
    }
    return m_menu.get();
}

} // namespace serialcmds
} // namespace abbot
