#include "serial_commands/WebUiCommandHandler.h"

#include "FilterService.h"
#include "balancer_controller.h"
#include "balancing/BalancingManager.h"
#include "balancing/strategies/CascadedLqrStrategy.h"
#include "esp_wifi_console.h"
#include "logging.h"
#include "motor_drivers/driver_manager.h"

#include <Arduino.h>

#include <cstdio>
#include <cstring>

namespace abbot {
namespace serialcmds {
namespace {

constexpr size_t CONTROLLER_JSON_BUFFER_SIZE = 768;
constexpr size_t FILTER_JSON_BUFFER_SIZE = 768;
constexpr size_t FILTER_LIST_BUFFER_SIZE = 256;
constexpr size_t SUPPORTED_PARAMS_BUFFER_SIZE = 192;
constexpr size_t PARAMS_BUFFER_SIZE = 256;
constexpr size_t MOTORS_JSON_BUFFER_SIZE = 320;
constexpr size_t SYSTEM_JSON_BUFFER_SIZE = 256;
constexpr size_t LOGS_JSON_BUFFER_SIZE = 256;
constexpr size_t ENABLED_CHANNELS_BUFFER_SIZE = 160;

constexpr const char* CONTROLLER_PREFIX = "WEBUI_CONTROLLER_JSON: ";
constexpr const char* FILTER_PREFIX = "WEBUI_FILTER_JSON: ";
constexpr const char* MOTORS_PREFIX = "WEBUI_MOTORS_JSON: ";
constexpr const char* SYSTEM_PREFIX = "WEBUI_SYSTEM_JSON: ";
constexpr const char* LOGS_PREFIX = "WEBUI_LOGS_JSON: ";

using WebUiSectionEmitter = void (*)();

struct WebUiGetRoute {
    const char* section;
    WebUiSectionEmitter emitter;
};

struct FilterParamDescriptor {
    const char* name;
};

constexpr FilterParamDescriptor FILTER_PARAM_DESCRIPTORS[] = {
    {"ALPHA"},
    {"KACC"},
    {"KBIAS"},
    {"BETA"},
};

constexpr abbot::log::Channel LOG_CHANNELS[] = {
    abbot::log::CHANNEL_TUNING,
    abbot::log::CHANNEL_BLE,
    abbot::log::CHANNEL_IMU,
    abbot::log::CHANNEL_MOTOR,
    abbot::log::CHANNEL_BALANCER,
    abbot::log::CHANNEL_DEFAULT,
};

bool appendString(char* buffer, size_t bufferSize, size_t& offset, const char* text) {
    if (!buffer || !text || offset >= bufferSize) {
        return false;
    }

    const int written = snprintf(buffer + offset, bufferSize - offset, "%s", text);
    if (written < 0 || static_cast<size_t>(written) >= (bufferSize - offset)) {
        return false;
    }

    offset += static_cast<size_t>(written);
    return true;
}

const char* controllerModeName(abbot::balancer::controller::ControllerMode mode) {
    return mode == abbot::balancer::controller::ControllerMode::CASCADED_LQR ? "LQR" : "PID";
}

void emitJsonLine(const char* prefix, const char* payload) {
    if (!prefix || !payload) {
        return;
    }
    LOG_PRINTF(abbot::log::CHANNEL_DEFAULT, "%s%s\n", prefix, payload);
}

} // namespace

bool WebUiCommandHandler::handleCommand(const String& line, const String& lineUpper) {
    (void)line;
    String command = lineUpper;
    command.trim();

    if (command == "WEBUI" || command == "WEBUI HELP") {
        LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "WEBUI usage: WEBUI GET CONTROLLER | WEBUI GET FILTER | WEBUI GET MOTORS | WEBUI GET SYSTEM | WEBUI GET LOGS");
        return true;
    }

    if (command.startsWith("WEBUI GET ")) {
        return handleGetCommand(command);
    }

    return false;
}

bool WebUiCommandHandler::handleGetCommand(const String& lineUpper) {
    const String section = lineUpper.substring(10);
    static const WebUiGetRoute webUiGetRoutes[] = {
        {"CONTROLLER", &WebUiCommandHandler::emitControllerJson},
        {"FILTER", &WebUiCommandHandler::emitFilterJson},
        {"MOTORS", &WebUiCommandHandler::emitMotorsJson},
        {"SYSTEM", &WebUiCommandHandler::emitSystemJson},
        {"LOGS", &WebUiCommandHandler::emitLogsJson},
    };

    for (const auto& route : webUiGetRoutes) {
        if (section == route.section) {
            route.emitter();
            return true;
        }
    }

    LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "WEBUI GET usage: WEBUI GET CONTROLLER | FILTER | MOTORS | SYSTEM | LOGS");
    return true;
}

void WebUiCommandHandler::emitControllerJson() {
    float pidKp = 0.0f;
    float pidKi = 0.0f;
    float pidKd = 0.0f;
    abbot::balancer::controller::getGains(pidKp, pidKi, pidKd);

    abbot::balancer::controller::CascadedGains lqrGains {};
    abbot::balancer::controller::getCascadedGains(lqrGains);

    float leftMotorGain = 0.0f;
    float rightMotorGain = 0.0f;
    abbot::balancer::controller::getMotorGains(leftMotorGain, rightMotorGain);

    float pitchRateLpfHz = 0.0f;
    float cmdLpfHz = 0.0f;
    float yawGain = 0.0f;
    float yawRateGain = 0.0f;
    if (auto* strategy = abbot::balancing::BalancingManager::getInstance().getStrategy(abbot::balancing::StrategyType::CASCADED_LQR)) {
        auto lqrConfig = static_cast<abbot::balancing::CascadedLqrStrategy*>(strategy)->getConfig();
        pitchRateLpfHz = lqrConfig.pitch_rate_lpf_hz;
        cmdLpfHz = lqrConfig.cmd_lpf_hz;
        yawGain = lqrConfig.k_yaw;
        yawRateGain = lqrConfig.k_yaw_rate;
    }

    char payload[CONTROLLER_JSON_BUFFER_SIZE];
    snprintf(
        payload,
        sizeof(payload),
        "{\"section\":\"controller\",\"mode\":\"%s\",\"pid\":{\"kp\":%.6f,\"ki\":%.6f,\"kd\":%.6f},\"lqr\":{\"gains\":{\"k_pitch\":%.6f,\"k_gyro\":%.6f,\"k_dist\":%.6f,\"k_speed\":%.6f},\"adaptive_trim_enabled\":%s,\"filters\":{\"pitch_rate_lpf_hz\":%.6f,\"cmd_lpf_hz\":%.6f},\"yaw\":{\"k_yaw\":%.6f,\"k_yaw_rate\":%.6f}},\"output\":{\"deadband\":%.6f,\"min_cmd\":%.6f,\"motor_gains\":{\"left\":%.6f,\"right\":%.6f}}}",
        controllerModeName(abbot::balancer::controller::getMode()),
        static_cast<double>(pidKp),
        static_cast<double>(pidKi),
        static_cast<double>(pidKd),
        static_cast<double>(lqrGains.k_pitch),
        static_cast<double>(lqrGains.k_gyro),
        static_cast<double>(lqrGains.k_dist),
        static_cast<double>(lqrGains.k_speed),
        abbot::balancer::controller::isAdaptiveTrimEnabled() ? "true" : "false",
        static_cast<double>(pitchRateLpfHz),
        static_cast<double>(cmdLpfHz),
        static_cast<double>(yawGain),
        static_cast<double>(yawRateGain),
        static_cast<double>(abbot::balancer::controller::getDeadband()),
        static_cast<double>(abbot::balancer::controller::getMinCmd()),
        static_cast<double>(leftMotorGain),
        static_cast<double>(rightMotorGain));
    emitJsonLine(CONTROLLER_PREFIX, payload);
}

void WebUiCommandHandler::emitFilterJson() {
    FilterService filterService;
    char availableFilters[FILTER_LIST_BUFFER_SIZE] = {0};
    char supportedParams[SUPPORTED_PARAMS_BUFFER_SIZE] = {0};
    char paramsPayload[PARAMS_BUFFER_SIZE] = {0};

    size_t availableOffset = 0;
    size_t supportedOffset = 0;
    size_t paramsOffset = 0;
    appendString(availableFilters, sizeof(availableFilters), availableOffset, "[");
    appendString(supportedParams, sizeof(supportedParams), supportedOffset, "[");
    appendString(paramsPayload, sizeof(paramsPayload), paramsOffset, "{");

    const int filterCount = filterService.getAvailableFilterCount();
    for (int index = 0; index < filterCount; ++index) {
        if (index > 0) {
            appendString(availableFilters, sizeof(availableFilters), availableOffset, ",");
        }
        char filterEntry[64];
        snprintf(filterEntry, sizeof(filterEntry), "\"%s\"", filterService.getAvailableFilterName(index));
        appendString(availableFilters, sizeof(availableFilters), availableOffset, filterEntry);
    }
    appendString(availableFilters, sizeof(availableFilters), availableOffset, "]");

    bool firstSupported = true;
    bool firstParam = true;
    if (auto* activeFilter = filterService.getActiveFilter()) {
        for (const auto& descriptor : FILTER_PARAM_DESCRIPTORS) {
            float value = 0.0f;
            if (!activeFilter->getParam(descriptor.name, value)) {
                continue;
            }

            if (!firstSupported) {
                appendString(supportedParams, sizeof(supportedParams), supportedOffset, ",");
            }
            if (!firstParam) {
                appendString(paramsPayload, sizeof(paramsPayload), paramsOffset, ",");
            }

            char supportedEntry[32];
            char paramEntry[64];
            snprintf(supportedEntry, sizeof(supportedEntry), "\"%s\"", descriptor.name);
            snprintf(paramEntry, sizeof(paramEntry), "\"%s\":%.6f", descriptor.name, static_cast<double>(value));
            appendString(supportedParams, sizeof(supportedParams), supportedOffset, supportedEntry);
            appendString(paramsPayload, sizeof(paramsPayload), paramsOffset, paramEntry);
            firstSupported = false;
            firstParam = false;
        }
    }

    appendString(supportedParams, sizeof(supportedParams), supportedOffset, "]");
    appendString(paramsPayload, sizeof(paramsPayload), paramsOffset, "}");

    char payload[FILTER_JSON_BUFFER_SIZE];
    snprintf(
        payload,
        sizeof(payload),
        "{\"section\":\"filter\",\"current\":\"%s\",\"available\":%s,\"supported\":%s,\"params\":%s}",
        filterService.getCurrentFilterName(),
        availableFilters,
        supportedParams,
        paramsPayload);
    emitJsonLine(FILTER_PREFIX, payload);
}

void WebUiCommandHandler::emitMotorsJson() {
    auto* driver = abbot::motor::getActiveMotorDriver();
    const bool motorsEnabled = driver ? driver->areMotorsEnabled() : false;
    const int leftMotorId = abbot::motor::getActiveMotorId(abbot::motor::IMotorDriver::MotorSide::LEFT, -1);
    const int rightMotorId = abbot::motor::getActiveMotorId(abbot::motor::IMotorDriver::MotorSide::RIGHT, -1);
    const bool leftInverted = abbot::motor::isActiveMotorInverted(abbot::motor::IMotorDriver::MotorSide::LEFT, false);
    const bool rightInverted = abbot::motor::isActiveMotorInverted(abbot::motor::IMotorDriver::MotorSide::RIGHT, false);
    const float leftLastCommand = driver ? driver->getLastMotorCommand(abbot::motor::IMotorDriver::MotorSide::LEFT) : 0.0f;
    const float rightLastCommand = driver ? driver->getLastMotorCommand(abbot::motor::IMotorDriver::MotorSide::RIGHT) : 0.0f;

    char payload[MOTORS_JSON_BUFFER_SIZE];
    snprintf(
        payload,
        sizeof(payload),
        "{\"section\":\"motors\",\"driver\":\"%s\",\"enabled\":%s,\"left\":{\"id\":%d,\"inverted\":%s,\"last_command\":%.6f},\"right\":{\"id\":%d,\"inverted\":%s,\"last_command\":%.6f}}",
        abbot::motor::getActiveDriverName("none"),
        motorsEnabled ? "true" : "false",
        leftMotorId,
        leftInverted ? "true" : "false",
        static_cast<double>(leftLastCommand),
        rightMotorId,
        rightInverted ? "true" : "false",
        static_cast<double>(rightLastCommand));
    emitJsonLine(MOTORS_PREFIX, payload);
}

void WebUiCommandHandler::emitSystemJson() {
    const bool wifiClientConnected = abbot::wifi_console::getClientIP() != INADDR_NONE;

    char payload[SYSTEM_JSON_BUFFER_SIZE];
    snprintf(
        payload,
        sizeof(payload),
        "{\"section\":\"system\",\"heap_bytes\":%lu,\"uptime_ms\":%lu,\"wifi_client_connected\":%s}",
        static_cast<unsigned long>(ESP.getFreeHeap()),
        static_cast<unsigned long>(millis()),
        wifiClientConnected ? "true" : "false");
    emitJsonLine(SYSTEM_PREFIX, payload);
}

void WebUiCommandHandler::emitLogsJson() {
    char enabledChannels[ENABLED_CHANNELS_BUFFER_SIZE] = {0};
    size_t enabledOffset = 0;
    appendString(enabledChannels, sizeof(enabledChannels), enabledOffset, "[");

    bool firstChannel = true;
    for (const auto channel : LOG_CHANNELS) {
        if (!abbot::log::isChannelEnabled(channel)) {
            continue;
        }
        if (!firstChannel) {
            appendString(enabledChannels, sizeof(enabledChannels), enabledOffset, ",");
        }
        char channelEntry[32];
        snprintf(channelEntry, sizeof(channelEntry), "\"%s\"", abbot::log::channelName(channel));
        appendString(enabledChannels, sizeof(enabledChannels), enabledOffset, channelEntry);
        firstChannel = false;
    }
    appendString(enabledChannels, sizeof(enabledChannels), enabledOffset, "]");

    char payload[LOGS_JSON_BUFFER_SIZE];
    snprintf(payload, sizeof(payload), "{\"section\":\"logs\",\"enabled_channels\":%s}", enabledChannels);
    emitJsonLine(LOGS_PREFIX, payload);
}

} // namespace serialcmds
} // namespace abbot
