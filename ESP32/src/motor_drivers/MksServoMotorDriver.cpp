// MksServoMotorDriver.cpp - Implementation for MKS SERVO42D/57D serial bus motors
#include "../../include/motor_drivers/MksServoMotorDriver.h"
#include "../../include/balancer_controller.h"
#include "../../config/balancer_config.h"
#include <Arduino.h>
#include <utility>

#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

namespace abbot {
namespace motor {

namespace {

const char *stepGeneratorModeName(abbot::motor::StepGeneratorMode mode) {
    switch (mode) {
        case abbot::motor::StepGeneratorMode::MCPWM:
            return "MCPWM";
        case abbot::motor::StepGeneratorMode::RMT:
            return "RMT";
        case abbot::motor::StepGeneratorMode::SOFTWARE:
            return "SOFTWARE";
        default:
            return "UNKNOWN";
    }
}

bool parseStepGeneratorModeToken(const String& token, abbot::motor::StepGeneratorMode& mode) {
    String s = token;
    s.trim();
    s.toUpperCase();
    if (s == "0" || s == "MCPWM") {
        mode = abbot::motor::StepGeneratorMode::MCPWM;
        return true;
    }
    if (s == "1" || s == "RMT") {
        mode = abbot::motor::StepGeneratorMode::RMT;
        return true;
    }
    if (s == "2" || s == "SOFTWARE" || s == "SW") {
        mode = abbot::motor::StepGeneratorMode::SOFTWARE;
        return true;
    }
    return false;
}

const char *enableLevelName(uint8_t level) {
    switch (level) {
        case 0: return "L";
        case 1: return "H";
        case 2: return "Hold";
        default: return "Unknown";
    }
}

void setHardwareEnablePin(abbot::motor::IMotorDriver::MotorSide side, bool enable, uint8_t en_effective_level = 0) {
#if MKS_SERVO_USE_STEP_DIR
    bool active_level = (en_effective_level == 1u) ? HIGH : LOW;
    bool pin_level = enable ? active_level : !active_level;
#if !MKS_SERVO_COMMON_ANODE
    pin_level = !pin_level;
#endif
    if (side == abbot::motor::IMotorDriver::MotorSide::LEFT) {
        digitalWrite(MKS_SERVO_P1_EN_PIN, pin_level);
    } else {
        digitalWrite(MKS_SERVO_P2_EN_PIN, pin_level);
    }
#else
    (void)side;
    (void)enable;
#endif
}

} // namespace

MksServoMotorDriver::MksServoMotorDriver()
    : m_left{LEFT_MOTOR_ID, LEFT_MOTOR_INVERT != 0, 0.0f, 0, 0, 0, 0, 0, false, SpeedEstimator(MKS_SERVO_SPEED_ALPHA)},
      m_right{RIGHT_MOTOR_ID, RIGHT_MOTOR_INVERT != 0, 0.0f, 0, 0, 0, 0, 0, false, SpeedEstimator(MKS_SERVO_SPEED_ALPHA)},
      m_leftBusMutex(xSemaphoreCreateRecursiveMutex()),
      m_rightBusMutex(xSemaphoreCreateRecursiveMutex()) {
#if !defined(UNIT_TEST_HOST)
    m_leftCommandQueue = xQueueCreate(MKS_SERVO_FUNCTION_QUEUE_LEN, sizeof(FunctionCommandItem));
    m_rightCommandQueue = xQueueCreate(MKS_SERVO_FUNCTION_QUEUE_LEN, sizeof(FunctionCommandItem));
#endif
}

MksServoMotorDriver::~MksServoMotorDriver() {
    if (m_leftProtocol != nullptr) {
        delete m_leftProtocol;
        m_leftProtocol = nullptr;
    }

    if (m_rightProtocol != nullptr) {
        delete m_rightProtocol;
        m_rightProtocol = nullptr;
    }

    if (m_leftTelemetryIngest != nullptr) {
        delete m_leftTelemetryIngest;
        m_leftTelemetryIngest = nullptr;
    }

    if (m_rightTelemetryIngest != nullptr) {
        delete m_rightTelemetryIngest;
        m_rightTelemetryIngest = nullptr;
    }
}

void MksServoMotorDriver::initMotorDriver() {
    // Stage 1: Initial Link Establishment (Baud Rate Detection)
    LOG_PRINTF(abbot::log::CHANNEL_DEFAULT, "mks_servo: Testing links at %lu baud...\n", (unsigned long)MKS_SERVO_BAUD);
    
    // Set buffer sizes BEFORE calling begin()
    Serial2.setRxBufferSize(1024);
    Serial2.begin(MKS_SERVO_BAUD, SERIAL_8N1, MKS_SERVO_P1_RX_PIN, MKS_SERVO_P1_TX_PIN);
    
    Serial1.setRxBufferSize(1024);
    Serial1.begin(MKS_SERVO_BAUD, SERIAL_8N1, MKS_SERVO_P2_RX_PIN, MKS_SERVO_P2_TX_PIN);

    // IMPORTANT: Delay to let Serials stabilize
    delay(50);
    
    // Create protocols
    if (m_leftProtocol) delete m_leftProtocol;
    if (m_rightProtocol) delete m_rightProtocol;
    m_leftProtocol = new MksServoProtocol(Serial2);
    m_rightProtocol = new MksServoProtocol(Serial1);

    if (m_leftTelemetryIngest) delete m_leftTelemetryIngest;
    if (m_rightTelemetryIngest) delete m_rightTelemetryIngest;
    m_leftTelemetryIngest = new MksServoTelemetryIngest(*m_leftProtocol, m_left.speedEstimator, &m_left.invert);
    m_rightTelemetryIngest = new MksServoTelemetryIngest(*m_rightProtocol, m_right.speedEstimator, &m_right.invert);

    m_left_async.interpolator.setTau(MKS_SERVO_INTERP_TAU_S);
    m_right_async.interpolator.setTau(MKS_SERVO_INTERP_TAU_S);

    delay(100);
    int found_count = scanBusOnCurrentBaud();

    if (found_count == 0) {
        LOG_PRINTF(abbot::log::CHANNEL_DEFAULT, "mks_servo: WARNING: No motors detected at %lu baud!\n", (unsigned long)MKS_SERVO_BAUD);
    } else {
        LOG_PRINTF(abbot::log::CHANNEL_DEFAULT, "mks_servo: %d motor(s) detected at %lu baud.\n", found_count, (unsigned long)MKS_SERVO_BAUD);
    }

    // Stage 2: Hardware Setup (MCPWM)
#if MKS_SERVO_USE_STEP_DIR
    pinMode(MKS_SERVO_P1_STEP_PIN, OUTPUT);
    pinMode(MKS_SERVO_P1_DIR_PIN, OUTPUT);
    pinMode(MKS_SERVO_P2_STEP_PIN, OUTPUT);
    pinMode(MKS_SERVO_P2_DIR_PIN, OUTPUT);
    pinMode(MKS_SERVO_P1_EN_PIN, OUTPUT);
    pinMode(MKS_SERVO_P2_EN_PIN, OUTPUT);

#if MKS_SERVO_COMMON_ANODE
    digitalWrite(MKS_SERVO_P1_STEP_PIN, HIGH);
    digitalWrite(MKS_SERVO_P1_DIR_PIN, HIGH);
    digitalWrite(MKS_SERVO_P2_STEP_PIN, HIGH);
    digitalWrite(MKS_SERVO_P2_DIR_PIN, HIGH);
    digitalWrite(MKS_SERVO_P1_EN_PIN, HIGH);
    digitalWrite(MKS_SERVO_P2_EN_PIN, HIGH);
#else
    digitalWrite(MKS_SERVO_P1_STEP_PIN, LOW);
    digitalWrite(MKS_SERVO_P1_DIR_PIN, LOW);
    digitalWrite(MKS_SERVO_P2_STEP_PIN, LOW);
    digitalWrite(MKS_SERVO_P2_DIR_PIN, LOW);
    digitalWrite(MKS_SERVO_P1_EN_PIN, LOW); 
    digitalWrite(MKS_SERVO_P2_EN_PIN, LOW); 
#endif

    StepGeneratorMode bootMode = static_cast<StepGeneratorMode>(m_stepGeneratorMode.load());
    if (!setStepGeneratorMode(bootMode, true)) {
        LOG_PRINTF(abbot::log::CHANNEL_DEFAULT,
                   "mks_servo: failed to init step generator mode=%s; falling back to SOFTWARE\n",
                   stepGeneratorModeName(bootMode));
        (void)setStepGeneratorMode(StepGeneratorMode::SOFTWARE, true);
    }

    LOG_PRINTF(abbot::log::CHANNEL_DEFAULT,
               "mks_servo: Step/Dir (Hybrid) Hardware Initialized with %s backend\n",
               stepGeneratorModeName(getStepGeneratorMode()));
#endif

    // Stage 3: Initial Configuration Injection
    MotorSide sides[] = {MotorSide::LEFT, MotorSide::RIGHT};
    for (MotorSide side : sides) {
        uint8_t id = getMotorId(side);
        LOG_PRINTF(abbot::log::CHANNEL_DEFAULT, "mks_servo: Configuring %s motor 0x%02X...\n", 
                   (side == MotorSide::LEFT ? "LEFT" : "RIGHT"), id);
        
        // Use a small delay between commands to avoid bus congestion
        setResponseMethod(side, true, true);
        delay(10);
        setMode(side, MKS_SERVO_DEFAULT_MODE);
        delay(10);
        setMStep(side, MKS_SERVO_DEFAULT_MSTEP);
        delay(10);
        setCurrent(side, MKS_SERVO_MA);
        delay(10);
        setHoldCurrent(side, MKS_SERVO_HOLD_PCT);
        delay(10);
    }

    // Stage 4: Launch Core Tasks
    xTaskCreatePinnedToCore(motorTaskEntry, "motorL_task", 4096, new std::pair<MksServoMotorDriver*, MotorSide>(this, MotorSide::LEFT), MKS_SERVO_TASK_PRIORITY, &m_left_async.task_handle, 0);
    xTaskCreatePinnedToCore(motorTaskEntry, "motorR_task", 4096, new std::pair<MksServoMotorDriver*, MotorSide>(this, MotorSide::RIGHT), MKS_SERVO_TASK_PRIORITY, &m_right_async.task_handle, 0);

    // Stage 5: Verification Dump
    delay(200); // Wait for task startup
    dumpAllConfigs();

    disableMotors();
}

void MksServoMotorDriver::motorTaskEntry(void* pvParameters) {
    auto args = static_cast<std::pair<MksServoMotorDriver*, MotorSide>*>(pvParameters);
    MksServoMotorDriver* driver = args->first;
    MotorSide side = args->second;
    delete args;

    driver->runMotorTask(side);
}

namespace {
uint32_t stepsPerRevFromConfigByte(uint8_t mstep_value) {
    const uint32_t microsteps = (mstep_value == 0U) ? 256U : (uint32_t)mstep_value;
    switch (microsteps) {
        case 1U:
        case 2U:
        case 4U:
        case 8U:
        case 16U:
        case 32U:
        case 64U:
        case 128U:
        case 256U:
            return MKS_SERVO_FULL_STEPS_PER_REV * microsteps;
        default:
            return MKS_SERVO_STEPS_PER_REV;
    }
}

namespace Config47Index {
constexpr size_t kWorkMode = 0;
constexpr size_t kOperatingCurrentHi = 1;
constexpr size_t kOperatingCurrentLo = 2;
constexpr size_t kHoldingCurrent = 3;
constexpr size_t kMicrostep = 4;
constexpr size_t kEnableEffectiveLevel = 5;
constexpr size_t kResponseMethod1 = 13;
constexpr size_t kResponseMethod2 = 14;
}
}

void MksServoMotorDriver::applyStepDirCommand(MotorSide side, float command, bool enabled) {
#if MKS_SERVO_USE_STEP_DIR
    AsyncState& async = (side == MotorSide::LEFT) ? m_left_async : m_right_async;
    MotorState& state = (side == MotorSide::LEFT) ? m_left : m_right;

    bool dir = (command >= 0.0f);
    if (state.invert) {
        dir = !dir;
    }
#if MKS_SERVO_PULSE_DIR_INVERT
    dir = !dir;
#endif
#if MKS_SERVO_COMMON_ANODE
    dir = !dir;
#endif

    const int dir_pin = (side == MotorSide::LEFT) ? MKS_SERVO_P1_DIR_PIN : MKS_SERVO_P2_DIR_PIN;
    digitalWrite(dir_pin, dir ? HIGH : LOW);

    float abs_cmd = fabsf(command);
    uint32_t freq = 0;
    if (abs_cmd >= 0.001f && enabled) {
        const uint32_t steps_per_rev = async.configured_steps_per_rev.load();
        freq = (uint32_t)((abs_cmd * VELOCITY_MAX_SPEED / 60.0f) * (float)steps_per_rev);
    }

    uint32_t now_us = micros();
    uint32_t step_interval_us = 0;
#if MKS_SERVO_STEP_UPDATE_HZ > 0
    step_interval_us = (uint32_t)(1000000UL / (uint32_t)MKS_SERVO_STEP_UPDATE_HZ);
#endif
    async.step_freq_requested.store(freq);
    bool time_ok = (step_interval_us == 0U) ||
                   ((int32_t)(now_us - async.last_ledc_update_us) >= (int32_t)step_interval_us);
    bool should_update = false;
    const uint32_t applied_freq = async.last_ledc_freq;

    if (freq == 0U) {
        should_update = (async.last_ledc_freq != 0U) || !enabled;
    } else if (enabled && time_ok && abs((int)freq - (int)async.last_ledc_freq) > MKS_SERVO_STEP_DEADBAND_HZ) {
        should_update = true;
    } else if (enabled && freq > 0U && applied_freq > 0U &&
               (applied_freq < (freq / 4U) || applied_freq > (freq * 4U))) {
        should_update = true;
    }

    if (!should_update) {
        return;
    }

    if (!m_stepGenerator || !m_stepGenerator->setFrequency(side == MotorSide::LEFT, freq)) {
        async.step_mutex_miss.fetch_add(1);
        return;
    }
    async.last_ledc_freq = freq;
    async.last_ledc_update_us = now_us;
    async.step_apply_ok.fetch_add(1);
#else
    (void)side;
    (void)command;
    (void)enabled;
#endif
}

void MksServoMotorDriver::runMotorTask(MotorSide side) {
    AsyncState& async = (side == MotorSide::LEFT) ? m_left_async : m_right_async;
    MotorState& state = (side == MotorSide::LEFT) ? m_left : m_right;
    HardwareSerial& serial = (side == MotorSide::LEFT) ? Serial2 : Serial1;
    MksServoProtocol& protocol = getProtocolForSide(side);
    MksServoTelemetryIngest& telemetryIngest = getTelemetryIngestForSide(side);
    SemaphoreHandle_t mutex = (side == MotorSide::LEFT) ? m_leftBusMutex : m_rightBusMutex;

    const uint32_t telemetry_interval_ms = (MKS_SERVO_ENCODER_UPDATE_HZ > 0) ? (1000 / MKS_SERVO_ENCODER_UPDATE_HZ) : 10;
    uint8_t loop_divider = 0;
    uint32_t next_tx_allowed_us = 0;
    bool ack_pending = false;
    uint32_t ack_deadline_us = 0;
    uint32_t ack_start_us = 0;
    uint8_t ack_expected_function = 0xF6;
    MksServoProtocol::AckParser ack_parser;
    bool function_pending = false;
    FunctionCommandItem pending_function;
    uint32_t seen_reset_epoch = async.runtime_reset_epoch.load();
    uint32_t applied_periodic_interval_ms = 0xFFFFFFFFu;

    auto canTransmitNow = [&]() -> bool {
        uint32_t now_us = micros();
        return (int32_t)(now_us - next_tx_allowed_us) >= 0;
    };

    auto markTransmitted = [&]() {
        uint32_t now_us = micros();
        if (MKS_SERVO_MIN_INTERFRAME_US > 0) {
            next_tx_allowed_us = now_us + (uint32_t)MKS_SERVO_MIN_INTERFRAME_US;
        } else {
            next_tx_allowed_us = now_us;
        }
    };

    while (true) {
        uint32_t current_reset_epoch = async.runtime_reset_epoch.load();
        if (current_reset_epoch != seen_reset_epoch) {
            seen_reset_epoch = current_reset_epoch;
            ack_pending = false;
            ack_deadline_us = 0;
            ack_start_us = 0;
            ack_expected_function = 0xF6;
            ack_parser.received = 0;
            function_pending = false;
            pending_function = FunctionCommandItem{};
            next_tx_allowed_us = 0;
            applied_periodic_interval_ms = 0xFFFFFFFFu;
            async.ack_pending_time_us.store(0);
            LOG_PRINTF_TRY(abbot::log::CHANNEL_MOTOR,
                           "mks_servo: task state reset %s epoch=%lu\n",
                           (side == MotorSide::LEFT ? "LEFT" : "RIGHT"),
                           (unsigned long)current_reset_epoch);
        }

        // In Hybrid (Step/Dir) mode, we don't need 1000Hz RS485 reactivity because 
        // pulses are hardware-generated. We wait for the telemetry interval.
        // In Serial mode, we wait 1ms to process speed commands with low latency.
        uint32_t wait_ms = 1;
#if MKS_SERVO_USE_STEP_DIR
        // In hybrid Step/Dir mode, keep the motor task on a short periodic cadence
        // even if notifications are missed/coalesced. This prevents one side from
        // falling behind purely because its task wasn't explicitly woken.
        wait_ms = ack_pending ? 1 : 2;
#endif
        uint32_t wake_count = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(wait_ms));
        if (wake_count > 0) {
            async.task_notify_wake.fetch_add(1);
        } else {
            async.task_timeout_wake.fetch_add(1);
        }

        uint32_t now_ms = millis();
        if (xSemaphoreTakeRecursive(mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            // Non-blocking ACK processing (per-bus). When ACK is enabled, do not
            // interleave encoder reads/other frames until the motor confirms.
            if (ack_pending) {
                uint8_t ack_status = 0;
                if (protocol.tryConsumeAckNonBlocking((uint8_t)state.id, ack_expected_function, ack_parser, ack_status)) {
                    uint32_t now_us = micros();
                    async.ack_pending_time_us.store((uint32_t)(now_us - ack_start_us));
                    async.last_latency_us.store((uint32_t)(now_us - ack_start_us));
                    async.last_latency_age_ms.store(0);
                    if (ack_status != 0x01) {
                        async.speed_ack_error.fetch_add(1);
                        if ((millis() - state.last_ack_log_ms >= MKS_SERVO_DIAG_LOG_ms)) {
                            LOG_PRINTF(abbot::log::CHANNEL_MOTOR,
                                       "mks_servo: ID 0x%02X command error status=0x%02X\n",
                                       state.id, ack_status);
                            state.last_ack_log_ms = millis();
                        }
                    }
                    ack_pending = false;
                    async.ack_pending_time_us.store(0);
                } else {
                    uint32_t now_us = micros();
                    async.ack_pending_time_us.store((uint32_t)(now_us - ack_start_us));
                    if ((int32_t)(now_us - ack_deadline_us) >= 0) {
                        async.speed_ack_timeout.fetch_add(1);
                        async.last_latency_us.store((uint32_t)(now_us - ack_start_us));
                        async.last_latency_age_ms.store(0);
                        ack_pending = false;
                        ack_parser.received = 0;
                        async.ack_pending_time_us.store(0);
                    }
                }

                xSemaphoreGiveRecursive(mutex);
                continue;
            }

            // Function command queue (non-blocking). Prioritize configuration frames.
            if (!function_pending) {
                QueueHandle_t queue = getQueueForMotor(side);
                if (queue) {
                    if (xQueueReceive(queue, &pending_function, 0) == pdTRUE) {
                        function_pending = true;
                    }
                }
            }

            if (function_pending) {
                if (canTransmitNow()) {
                    MksServoProtocol::StatusCode send_status = MksServoProtocol::StatusCode::Ok;
                    uint8_t code = pending_function.function_code;
                    uint8_t id = pending_function.id;
                    
                    bool handled = true;
                    switch(code) {
                        case MksServoProtocol::FUNC_TORQUE_ENABLE:
                            protocol.setTorqueEnable(id, pending_function.data[0] != 0, send_status);
                            break;
                        case MksServoProtocol::FUNC_SET_MODE:
                            protocol.setMode(id, (MksServoMode)pending_function.data[0], send_status);
                            break;
                        case MksServoProtocol::FUNC_SET_BAUD:
                            protocol.setBaudRate(id, pending_function.data[0], send_status);
                            break;
                        case MksServoProtocol::FUNC_SET_RESPONSE_MODE:
                            protocol.setResponseMode(id, pending_function.data[0] != 0, pending_function.data[1] != 0, send_status);
                            break;
                        case MksServoProtocol::FUNC_SET_MSTEP:
                            protocol.setMicrostep(id, (MksServoMicrostep)pending_function.data[0], send_status);
                            break;
                        case MksServoProtocol::FUNC_SET_CURRENT:
                            protocol.setCurrent(id, (uint16_t)(((uint16_t)pending_function.data[0] << 8) | pending_function.data[1]), send_status);
                            break;
                        case MksServoProtocol::FUNC_SET_HOLD_CURRENT:
                            protocol.setHoldCurrent(id, (MksServoHoldCurrent)pending_function.data[0], send_status);
                            break;
                        case MksServoProtocol::FUNC_SET_PERIODIC:
                            protocol.setPeriodicReadParameter(id, pending_function.data[0], (uint16_t)(((uint16_t)pending_function.data[1] << 8) | pending_function.data[2]), send_status);
                            break;
                        default:
                            handled = false;
                            break;
                    }

                    if (!handled) {
                        protocol.sendFunctionCommand(id, code, pending_function.data, pending_function.length, send_status);
                    }

                    markTransmitted();

                    if (m_wait_for_ack.load() && pending_function.function_code >= 0x80) {
                        uint32_t tx_start_us = micros();
                        ack_pending = true;
                        ack_expected_function = pending_function.function_code;
                        ack_start_us = tx_start_us;
                        ack_deadline_us = tx_start_us + (uint32_t)MKS_SERVO_TIMEOUT_CONTROL_US;
                        ack_parser.received = 0;
                        async.ack_pending_time_us.store(0);
                    }
                    function_pending = false;
                }

                xSemaphoreGiveRecursive(mutex);
                continue;
            }

            bool current_enabled = m_enabled.load();
            bool enabled_changed = (current_enabled != async.last_reported_enabled.load());
            bool speed_dirty = async.speed_dirty.exchange(false);
            
            // 1. Handle Torque State Changes
            if (enabled_changed) {
                if (canTransmitNow()) {
                    bool target_en = current_enabled;

                    MksServoProtocol::StatusCode status;
                    protocol.setTorqueEnable(state.id, target_en, status);
                    markTransmitted();

                    // Update physical EN pins (Now mandatory with Step/Dir hybrid)
                    setHardwareEnablePin(side, target_en, async.configured_enable_level.load());

                    async.last_reported_enabled.store(target_en);
                    speed_dirty = true;
                } else {
                    // Try again on next tick; keep last_reported_enabled unchanged.
                }
            }

            // 1b. Apply desired periodic telemetry state from within the motor task
            // so ordering stays deterministic relative to torque enable/disable.
            const uint32_t desired_periodic_interval_ms =
                async.desired_periodic_interval_ms.load();
            if (desired_periodic_interval_ms != applied_periodic_interval_ms) {
                const bool can_disable_periodic = (desired_periodic_interval_ms == 0u);
                const bool can_enable_periodic =
                    current_enabled && !enabled_changed &&
                    (async.last_reported_enabled.load() == current_enabled);
                if ((can_disable_periodic || can_enable_periodic) &&
                    canTransmitNow()) {
                    MksServoProtocol::StatusCode status =
                        MksServoProtocol::StatusCode::Ok;
                    protocol.setPeriodicReadParameter(
                        state.id,
                        MksServoProtocol::FUNC_READ_TELEMETRY,
                        (uint16_t)desired_periodic_interval_ms,
                        status);
                    markTransmitted();
                    applied_periodic_interval_ms = desired_periodic_interval_ms;
                    async.last_telemetry_ms = now_ms;
                    LOG_PRINTF_TRY(
                        abbot::log::CHANNEL_MOTOR,
                        "mks_servo: periodic applied %s id=0x%02X interval=%lu ms\n",
                        (side == MotorSide::LEFT ? "LEFT" : "RIGHT"),
                        state.id,
                        (unsigned long)desired_periodic_interval_ms);
                }
            }

            // 2. Send Speed Command
#if MKS_SERVO_USE_STEP_DIR
            // In hybrid Step/Dir mode, apply hardware pulse updates from the dedicated
            // motor task instead of the IMU consumer thread. This keeps any potentially
            // blocking peripheral interaction off the 1kHz fusion/control path.
            bool should_poll_step_dir = speed_dirty || enabled_changed || current_enabled ||
                                        (async.last_ledc_freq != 0U);
            if (should_poll_step_dir) {
                float cmd = current_enabled ? async.target_speed.load() : 0.0f;
                async.step_poll_count.fetch_add(1);
                applyStepDirCommand(side, cmd, current_enabled);
                if (speed_dirty || enabled_changed) {
                    async.speed_cmd_sent.fetch_add(1);
                }
            }
#else
            if (speed_dirty || (enabled_changed && current_enabled)) {
                float cmd = current_enabled ? async.target_speed.load() : 0.0f;
                // SPEED ACK: keep disabled during balancing; enable only for diagnostics.
                // Waiting for ACK adds latency and can reduce effective control bandwidth.
                bool waitAck = m_wait_for_ack.load();
                if (canTransmitNow()) {
                    uint32_t tx_start_us = micros();
                    sendSpeedCommand(side, cmd, state.invert, waitAck);
                    markTransmitted();

                    if (waitAck) {
                        ack_pending = true;
                        ack_expected_function = 0xF6;
                        ack_start_us = tx_start_us;
                        ack_deadline_us = tx_start_us + (uint32_t)MKS_SERVO_TIMEOUT_CONTROL_US;
                        ack_parser.received = 0;
                    } else {
                        async.last_latency_us.store((uint32_t)(micros() - tx_start_us));
                        async.last_latency_age_ms.store(0);
                    }
                } else {
                    // Could not transmit yet; keep the command pending.
                    async.speed_dirty.store(true);
                }
            } 
#endif
            else if (!current_enabled) {
                // Occasional safety zeroing
                uint32_t now_ms = millis();
                if (now_ms - async.last_zero_ms > 200) {
                    if (canTransmitNow()) {
                        sendSpeedCommand(side, 0.0f, state.invert, m_wait_for_ack.load());
                        markTransmitted();
                        async.last_zero_ms = now_ms;
                    }
                }
            }

            // 3. Periodic telemetry parsing (no polling)
#if MKS_SERVO_TELEMETRY_ENABLED
            MksServoTelemetryIngest::TelemetrySample sample;
            while (serial.available() > 0) {
                uint8_t byte_value = (uint8_t)serial.read();
                if (telemetryIngest.ingestByte(byte_value, sample)) {
                    uint32_t now_us = (uint32_t)esp_timer_get_time();
                    uint32_t last_pkt_us = async.last_encoder_time_us.load();
                    
                    // BUS LATENCY: Measure time between packets
                    if (last_pkt_us > 0) {
                        uint32_t delta_us = now_us - last_pkt_us;
                        async.last_latency_us.store(delta_us);
                        async.last_latency_age_ms.store(0);
                    }
                    
                    async.speed_value.store(sample.speed);
                    async.encoder_value.store(sample.position);
                    async.encoder_dirty.store(true);
                    async.last_encoder_time_us.store(now_us);
                    
                    // We use a small deadband/threshold for the speed used in interpolation
                    // to prevent "fuzz" when the robot is nearly still.
                    float interp_speed = (abs(sample.speed) < 10.0f) ? 0.0f : sample.speed;
                    async.interpolator.update(sample.position, interp_speed, now_us);
                    
                    async.encoder_ok.fetch_add(1);
                }
            }

            if (current_enabled && desired_periodic_interval_ms > 0) {
                uint32_t enc_age_ms = getLastEncoderAgeMs(side);
                if (enc_age_ms > 250 &&
                    (now_ms - async.last_telemetry_ms) >= 1000 &&
                    canTransmitNow()) {
                    MksServoProtocol::StatusCode status = MksServoProtocol::StatusCode::Ok;
                    protocol.setPeriodicReadParameter(
                        state.id,
                        MksServoProtocol::FUNC_READ_TELEMETRY,
                        (uint16_t)desired_periodic_interval_ms,
                        status);
                    markTransmitted();
                    async.last_telemetry_ms = now_ms;
                    async.periodic_rearm.fetch_add(1);
                    LOG_PRINTF_TRY(abbot::log::CHANNEL_MOTOR,
                                   "mks_servo: rearm telemetry %s id=0x%02X enc_age=%lu ms\n",
                                   (side == MotorSide::LEFT ? "LEFT" : "RIGHT"),
                                   state.id,
                                   (unsigned long)enc_age_ms);
                }
            }

#endif
            xSemaphoreGiveRecursive(mutex);
        } else {
            async.bus_mutex_miss.fetch_add(1);
        }

        // Cooperative yield to avoid Watchdog triggers, especially if telemetry is failing.
        vTaskDelay(1);
    }
}

MksServoProtocol& MksServoMotorDriver::getProtocolForMotor(MotorSide side) {
    if (side == MotorSide::LEFT) {
        return *m_leftProtocol;
    }
    return *m_rightProtocol;
}

MksServoProtocol& MksServoMotorDriver::getProtocolForSide(MotorSide side) {
    if (side == MotorSide::LEFT) {
        return *m_leftProtocol;
    }
    return *m_rightProtocol;
}

MksServoTelemetryIngest& MksServoMotorDriver::getTelemetryIngestForSide(MotorSide side) {
    if (side == MotorSide::LEFT) {
        return *m_leftTelemetryIngest;
    }
    return *m_rightTelemetryIngest;
}

void MksServoMotorDriver::resetRuntimeStateForSide(MotorSide side, bool target_enabled) {
    AsyncState& async = (side == MotorSide::LEFT) ? m_left_async : m_right_async;
    MotorState& state = (side == MotorSide::LEFT) ? m_left : m_right;
    HardwareSerial& serial = (side == MotorSide::LEFT) ? Serial2 : Serial1;
    MksServoTelemetryIngest* ingest = (side == MotorSide::LEFT) ? m_leftTelemetryIngest : m_rightTelemetryIngest;
    QueueHandle_t queue = getQueueForMotor(side);
    SemaphoreHandle_t mutex = getMutexForMotor(side);
    const uint32_t now_us = (uint32_t)esp_timer_get_time();

    if (queue) {
        xQueueReset(queue);
    }

    if (mutex && xSemaphoreTakeRecursive(mutex, pdMS_TO_TICKS(20)) == pdTRUE) {
        while (serial.available() > 0) {
            serial.read();
        }

        if (ingest) {
            ingest->reset();
        }

        xSemaphoreGiveRecursive(mutex);
    } else if (ingest) {
        ingest->reset();
    }

    async.target_speed.store(0.0f);
    async.speed_dirty.store(true);
    // NOTE: encoder_value is intentionally NOT reset to 0 here.
    // The MKS servo encoder is absolute (hardware position persists between runs).
    // Resetting to 0 creates a spurious "dist_err" jump when the next telemetry
    // packet arrives with the real hardware position, causing immediate LQR saturation.
    // The stale guard (last_encoder_time_us = 0 below) is sufficient to mark data as untrustworthy.
    async.speed_value.store(0.0f);
    async.encoder_dirty.store(false);
    async.last_latency_us.store(0);
    async.last_latency_age_ms.store(0);
    async.ack_pending_time_us.store(0);
    async.last_cmd_send_time_us.store(0);
    async.last_cmd_update_time_us.store(now_us);
    async.last_encoder_time_us.store(0);
    async.last_reported_enabled.store(!target_enabled);
    async.step_freq_requested.store(0);
    async.configured_enable_level.store(0);
    async.desired_periodic_interval_ms.store(0xFFFFFFFFu);
    async.interpolator.reset();
    async.last_zero_ms = 0;
    async.last_ledc_freq = 0;
    async.last_ledc_update_us = 0;
    async.last_telemetry_ms = 0;

    // Reset per-run diagnostic counters so each run's stats are visible independently.
    async.speed_cmd_sent.store(0);
    async.speed_ack_timeout.store(0);
    async.speed_ack_error.store(0);
    async.encoder_ok.store(0);
    async.encoder_timeout.store(0);
    async.bus_mutex_miss.store(0);
    async.step_apply_ok.store(0);
    async.step_mutex_miss.store(0);
    async.step_poll_count.store(0);
    async.task_notify_wake.store(0);
    async.task_timeout_wake.store(0);
    async.function_queue_full.store(0);
    async.periodic_rearm.store(0);

    async.runtime_reset_epoch.fetch_add(1);

    state.last_command = 0.0f;
    state.enabled = target_enabled;
    state.last_encoder = 0;
    state.last_command_time_us = 0;
    state.last_encoder_read_ms = 0;
    state.speedEstimator.reset();

#if MKS_SERVO_USE_STEP_DIR
    if (!m_stepGenerator || !m_stepGenerator->setFrequency(side == MotorSide::LEFT, 0)) {
        async.step_mutex_miss.fetch_add(1);
    }
#endif
}

void MksServoMotorDriver::queuePeriodicTelemetry(MotorSide side, uint8_t code, uint16_t interval_ms) {
#if MKS_SERVO_TELEMETRY_ENABLED
    AsyncState& async = (side == MotorSide::LEFT) ? m_left_async : m_right_async;
    uint32_t previous_interval = async.desired_periodic_interval_ms.load();
    if (previous_interval == (uint32_t)interval_ms) {
        return;
    }
    async.desired_periodic_interval_ms.store((uint32_t)interval_ms);
    TaskHandle_t h = (side == MotorSide::LEFT) ? m_left_async.task_handle : m_right_async.task_handle;
    if (h) {
        xTaskNotifyGive(h);
    }
#else
    (void)side;
    (void)code;
    (void)interval_ms;
#endif
}


void MksServoMotorDriver::clearCommandState() {
    m_left.last_command = 0.0f;
    m_right.last_command = 0.0f;
    setMotorCommandBoth(0.0f, 0.0f);
#if MKS_SERVO_USE_STEP_DIR
    if (m_stepGenerator) {
        m_stepGenerator->stopAll();
    }
    resetStepGeneratorState();
#endif
}

float MksServoMotorDriver::getLastMotorCommand(MotorSide side) {
    return (side == MotorSide::LEFT) ? m_left.last_command : m_right.last_command;
}

void MksServoMotorDriver::enableMotors() {
    if (m_enabled.load()) {
        LOG_PRINTLN(abbot::log::CHANNEL_MOTOR, "mks_servo: enable requested while already enabled; forcing motor state resync");
    } else {
        LOG_PRINTLN(abbot::log::CHANNEL_MOTOR, "mks_servo: enabling motors (Queueing Torque ON)");
    }
    m_enabled.store(true);
    resetRuntimeStateForSide(MotorSide::LEFT, true);
    resetRuntimeStateForSide(MotorSide::RIGHT, true);
    setHardwareEnablePin(MotorSide::LEFT, true, m_left_async.configured_enable_level.load());
    setHardwareEnablePin(MotorSide::RIGHT, true, m_right_async.configured_enable_level.load());

    uint16_t interval = (uint16_t)MKS_SERVO_PERIODIC_TELEMETRY_MS;
    queuePeriodicTelemetry(MotorSide::LEFT, MksServoProtocol::FUNC_READ_TELEMETRY, interval);
    queuePeriodicTelemetry(MotorSide::RIGHT, MksServoProtocol::FUNC_READ_TELEMETRY, interval);
    
    // Notify background tasks to perform Torque ON and initial speed sync
    if (m_left_async.task_handle) {
        xTaskNotifyGive(m_left_async.task_handle);
    }
    if (m_right_async.task_handle) {
        xTaskNotifyGive(m_right_async.task_handle);
    }
}

void MksServoMotorDriver::disableMotors() {
    if (!m_enabled.load()) {
        LOG_PRINTLN(abbot::log::CHANNEL_MOTOR, "mks_servo: disable requested while already disabled; forcing motor state reset");
    } else {
        LOG_PRINTLN(abbot::log::CHANNEL_MOTOR, "mks_servo: disabling motors (Queueing Torque OFF)");
    }
    m_enabled.store(false);
    resetRuntimeStateForSide(MotorSide::LEFT, false);
    resetRuntimeStateForSide(MotorSide::RIGHT, false);
    setHardwareEnablePin(MotorSide::LEFT, false, m_left_async.configured_enable_level.load());
    setHardwareEnablePin(MotorSide::RIGHT, false, m_right_async.configured_enable_level.load());

#if MKS_SERVO_USE_STEP_DIR
    if (m_stepGenerator) {
        m_stepGenerator->stopAll();
    }
    resetStepGeneratorState();
#endif

    queuePeriodicTelemetry(MotorSide::LEFT, MksServoProtocol::FUNC_READ_TELEMETRY, 0);
    queuePeriodicTelemetry(MotorSide::RIGHT, MksServoProtocol::FUNC_READ_TELEMETRY, 0);

    // Notify background tasks to perform Torque OFF
    if (m_left_async.task_handle) {
        xTaskNotifyGive(m_left_async.task_handle);
    }
    if (m_right_async.task_handle) {
        xTaskNotifyGive(m_right_async.task_handle);
    }
}

uint32_t MksServoMotorDriver::getLastBusLatencyUs() const {
    uint32_t l = m_left_async.last_latency_us.load();
    uint32_t r = m_right_async.last_latency_us.load();
    return (l > r) ? l : r;
}

uint32_t MksServoMotorDriver::getLastBusLatencyUs(MotorSide side) const {
    if (side == MotorSide::LEFT) {
        return m_left_async.last_latency_us.load();
    }
    return m_right_async.last_latency_us.load();
}

uint32_t MksServoMotorDriver::getLastBusLatencyAgeMs(MotorSide side) const {
    if (side == MotorSide::LEFT) {
        return m_left_async.last_latency_age_ms.load();
    }
    return m_right_async.last_latency_age_ms.load();
}

uint32_t MksServoMotorDriver::getAckPendingTimeUs() const {
    uint32_t l = m_left_async.ack_pending_time_us.load();
    uint32_t r = m_right_async.ack_pending_time_us.load();
    return (l > r) ? l : r;
}

uint32_t MksServoMotorDriver::getAckPendingTimeUs(MotorSide side) const {
    if (side == MotorSide::LEFT) {
        return m_left_async.ack_pending_time_us.load();
    }
    return m_right_async.ack_pending_time_us.load();
}

uint32_t MksServoMotorDriver::getAppliedStepFrequencyHz(MotorSide side) const {
    if (side == MotorSide::LEFT) {
        return m_left_async.last_ledc_freq;
    }
    return m_right_async.last_ledc_freq;
}

uint32_t MksServoMotorDriver::getLastEncoderAgeMs(MotorSide side) const {
    const AsyncState& async = (side == MotorSide::LEFT) ? m_left_async : m_right_async;
    uint32_t last_us = async.last_encoder_time_us.load();
    if (last_us == 0) {
        return 4294967; // approx max to indicate no data
    }

    uint32_t now_us = (uint32_t)esp_timer_get_time();
    if (now_us <= last_us) {
        return 0;
    }

    uint32_t age_us = now_us - last_us;
    return age_us / 1000;
}

bool MksServoMotorDriver::areMotorsEnabled() {
    return m_enabled.load();
}

void MksServoMotorDriver::printStatus() {
    uint32_t lat_us = getLastBusLatencyUs();
    LOG_PRINTF(abbot::log::CHANNEL_MOTOR,
               "mks_servo: enabled=%d ack=%d accel=%u stepgen=%s bus_lat_us=%lu L_cmd=%.3f R_cmd=%.3f\n",
               (int)m_enabled.load(), (int)m_wait_for_ack.load(), (unsigned)m_speed_accel.load(),
               stepGeneratorModeName(getStepGeneratorMode()), (unsigned long)lat_us,
               (double)m_left.last_command, (double)m_right.last_command);
    LOG_PRINTF(abbot::log::CHANNEL_MOTOR,
               "mks_servo: L sent=%lu ack_to=%lu ack_err=%lu enc_ok=%lu enc_to=%lu | R sent=%lu ack_to=%lu ack_err=%lu enc_ok=%lu enc_to=%lu\n",
               (unsigned long)m_left_async.speed_cmd_sent.load(),
               (unsigned long)m_left_async.speed_ack_timeout.load(),
               (unsigned long)m_left_async.speed_ack_error.load(),
               (unsigned long)m_left_async.encoder_ok.load(),
               (unsigned long)m_left_async.encoder_timeout.load(),
               (unsigned long)m_right_async.speed_cmd_sent.load(),
               (unsigned long)m_right_async.speed_ack_timeout.load(),
               (unsigned long)m_right_async.speed_ack_error.load(),
               (unsigned long)m_right_async.encoder_ok.load(),
               (unsigned long)m_right_async.encoder_timeout.load());
    LOG_PRINTF(abbot::log::CHANNEL_MOTOR,
               "mks_servo: L step_ok=%lu step_miss=%lu req_freq=%lu poll=%lu notify=%lu timeout=%lu bus_miss=%lu qfull=%lu rearm=%lu periodic=%lu | R step_ok=%lu step_miss=%lu req_freq=%lu poll=%lu notify=%lu timeout=%lu bus_miss=%lu qfull=%lu rearm=%lu periodic=%lu\n",
               (unsigned long)m_left_async.step_apply_ok.load(),
               (unsigned long)m_left_async.step_mutex_miss.load(),
               (unsigned long)m_left_async.step_freq_requested.load(),
               (unsigned long)m_left_async.step_poll_count.load(),
               (unsigned long)m_left_async.task_notify_wake.load(),
               (unsigned long)m_left_async.task_timeout_wake.load(),
               (unsigned long)m_left_async.bus_mutex_miss.load(),
               (unsigned long)m_left_async.function_queue_full.load(),
               (unsigned long)m_left_async.periodic_rearm.load(),
               (unsigned long)m_left_async.desired_periodic_interval_ms.load(),
               (unsigned long)m_right_async.step_apply_ok.load(),
               (unsigned long)m_right_async.step_mutex_miss.load(),
               (unsigned long)m_right_async.step_freq_requested.load(),
               (unsigned long)m_right_async.step_poll_count.load(),
               (unsigned long)m_right_async.task_notify_wake.load(),
               (unsigned long)m_right_async.task_timeout_wake.load(),
               (unsigned long)m_right_async.bus_mutex_miss.load(),
               (unsigned long)m_right_async.function_queue_full.load(),
               (unsigned long)m_right_async.periodic_rearm.load(),
               (unsigned long)m_right_async.desired_periodic_interval_ms.load());

    unsigned long left_stack = 0;
    unsigned long right_stack = 0;
    int left_state = -1;
    int right_state = -1;
    if (m_left_async.task_handle) {
        left_stack = (unsigned long)uxTaskGetStackHighWaterMark(m_left_async.task_handle);
        left_state = (int)eTaskGetState(m_left_async.task_handle);
    }
    if (m_right_async.task_handle) {
        right_stack = (unsigned long)uxTaskGetStackHighWaterMark(m_right_async.task_handle);
        right_state = (int)eTaskGetState(m_right_async.task_handle);
    }

    LOG_PRINTF(abbot::log::CHANNEL_MOTOR,
               "mks_servo: L tgt=%.3f cmd=%.3f freq=%lu en_sync=%d enc_age=%lums task_state=%d stack_hw=%lu | R tgt=%.3f cmd=%.3f freq=%lu en_sync=%d enc_age=%lums task_state=%d stack_hw=%lu\n",
               (double)m_left_async.target_speed.load(),
               (double)m_left.last_command,
               (unsigned long)m_left_async.last_ledc_freq,
               (int)m_left_async.last_reported_enabled.load(),
               (unsigned long)getLastEncoderAgeMs(MotorSide::LEFT),
               left_state,
               left_stack,
               (double)m_right_async.target_speed.load(),
               (double)m_right.last_command,
               (unsigned long)m_right_async.last_ledc_freq,
               (int)m_right_async.last_reported_enabled.load(),
               (unsigned long)getLastEncoderAgeMs(MotorSide::RIGHT),
               right_state,
               right_stack);
}

void MksServoMotorDriver::dumpConfig() {
    LOG_PRINTF(abbot::log::CHANNEL_MOTOR,
               "mks_servo: L_ID=0x%02X R_ID=0x%02X Baud=%d Ma=%d vmax_rpm=%d accel=%u\n",
               m_left.id, m_right.id, MKS_SERVO_BAUD, MKS_SERVO_MA, (int)VELOCITY_MAX_SPEED, (unsigned)m_speed_accel.load());
}

void MksServoMotorDriver::setMotorCommandBoth(float left_command, float right_command) {
    uint32_t now_cmd_us = (uint32_t)esp_timer_get_time();

    // 1. Update targets (Atomics for background tasks telemetry/config)
    m_left_async.target_speed.store(left_command);
    m_left_async.speed_dirty.store(true);
    m_left_async.last_cmd_update_time_us.store(now_cmd_us);
    
    m_right_async.target_speed.store(right_command);
    m_right_async.speed_dirty.store(true);
    m_right_async.last_cmd_update_time_us.store(now_cmd_us);

    m_left.last_command = left_command;
    m_right.last_command = right_command;

    // Notify background tasks to process RS485 and, in hybrid mode, apply Step/Dir
    // hardware updates off the IMU consumer thread.
    if (m_left_async.task_handle) {
        xTaskNotifyGive(m_left_async.task_handle);
    }
    if (m_right_async.task_handle) {
        xTaskNotifyGive(m_right_async.task_handle);
    }

    // Update legacy state for monitoring/logging
    m_left.last_command = left_command;
    m_right.last_command = right_command;
}

void MksServoMotorDriver::readEncodersBoth(int32_t& left_out, int32_t& right_out) {
    // Return interpolated values to smooth out LQR control loop
    left_out = getInterpolatedEncoder(m_left_async);
    right_out = getInterpolatedEncoder(m_right_async);
}

int32_t MksServoMotorDriver::getInterpolatedEncoder(const AsyncState& async) const {
    uint32_t now_us = (uint32_t)esp_timer_get_time();
    return async.interpolator.getInterpolated(now_us, (uint32_t)MKS_SERVO_ENCODER_EXTRAPOLATE_MS * 1000UL);
}

void MksServoMotorDriver::setMotorCommand(MotorSide side, float command) {
    uint32_t now_cmd_us = (uint32_t)esp_timer_get_time();

    if (side == MotorSide::LEFT) {
        m_left_async.target_speed.store(command);
        m_left_async.speed_dirty.store(true);
        m_left_async.last_cmd_update_time_us.store(now_cmd_us);
#if !MKS_SERVO_USE_STEP_DIR
        if (m_left_async.task_handle) {
            xTaskNotifyGive(m_left_async.task_handle);
        }
#endif
        m_left.last_command = command;
    } else {
        m_right_async.target_speed.store(command);
        m_right_async.speed_dirty.store(true);
        m_right_async.last_cmd_update_time_us.store(now_cmd_us);
#if !MKS_SERVO_USE_STEP_DIR
        if (m_right_async.task_handle) {
            xTaskNotifyGive(m_right_async.task_handle);
        }
#endif
        m_right.last_command = command;
    }

    TaskHandle_t h = (side == MotorSide::LEFT) ? m_left_async.task_handle : m_right_async.task_handle;
    if (h) {
        xTaskNotifyGive(h);
    }
}

void MksServoMotorDriver::setMotorCommandRaw(MotorSide side, int16_t rawSpeed) {
    // Treat raw speed as normalized units / 1000
    float normalized = (float)rawSpeed / 1000.0f;
    setMotorCommand(side, normalized);
}

int32_t MksServoMotorDriver::readEncoder(MotorSide side) {
    return getInterpolatedEncoder((side == MotorSide::LEFT) ? m_left_async : m_right_async);
}

float MksServoMotorDriver::readSpeed(MotorSide side) {
    // Return velocity estimator result from atomic async state (stable core transfer)
    return (side == MotorSide::LEFT) ? m_left_async.speed_value.load() : m_right_async.speed_value.load();
}

void MksServoMotorDriver::resetSpeedEstimator() {
    m_left.speedEstimator.reset();
    m_right.speedEstimator.reset();
    LOG_PRINTLN(abbot::log::CHANNEL_MOTOR, "mks_servo: speed estimators reset");
}

void MksServoMotorDriver::resetPositionTracking() {
    // For MKS, we track absolute position, but we can clear our local cache
    m_left_async.encoder_value.store(0);
    m_right_async.encoder_value.store(0);
}

uint64_t MksServoMotorDriver::getLastCommandTimeUs(MotorSide side) const {
    // In Step/Dir hybrid mode, the meaningful command freshness is the last
    // control-side target update, not the age of the last RS485 speed frame.
    // In pure serial mode, keep returning the transport timestamp.
#if MKS_SERVO_USE_STEP_DIR
    return (side == MotorSide::LEFT) ? m_left_async.last_cmd_update_time_us.load()
                                    : m_right_async.last_cmd_update_time_us.load();
#else
    return (side == MotorSide::LEFT) ? m_left_async.last_cmd_send_time_us.load()
                                    : m_right_async.last_cmd_send_time_us.load();
#endif
}

int MksServoMotorDriver::getMotorId(MotorSide side) const {
    return (side == MotorSide::LEFT) ? m_left.id : m_right.id;
}

bool MksServoMotorDriver::isMotorInverted(MotorSide side) const {
    return (side == MotorSide::LEFT) ? m_left.invert : m_right.invert;
}

float MksServoMotorDriver::getVelocityMaxSpeed() const {
    return (float)VELOCITY_MAX_SPEED;
}

float MksServoMotorDriver::getVelocityTargetIncrementScale() const {
    return 100.0f; 
}

float MksServoMotorDriver::getVelocityPositionKp() const {
    return 10.0f;
}

void MksServoMotorDriver::dumpAllConfigs() {
    LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "mks_servo: --- DEPLOYED CONFIGURATION DUMP ---");
    MotorSide sides[] = {MotorSide::LEFT, MotorSide::RIGHT};
    const char* names[] = {"LEFT", "RIGHT"};
    
    for (int i = 0; i < 2; ++i) {
        MotorSide side = sides[i];
        uint8_t id = getMotorId(side);
        SemaphoreHandle_t mutex = getMutexForMotor(side);
        HardwareSerial& serial = getSerialForMotor(side);
        
        if (xSemaphoreTakeRecursive(mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
            LOG_PRINTF(abbot::log::CHANNEL_DEFAULT, "  [ID 0x%02X] Skipping dump: bus busy\n", id);
            continue;
        }

        MksServoProtocol& protocol = getProtocolForSide(side);
        MksServoProtocol::StatusCode status;
        uint8_t params[34];

        LOG_PRINTF(abbot::log::CHANNEL_DEFAULT, "mks_servo: Reading ID 0x%02X (%s)...\n", id, names[i]);

        // Clear RX
        while(serial.available()) serial.read();

        bool success = protocol.readAllConfigParameters(id, params, sizeof(params), 50000, status);
        xSemaphoreGiveRecursive(mutex);

        if (success) {
            AsyncState& async = (side == MotorSide::LEFT) ? m_left_async : m_right_async;
            const uint32_t steps_per_rev = stepsPerRevFromConfigByte(params[Config47Index::kMicrostep]);
            const uint16_t operating_current_ma =
                ((uint16_t)params[Config47Index::kOperatingCurrentHi] << 8) |
                (uint16_t)params[Config47Index::kOperatingCurrentLo];
            async.configured_steps_per_rev.store(steps_per_rev);
            async.configured_enable_level.store(params[Config47Index::kEnableEffectiveLevel]);
            LOG_PRINTF(abbot::log::CHANNEL_DEFAULT, "  [ID 0x%02X] Mode: %d, MStep: %d, Current: %d mA, Hold: %d%%, EN: %s\n",
                       id,
                       params[Config47Index::kWorkMode],
                       params[Config47Index::kMicrostep],
                       (int)operating_current_ma,
                       params[Config47Index::kHoldingCurrent] * 10,
                       enableLevelName(params[Config47Index::kEnableEffectiveLevel]));
            LOG_PRINTF(abbot::log::CHANNEL_DEFAULT, "  [ID 0x%02X] Effective STEP scaling: %lu steps/rev\n",
                       id, (unsigned long)steps_per_rev);
            LOG_PRINTF(abbot::log::CHANNEL_DEFAULT, "  [ID 0x%02X] Response mode bytes: %u %u\n",
                       id,
                       (unsigned)params[Config47Index::kResponseMethod1],
                       (unsigned)params[Config47Index::kResponseMethod2]);
            LOG_PRINTF(abbot::log::CHANNEL_DEFAULT, "  [ID 0x%02X] KP: %d, KI: %d, KD: %d\n",
                       id, (params[6] << 8) | params[7], (params[8] << 8) | params[9], (params[10] << 8) | params[11]);
        } else {
            LOG_PRINTF(abbot::log::CHANNEL_DEFAULT, "  [ID 0x%02X] FAILED to read config (Status: %d)\n", id, (int)status);
        }
    }
    LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "mks_servo: ----------------------------------");
}

void MksServoMotorDriver::calibrateMotor(uint8_t id) {
    LOG_PRINTF(abbot::log::CHANNEL_MOTOR, "mks_servo: starting calibration for ID 0x%02X\n", id);
    // Explicitly send to both buses to handle duplicate IDs across Serial1/Serial2
    sendFunctionCommand(MotorSide::LEFT, MksServoProtocol::FUNC_CALIBRATE_ENCODER, nullptr, 0);
    sendFunctionCommand(MotorSide::RIGHT, MksServoProtocol::FUNC_CALIBRATE_ENCODER, nullptr, 0);
}

void MksServoMotorDriver::scanBus() {
    LOG_PRINTLN(abbot::log::CHANNEL_MOTOR, "mks_servo: starting scan on both Serial1 and Serial2 (IDs 1-15)...");
    int found_count = scanBusOnCurrentBaud();
    LOG_PRINTF(abbot::log::CHANNEL_MOTOR, "mks_servo: scan finished. Found %d motors total.", found_count);
}

int MksServoMotorDriver::scanBusOnCurrentBaud() {
    int found_count = 0;
    MotorSide sides[] = {MotorSide::RIGHT, MotorSide::LEFT};
    const char* port_names[] = {"RIGHT-BUS (Serial1)", "LEFT-BUS (Serial2)"};

    for (int i = 0; i < 2; ++i) {
        MotorSide side = sides[i];
        SemaphoreHandle_t mutex = getMutexForMotor(side);
        HardwareSerial& serial = getSerialForMotor(side);
        MksServoProtocol& protocol = getProtocolForSide(side);

        LOG_PRINTF(abbot::log::CHANNEL_MOTOR, "mks_servo: Scanning %s...\n", port_names[i]);

        for (uint8_t id = 1; id <= 10; ++id) {
            if (xSemaphoreTakeRecursive(mutex, pdMS_TO_TICKS(50)) != pdTRUE) {
                continue;
            }

            // Clear any leftover data in RX buffer
            while (serial.available() > 0) {
                serial.read();
            }

            // Send read Enabled status request (0x3A) - it's a very safe non-destructive read 
            // and we know this motor version responds to it with a 5-byte FB frame.
            MksServoProtocol::StatusCode status = MksServoProtocol::StatusCode::Ok;
            protocol.sendReadOnlyRequest(id, MksServoProtocol::FUNC_READ_ENABLED, status);

            uint8_t responseBuffer[16];
            uint8_t expected_len = MksServoProtocol::getExpectedResponseLength(MksServoProtocol::FUNC_READ_ENABLED);

            bool success = protocol.readResponse(id,
                                                 MksServoProtocol::FUNC_READ_ENABLED,
                                                 responseBuffer,
                                                 expected_len,
                                                 MKS_SERVO_TIMEOUT_HEAVY_US); // 10ms for scan, per-ID
            xSemaphoreGiveRecursive(mutex);

            if (success) {
                LOG_PRINTF(abbot::log::CHANNEL_MOTOR,
                           "mks_servo: Found motor at ID 0x%02X on %s\n",
                           id,
                           port_names[i]);
                found_count++;
            }
            // Small delay to not overwhelm the bus
            delay(5);
        }
    }

    return found_count;
}

bool MksServoMotorDriver::processSerialCommand(const String &line) {
    // Ack toggle for diagnostics. Example: "MOTOR ACK ON" or "MOTOR ACK OFF".
    // Note: the command handler strips the leading "MOTOR " and passes just "ACK ..." here.
    if (line.equalsIgnoreCase("ACK") || line.startsWith("ACK ") ||
        line.equalsIgnoreCase("MOTOR ACK") || line.startsWith("MOTOR ACK ")) {
        String s = line;
        s.replace("MOTOR", "");
        s.trim();
        // s is now "ACK" or "ACK <arg>"
        int sp = s.indexOf(' ');
        bool hasArg = (sp >= 0);
        String arg = hasArg ? s.substring(sp + 1) : String();
        arg.trim();
        arg.toUpperCase();
        if (!hasArg || arg.length() == 0) {
            LOG_PRINTF(abbot::log::CHANNEL_MOTOR, "mks_servo: ACK wait is %s\n", m_wait_for_ack.load() ? "ON" : "OFF");
            return true;
        }
        bool enable = false;
        if (arg == "ON" || arg == "1" || arg == "TRUE") {
            enable = true;
        } else if (arg == "OFF" || arg == "0" || arg == "FALSE") {
            enable = false;
        } else {
            LOG_PRINTLN(abbot::log::CHANNEL_MOTOR, "mks_servo: Usage: MOTOR ACK <ON|OFF>");
            return true;
        }
        m_wait_for_ack.store(enable);
        LOG_PRINTF(abbot::log::CHANNEL_MOTOR, "mks_servo: ACK wait set to %s\n", enable ? "ON" : "OFF");
        return true;
    }

    if (line.equalsIgnoreCase("SCAN") || line.startsWith("SCAN ") ||
        line.equalsIgnoreCase("MOTOR SCAN") || line.startsWith("MOTOR SCAN ")) {
        scanBus();
        return true;
    }

    if (line.equalsIgnoreCase("STEPGEN") || line.startsWith("STEPGEN ") ||
        line.equalsIgnoreCase("MOTOR STEPGEN") || line.startsWith("MOTOR STEPGEN ")) {
        String s = line;
        s.replace("MOTOR", "");
        s.trim();
        if (s.startsWith("STEPGEN")) {
            s = s.substring(7);
        }
        s.trim();

        if (s.length() == 0 || s.equalsIgnoreCase("SHOW")) {
            LOG_PRINTF(abbot::log::CHANNEL_DEFAULT,
                       "mks_servo: STEPGEN=%s (0=MCPWM 1=RMT 2=SOFTWARE)\n",
                       stepGeneratorModeName(getStepGeneratorMode()));
            return true;
        }

        StepGeneratorMode mode = getStepGeneratorMode();
        if (!parseStepGeneratorModeToken(s, mode)) {
            LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT,
                        "mks_servo: Usage: MOTOR STEPGEN [SHOW|MCPWM|RMT|SOFTWARE]");
            return true;
        }

        (void)setStepGeneratorMode(mode, false);
        return true;
    }

    if (line.equalsIgnoreCase("TELEMETRY") || line.startsWith("TELEMETRY ") ||
        line.equalsIgnoreCase("MOTOR TELEMETRY") || line.startsWith("MOTOR TELEMETRY ")) {
        String s = line;
        s.replace("MOTOR", "");
        s.trim();
        if (s.startsWith("TELEMETRY")) {
            s = s.substring(9);
        }
        s.trim();
        int sp = s.indexOf(' ');
        if (sp < 0) {
            LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT,
                        "mks_servo: Usage: MOTOR TELEMETRY <LEFT|RIGHT|ALL> <ms>");
            return true;
        }
        String side_token = s.substring(0, sp);
        String interval_token = s.substring(sp + 1);
        side_token.trim();
        interval_token.trim();
        side_token.toUpperCase();

        int interval_ms = interval_token.toInt();
        if (interval_ms < 0) {
            LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT,
                        "mks_servo: TELEMETRY interval must be >= 0");
            return true;
        }

        if (side_token == "LEFT") {
            queuePeriodicTelemetry(MotorSide::LEFT, MksServoProtocol::FUNC_READ_TELEMETRY, (uint16_t)interval_ms);
        } else if (side_token == "RIGHT") {
            queuePeriodicTelemetry(MotorSide::RIGHT, MksServoProtocol::FUNC_READ_TELEMETRY, (uint16_t)interval_ms);
        } else if (side_token == "ALL") {
            queuePeriodicTelemetry(MotorSide::LEFT, MksServoProtocol::FUNC_READ_TELEMETRY, (uint16_t)interval_ms);
            queuePeriodicTelemetry(MotorSide::RIGHT, MksServoProtocol::FUNC_READ_TELEMETRY, (uint16_t)interval_ms);
        } else {
            LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT,
                        "mks_servo: Usage: MOTOR TELEMETRY <LEFT|RIGHT|ALL> <ms>");
            return true;
        }

        LOG_PRINTF(abbot::log::CHANNEL_DEFAULT,
                   "mks_servo: telemetry periodic set %s interval=%dms\n",
                   side_token.c_str(),
                   interval_ms);
        return true;
    }

    // Runtime accel tuning. Examples:
    // - "MOTOR ACCEL" (prints current)
    // - "MOTOR ACCEL 125"
    // - "MOTOR ACC LEFT 125" (compat with existing menu entry)
    auto handleAccelSet = [this](const String &rawLine) -> bool {
        String s = rawLine;
        s.replace("MOTOR", "");
        s.trim();
        // s is now "ACCEL ..." or "ACC ..."
        if (s.startsWith("ACCEL") || s.startsWith("accel")) {
            s = s.substring(5);
        } else if (s.startsWith("ACC") || s.startsWith("acc")) {
            s = s.substring(3);
        } else {
            return false;
        }
        s.trim();
        if (s.length() == 0) {
            uint32_t a = (uint32_t)m_speed_accel.load();
            LOG_PRINTF(abbot::log::CHANNEL_MOTOR, "mks_servo: ACCEL is %lu\n", (unsigned long)a);
            LOG_PRINTF(abbot::log::CHANNEL_DEFAULT, "mks_servo: ACCEL is %lu\n", (unsigned long)a);
            return true;
        }

        // Allow optional side token (LEFT/RIGHT/ID), ignore it.
        String token1 = s;
        String token2 = String();
        int sp = s.indexOf(' ');
        if (sp >= 0) {
            token1 = s.substring(0, sp);
            token2 = s.substring(sp + 1);
            token2.trim();
        }
        String t1u = token1;
        t1u.trim();
        t1u.toUpperCase();
        String valueStr = token1;
        if (t1u == "LEFT" || t1u == "RIGHT" || t1u == "ID") {
            valueStr = token2;
        }
        valueStr.trim();
        int v = valueStr.toInt();
        if (v < 0 || v > 255) {
            LOG_PRINTLN(abbot::log::CHANNEL_MOTOR, "mks_servo: Usage: MOTOR ACCEL <0-255>");
            LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "mks_servo: Usage: MOTOR ACCEL <0-255>");
            return true;
        }
        m_speed_accel.store((uint8_t)v);
        uint32_t a = (uint32_t)m_speed_accel.load();
        LOG_PRINTF(abbot::log::CHANNEL_MOTOR, "mks_servo: ACCEL set to %lu\n", (unsigned long)a);
        LOG_PRINTF(abbot::log::CHANNEL_DEFAULT, "mks_servo: ACCEL set to %lu\n", (unsigned long)a);
        return true;
    };

    if (line.equalsIgnoreCase("ACCEL") || line.startsWith("ACCEL ") ||
        line.equalsIgnoreCase("MOTOR ACCEL") || line.startsWith("MOTOR ACCEL ") ||
        line.equalsIgnoreCase("ACC") || line.startsWith("ACC ") ||
        line.equalsIgnoreCase("MOTOR ACC") || line.startsWith("MOTOR ACC ")) {
        return handleAccelSet(line);
    }

    if (line.equalsIgnoreCase("READ ALL") || line.equalsIgnoreCase("MOTOR READ ALL")) {
        dumpMotorRegisters(MotorSide::LEFT);
        dumpMotorRegisters(MotorSide::RIGHT);
        return true;
    }

    if (line.equalsIgnoreCase("ENABLE LEFT") || line.equalsIgnoreCase("MOTOR ENABLE LEFT")) {
        setEnable(MotorSide::LEFT, true);
        LOG_PRINTLN(abbot::log::CHANNEL_MOTOR, "mks_servo: manual ENABLE LEFT");
        return true;
    }
    if (line.equalsIgnoreCase("ENABLE RIGHT") || line.equalsIgnoreCase("MOTOR ENABLE RIGHT")) {
        setEnable(MotorSide::RIGHT, true);
        LOG_PRINTLN(abbot::log::CHANNEL_MOTOR, "mks_servo: manual ENABLE RIGHT");
        return true;
    }
    if (line.equalsIgnoreCase("DISABLE LEFT") || line.equalsIgnoreCase("MOTOR DISABLE LEFT")) {
        setEnable(MotorSide::LEFT, false);
        LOG_PRINTLN(abbot::log::CHANNEL_MOTOR, "mks_servo: manual DISABLE LEFT");
        return true;
    }
    if (line.equalsIgnoreCase("DISABLE RIGHT") || line.equalsIgnoreCase("MOTOR DISABLE RIGHT")) {
        setEnable(MotorSide::RIGHT, false);
        LOG_PRINTLN(abbot::log::CHANNEL_MOTOR, "mks_servo: manual DISABLE RIGHT");
        return true;
    }

    if (line.equalsIgnoreCase("MOTOR DUMP LEFT") || line.equalsIgnoreCase("DUMP LEFT")) {
        dumpMotorRegisters(MotorSide::LEFT);
        return true;
    }
    if (line.equalsIgnoreCase("MOTOR DUMP RIGHT") || line.equalsIgnoreCase("DUMP RIGHT")) {
        dumpMotorRegisters(MotorSide::RIGHT);
        return true;
    }

    return AbstractMotorDriver::processSerialCommand(line);
}

StepGeneratorMode MksServoMotorDriver::getStepGeneratorMode() const {
    return static_cast<StepGeneratorMode>(m_stepGeneratorMode.load());
}

void MksServoMotorDriver::resetStepGeneratorState() {
    m_left_async.last_ledc_freq = 0;
    m_right_async.last_ledc_freq = 0;
    m_left_async.last_ledc_update_us = 0;
    m_right_async.last_ledc_update_us = 0;
    m_left_async.step_freq_requested.store(0);
    m_right_async.step_freq_requested.store(0);
}

bool MksServoMotorDriver::setStepGeneratorMode(StepGeneratorMode mode, bool force_reinit) {
#if !MKS_SERVO_USE_STEP_DIR
    (void)mode;
    (void)force_reinit;
    return false;
#else
    const StepGeneratorMode current = getStepGeneratorMode();
    if (!force_reinit && mode == current && m_stepGenerator != nullptr) {
        LOG_PRINTF(abbot::log::CHANNEL_DEFAULT,
                   "mks_servo: STEPGEN already %s\n",
                   stepGeneratorModeName(mode));
        return true;
    }

    if (m_enabled.load()) {
        LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT,
                    "mks_servo: refuse STEPGEN change while motors are enabled; disable first");
        return false;
    }

    if (m_stepGenerator) {
        m_stepGenerator->stopAll();
    }

    IStepGenerator* next = nullptr;
    switch (mode) {
        case StepGeneratorMode::MCPWM:
            next = &m_mcpwmStepGenerator;
            break;
        case StepGeneratorMode::RMT:
            next = &m_rmtStepGenerator;
            break;
        case StepGeneratorMode::SOFTWARE:
        default:
            next = &m_softwareStepGenerator;
            break;
    }

    if (!next->init(MKS_SERVO_P1_STEP_PIN, MKS_SERVO_P2_STEP_PIN, MKS_SERVO_COMMON_ANODE != 0)) {
        LOG_PRINTF(abbot::log::CHANNEL_DEFAULT,
                   "mks_servo: STEPGEN init failed for %s\n",
                   stepGeneratorModeName(mode));
        return false;
    }

    m_stepGenerator = next;
    m_stepGeneratorMode.store((uint8_t)mode);
    resetStepGeneratorState();
    LOG_PRINTF(abbot::log::CHANNEL_DEFAULT,
               "mks_servo: STEPGEN switched to %s%s\n",
               stepGeneratorModeName(mode),
               (mode == StepGeneratorMode::SOFTWARE)
                   ? " (software remains the current stable balancing baseline; RMT kept for comparison)"
                   : "");
    return true;
#endif
}

void MksServoMotorDriver::dumpMotorRegisters(MotorSide side) {
    uint8_t id = getMotorId(side);
    LOG_PRINTF(abbot::log::CHANNEL_DEFAULT, "--- mks_servo: Dumping registers for motor %s (ID 0x%02X) ---\n", 
               (side == MotorSide::LEFT ? "LEFT" : "RIGHT"), id);
    
    // Commands to read: Pos(0x31), Speed(0x32), TargetPos(0x33), Pulse(0x34), Error(0x35), Enabled(0x3A)
    struct RegisterDefinition { uint8_t code; const char* name; };
    RegisterDefinition registerDefinitions[] = {
        {MksServoProtocol::FUNC_READ_TELEMETRY, "Telemetry"},
        {MksServoProtocol::FUNC_READ_SPEED,     "Speed    "},
        {MksServoProtocol::FUNC_READ_POS,       "Position "},
        {MksServoProtocol::FUNC_READ_PULSES,    "Pulses   "},
        {MksServoProtocol::FUNC_READ_ERROR,     "Error    "},
        {MksServoProtocol::FUNC_READ_ENABLED,   "Enabled  "}
    };

    for (const auto& reg : registerDefinitions) {
        SemaphoreHandle_t mutex = getMutexForMotor(side);
        if (xSemaphoreTakeRecursive(mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
            continue;
        }

        HardwareSerial& serial = getSerialForMotor(side);
        MksServoProtocol& protocol = getProtocolForMotor(side);

        // Clear any leftover data in RX buffer
        while (serial.available() > 0) {
            serial.read();
        }

        MksServoProtocol::StatusCode status = MksServoProtocol::StatusCode::Ok;
        protocol.sendReadOnlyRequest(id, reg.code, status);
        
        uint8_t responseBuffer[16];
        uint8_t expected_len = MksServoProtocol::getExpectedResponseLength(reg.code);
        bool success = protocol.readResponse(id,
                             reg.code,
                             responseBuffer,
                             expected_len,
                             MKS_SERVO_TIMEOUT_HEAVY_US); // 10ms heavy timeout for config dump registers
        xSemaphoreGiveRecursive(mutex);

        if (success) {
            if (reg.code == MksServoProtocol::FUNC_READ_TELEMETRY || 
                reg.code == MksServoProtocol::FUNC_READ_POS) {
                
                int32_t pos;
                uint8_t status_byte;
                MksServoProtocol::parsePositionPayload(responseBuffer, pos, status_byte);

                LOG_PRINTF(abbot::log::CHANNEL_DEFAULT, "  [0x%02X] %s: Pos: %ld Status: 0x%02X\n", 
                           reg.code, reg.name, (long)pos, status_byte);
            } else if (reg.code == MksServoProtocol::FUNC_READ_ERROR || reg.code == MksServoProtocol::FUNC_READ_ENABLED) {
                LOG_PRINTF(abbot::log::CHANNEL_DEFAULT, "  [0x%02X] %s: Value: 0x%02X\n",
                           reg.code, reg.name, responseBuffer[3]);
            } else {
                String hexDataOutput = "";
                for (size_t i = 0; i < expected_len; ++i) {
                    char buf[4];
                    sprintf(buf, "%02X ", responseBuffer[i]);
                    hexDataOutput += buf;
                }
                LOG_PRINTF(abbot::log::CHANNEL_DEFAULT, "  [0x%02X] %s: %s\n", reg.code, reg.name, hexDataOutput.c_str());
            }
        } else {
            LOG_PRINTF(abbot::log::CHANNEL_DEFAULT, "  [0x%02X] %s: TIMEOUT\n", reg.code, reg.name);
        }
        delay(10);
    }
    LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "--- End Dump ---");
}

// --- Protocol Helpers ---

void MksServoMotorDriver::sendSpeedCommand(MotorSide side, float normalized_speed, bool invert, bool wait_for_ack) {
    uint8_t id = getMotorId(side);
    AsyncState& async = (side == MotorSide::LEFT) ? m_left_async : m_right_async;
    (void)wait_for_ack;

    MksServoProtocol& protocol = getProtocolForMotor(side);
    MksServoProtocol::StatusCode status = MksServoProtocol::StatusCode::Ok;

    // normalized_speed is [-1.0, 1.0]
    // 1. Handle inversion and direction
    float speedValue = normalized_speed;
    if (invert) {
        speedValue = -speedValue;
    }

    uint8_t direction = (speedValue >= 0) ? 0 : 1;
    float absoluteSpeed = (speedValue >= 0) ? speedValue : -speedValue;

    // 2. Map to 12-bit speed (0-3000 RPM)
    uint32_t speedValueCalc = (uint32_t)(absoluteSpeed * (float)VELOCITY_MAX_SPEED);
    if (speedValueCalc > 3000) {
        speedValueCalc = 3000;
    }
    uint16_t speedValueUint = (uint16_t)speedValueCalc;

    protocol.sendSpeedCommand(id,
                              speedValueUint,
                              direction,
                              (uint8_t)m_speed_accel.load(),
                              status);

    async.last_cmd_send_time_us.store((uint64_t)esp_timer_get_time());
    async.speed_cmd_sent.fetch_add(1);
}


void MksServoMotorDriver::setMode(MotorSide side, MksServoMode mode) {
    uint8_t data = (uint8_t)mode;
    sendFunctionCommand(side, MksServoProtocol::FUNC_SET_MODE, &data, 1);
}

void MksServoMotorDriver::setResponseMethod(MotorSide side, bool enabled_respond, bool enabled_active) {
    uint8_t data[2];
    data[0] = enabled_respond ? 0x01 : 0x00;
    data[1] = enabled_active ? 0x01 : 0x00;
    sendFunctionCommand(side, MksServoProtocol::FUNC_SET_RESPONSE_MODE, data, 2);
}

void MksServoMotorDriver::setBaudCode(MotorSide side, uint8_t baud_code) {
    sendFunctionCommand(side, MksServoProtocol::FUNC_SET_BAUD, &baud_code, 1);
}

void MksServoMotorDriver::setMStep(MotorSide side, MksServoMicrostep mstep) {
    uint8_t data = (uint8_t)mstep;
    sendFunctionCommand(side, MksServoProtocol::FUNC_SET_MSTEP, &data, 1);
}

void MksServoMotorDriver::setCurrent(MotorSide side, uint16_t current_ma) {
    uint16_t v = current_ma;
    if (v > 3000) v = 3000;
    uint8_t data[2];
    data[0] = (v >> 8) & 0xFF;
    data[1] = v & 0xFF;
    sendFunctionCommand(side, MksServoProtocol::FUNC_SET_CURRENT, data, 2);
}

void MksServoMotorDriver::setHoldCurrent(MotorSide side, MksServoHoldCurrent hold_pct) {
    uint8_t data = (uint8_t)hold_pct;
    sendFunctionCommand(side, MksServoProtocol::FUNC_SET_HOLD_CURRENT, &data, 1);
}

void MksServoMotorDriver::setEnable(MotorSide side, bool enable) {
    AsyncState& async = (side == MotorSide::LEFT) ? m_left_async : m_right_async;
    if (side == MotorSide::LEFT) {
        m_left.enabled = enable;
    } else {
        m_right.enabled = enable;
    }

    // Support for physical EN pins
    setHardwareEnablePin(side, enable, async.configured_enable_level.load());

    uint8_t data = enable ? 0x01 : 0x00;
    sendFunctionCommand(side, MksServoProtocol::FUNC_TORQUE_ENABLE, &data, 1);
}

void MksServoMotorDriver::sendFunctionCommand(MotorSide side, uint8_t function_code, const uint8_t* data, size_t length) {
    uint8_t id = getMotorId(side);
    QueueHandle_t queue = getQueueForMotor(side);
    if (queue) {
        AsyncState& async = (side == MotorSide::LEFT) ? m_left_async : m_right_async;
        FunctionCommandItem item;
        item.id = id;
        item.function_code = function_code;
        item.length = (length > sizeof(item.data)) ? (uint8_t)sizeof(item.data) : (uint8_t)length;
        for (uint8_t i = 0; i < item.length; ++i) {
            item.data[i] = data[i];
        }
        if (xQueueSend(queue, &item, 0) != pdTRUE) {
            async.function_queue_full.fetch_add(1);
            LOG_PRINTF(abbot::log::CHANNEL_MOTOR,
                       "mks_servo: WARN: func queue full (%s ID=0x%02X FUNC=0x%02X)\n",
                       (side == MotorSide::LEFT ? "LEFT" : "RIGHT"), id, function_code);
        } else {
            // Notify task to process queued command promptly
            TaskHandle_t h = (side == MotorSide::LEFT) ? m_left_async.task_handle : m_right_async.task_handle;
            if (h) {
                xTaskNotifyGive(h);
            }
        }
        return;
    }

    SemaphoreHandle_t mutex = getMutexForMotor(side);
    if (xSemaphoreTakeRecursive(mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        return;
    }

    HardwareSerial& serial = getSerialForMotor(side);
    
    // Clear any leftover data in RX buffer before sending
    while (serial.available() > 0) {
        serial.read();
    }

    MksServoProtocol& protocol = getProtocolForMotor(side);
    MksServoProtocol::StatusCode status = MksServoProtocol::StatusCode::Ok;
    protocol.sendFunctionCommand(id, function_code, data, length, status);

    // Consume ACK (most config commands return 0xFB)
    if (function_code >= 0x80 || function_code == 0x01) {
        size_t ack_length = (function_code == 0x01) ? 6U : 5U;
        uint8_t ackBuffer[6];
        if (readResponse(side, function_code, ackBuffer, ack_length, MKS_SERVO_TIMEOUT_DATA_US)) {
            // Optional: log success if needed
        } else {
            LOG_PRINTF(abbot::log::CHANNEL_MOTOR,
                       "mks_servo: WARN: No ACK for FUNC 0x%02X to ID 0x%02X\n",
                       function_code, id);
        }
    }

    xSemaphoreGiveRecursive(mutex);
}

HardwareSerial& MksServoMotorDriver::getSerialForMotor(MotorSide side) {
    return (side == MotorSide::LEFT) ? Serial2 : Serial1;
}

SemaphoreHandle_t MksServoMotorDriver::getMutexForMotor(MotorSide side) {
    return (side == MotorSide::LEFT) ? m_leftBusMutex : m_rightBusMutex;
}

QueueHandle_t MksServoMotorDriver::getQueueForMotor(MotorSide side) {
    return (side == MotorSide::LEFT) ? m_leftCommandQueue : m_rightCommandQueue;
}

bool MksServoMotorDriver::verifyConfig(MotorSide side, uint8_t function_code, uint8_t expected_value, const char* label) {
    SemaphoreHandle_t mutex = getMutexForMotor(side);
    if (xSemaphoreTakeRecursive(mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        return false;
    }

    uint8_t id = getMotorId(side);
    HardwareSerial& serial = getSerialForMotor(side);
    MksServoProtocol& protocol = getProtocolForMotor(side);

    // Clear buffer before making a new synchronous request
    while (serial.available() > 0) {
        serial.read();
    }

    MksServoProtocol::StatusCode status = MksServoProtocol::StatusCode::Ok;
    protocol.sendReadOnlyRequest(id, function_code, status);

    uint8_t responseBuffer[16];
    size_t expected_len = MksServoProtocol::getExpectedResponseLength(function_code);

    bool success = readResponse(side, function_code, responseBuffer, expected_len, 30000);
    xSemaphoreGiveRecursive(mutex);

    if (success) {
        if (responseBuffer[3] == expected_value) {
            LOG_PRINTF(abbot::log::CHANNEL_MOTOR, "mks_servo: motor 0x%02X verified %s: 0x%02X\n", id, label, responseBuffer[3]);
            return true;
        } else {
            LOG_PRINTF(abbot::log::CHANNEL_DEFAULT, "mks_servo: motor 0x%02X %s mismatch! expected 0x%02X, got 0x%02X\n", id, label, expected_value, responseBuffer[3]);
        }
    } else {
        LOG_PRINTF(abbot::log::CHANNEL_DEFAULT, "mks_servo: motor 0x%02X %s verification TIMEOUT\n", id, label);
    }
    return false;
}

bool MksServoMotorDriver::readResponse(MotorSide side, uint8_t function_code, uint8_t* out_data, size_t expected_length, uint32_t timeout_us) {
    uint8_t id = getMotorId(side);
    MksServoProtocol& protocol = getProtocolForMotor(side);
    return protocol.readResponse(id,
                                 function_code,
                                 out_data,
                                 expected_length,
                                 timeout_us);
}

} // namespace motor
} // namespace abbot
