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

SemaphoreHandle_t g_step_dir_mutex = nullptr;

} // namespace

MksServoMotorDriver::MksServoMotorDriver()
    : m_left{LEFT_MOTOR_ID, LEFT_MOTOR_INVERT != 0, 0.0f, 0, 0, 0, 0, 0, false, SpeedEstimator(MKS_SERVO_SPEED_ALPHA)},
      m_right{RIGHT_MOTOR_ID, RIGHT_MOTOR_INVERT != 0, 0.0f, 0, 0, 0, 0, 0, false, SpeedEstimator(MKS_SERVO_SPEED_ALPHA)},
      m_enabled(false),
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
    
    // Create protocols
    if (m_leftProtocol) delete m_leftProtocol;
    if (m_rightProtocol) delete m_rightProtocol;
    m_leftProtocol = new MksServoProtocol(Serial2);
    m_rightProtocol = new MksServoProtocol(Serial1);

    if (m_leftTelemetryIngest) delete m_leftTelemetryIngest;
    if (m_rightTelemetryIngest) delete m_rightTelemetryIngest;
    m_leftTelemetryIngest = new MksServoTelemetryIngest(*m_leftProtocol, m_left.speedEstimator, &m_left.invert);
    m_rightTelemetryIngest = new MksServoTelemetryIngest(*m_rightProtocol, m_right.speedEstimator, &m_right.invert);

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

    m_stepGenerator.init(MKS_SERVO_P1_STEP_PIN, MKS_SERVO_P2_STEP_PIN, MKS_SERVO_COMMON_ANODE != 0);

    LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "mks_servo: Step/Dir (Hybrid) Hardware Initialized");

    if (!g_step_dir_mutex) {
        g_step_dir_mutex = xSemaphoreCreateMutex();
    }
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
        // In Hybrid (Step/Dir) mode, we don't need 1000Hz RS485 reactivity because 
        // pulses are hardware-generated. We wait for the telemetry interval.
        // In Serial mode, we wait 1ms to process speed commands with low latency.
        uint32_t wait_ms = 1;
#if MKS_SERVO_USE_STEP_DIR
        // If we are waiting for an ACK, wake up often (1ms). 
        // Otherwise, we can wait until the next telemetry interval (20ms).
        if (!ack_pending) {
            wait_ms = telemetry_interval_ms;
        }
#endif
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(wait_ms)); 

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
                        if ((millis() - state.last_ack_log_ms >= 1000)) {
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

            bool current_enabled = m_enabled;
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
                    bool pin_level = target_en ? LOW : HIGH;
#if !MKS_SERVO_COMMON_ANODE
                    pin_level = !pin_level;
#endif
                    if (state.id == m_left.id) {
                        digitalWrite(MKS_SERVO_P1_EN_PIN, pin_level);
                    } else {
                        digitalWrite(MKS_SERVO_P2_EN_PIN, pin_level);
                    }

                    async.last_reported_enabled.store(target_en);
                    speed_dirty = true;
                } else {
                    // Try again on next tick; keep last_reported_enabled unchanged.
                }
            }

            // 2. Send Speed Command
#if MKS_SERVO_USE_STEP_DIR
            // Speed is handled by hardware pulses (Zero Latency). 
            // We skip sending RS485 speed packets to avoid bus congestion and mode conflicts.
            if (speed_dirty) {
                async.speed_cmd_sent.fetch_add(1);
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
                    async.speed_value.store(sample.speed);
                    async.encoder_value.store(sample.position);
                    async.encoder_dirty.store(true);
                    async.last_encoder_time_us.store((uint64_t)esp_timer_get_time());
                    async.encoder_ok.fetch_add(1);
                }
            }
#endif
            xSemaphoreGiveRecursive(mutex);
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

void MksServoMotorDriver::queuePeriodicTelemetry(MotorSide side, uint16_t interval_ms) {
#if MKS_SERVO_TELEMETRY_ENABLED
    uint8_t payload[3];
    payload[0] = MksServoProtocol::FUNC_READ_TELEMETRY; // 0x31: speed + pulses
    payload[1] = (uint8_t)((interval_ms >> 8) & 0xFF);
    payload[2] = (uint8_t)(interval_ms & 0xFF);
    sendFunctionCommand(side, MksServoProtocol::FUNC_SET_PERIODIC, payload, 3);
#else
    (void)side;
    (void)interval_ms;
#endif
}


void MksServoMotorDriver::clearCommandState() {
    m_left.last_command = 0.0f;
    m_right.last_command = 0.0f;
    setMotorCommandBoth(0.0f, 0.0f);
}

float MksServoMotorDriver::getLastMotorCommand(MotorSide side) {
    return (side == MotorSide::LEFT) ? m_left.last_command : m_right.last_command;
}

void MksServoMotorDriver::enableMotors() {
    if (m_enabled) {
        return;
    }
    
    LOG_PRINTLN(abbot::log::CHANNEL_MOTOR, "mks_servo: enabling motors (Queueing Torque ON)");
    m_enabled = true;

    queuePeriodicTelemetry(MotorSide::LEFT, (uint16_t)MKS_SERVO_PERIODIC_TELEMETRY_MS);
    queuePeriodicTelemetry(MotorSide::RIGHT, (uint16_t)MKS_SERVO_PERIODIC_TELEMETRY_MS);
    
    // Notify background tasks to perform Torque ON and initial speed sync
    if (m_left_async.task_handle) {
        xTaskNotifyGive(m_left_async.task_handle);
    }
    if (m_right_async.task_handle) {
        xTaskNotifyGive(m_right_async.task_handle);
    }
}

void MksServoMotorDriver::disableMotors() {
    if (!m_enabled) {
        return;
    }

    LOG_PRINTLN(abbot::log::CHANNEL_MOTOR, "mks_servo: disabling motors (Queueing Torque OFF)");
    m_enabled = false;

    queuePeriodicTelemetry(MotorSide::LEFT, 0);
    queuePeriodicTelemetry(MotorSide::RIGHT, 0);

    // Notify background tasks to perform Torque OFF
    if (m_left_async.task_handle) {
        xTaskNotifyGive(m_left_async.task_handle);
    }
    if (m_right_async.task_handle) {
        xTaskNotifyGive(m_right_async.task_handle);
    }
}

bool MksServoMotorDriver::areMotorsEnabled() {
    return m_enabled;
}

uint32_t MksServoMotorDriver::getLastEncoderAgeMs(MotorSide side) const {
    const AsyncState& async = (side == MotorSide::LEFT) ? m_left_async : m_right_async;
    uint64_t last_us = async.last_encoder_time_us.load();
    if (last_us == 0) {
        return UINT32_MAX;
    }

    uint64_t now_us = (uint64_t)esp_timer_get_time();
    if (now_us <= last_us) {
        return 0;
    }

    uint64_t age_us = now_us - last_us;
    if (age_us > (uint64_t)UINT32_MAX * 1000ULL) {
        return UINT32_MAX;
    }
    return (uint32_t)(age_us / 1000ULL);
}

void MksServoMotorDriver::printStatus() {
    uint32_t lat_us = getLastBusLatencyUs();
    LOG_PRINTF(abbot::log::CHANNEL_MOTOR,
               "mks_servo: enabled=%d ack=%d accel=%u bus_lat_us=%lu L_cmd=%.3f R_cmd=%.3f\n",
               m_enabled, (int)m_wait_for_ack.load(), (unsigned)m_speed_accel.load(), (unsigned long)lat_us,
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
}

void MksServoMotorDriver::dumpConfig() {
    LOG_PRINTF(abbot::log::CHANNEL_MOTOR,
               "mks_servo: L_ID=0x%02X R_ID=0x%02X Baud=%d Ma=%d vmax_rpm=%d accel=%u\n",
               m_left.id, m_right.id, MKS_SERVO_BAUD, MKS_SERVO_MA, (int)VELOCITY_MAX_SPEED, (unsigned)m_speed_accel.load());
}

void MksServoMotorDriver::setMotorCommandBoth(float left_command, float right_command) {
    // 1. Update targets (Atomics for background tasks telemetry/config)
    m_left_async.target_speed.store(left_command);
    m_left_async.speed_dirty.store(true);
    
    m_right_async.target_speed.store(right_command);
    m_right_async.speed_dirty.store(true);

    m_left.last_command = left_command;
    m_right.last_command = right_command;

#if MKS_SERVO_USE_STEP_DIR
    // 2. Immediate Step/Dir hardware update (Zero Latency)
    // We update even if a telemetry quiet window is requested; the quiet window
    // will temporarily override this, but hardware settings must remain up-to-date.

    // Left Motor (Channel 0)
    {
        bool left_dir = (left_command >= 0);
        if (m_left.invert) { left_dir = !left_dir; }
#if MKS_SERVO_PULSE_DIR_INVERT
        left_dir = !left_dir;
#endif
#if MKS_SERVO_COMMON_ANODE
        left_dir = !left_dir; // Sinking logic for optocoupler: LOW = LED ON
#endif
        digitalWrite(MKS_SERVO_P1_DIR_PIN, left_dir ? HIGH : LOW);

        float left_abs = fabsf(left_command);
        if (left_abs < 0.001f || !m_enabled) {
            m_left_async.last_ledc_freq = 0;
            if (!m_telemetry_quiet_pending) {
                m_stepGenerator.setFrequency(true, 0);
            }
            m_left_async.last_ledc_update_us = micros();
        } else {
            uint32_t freq = (uint32_t)((left_abs * VELOCITY_MAX_SPEED / 60.0f) * MKS_SERVO_STEPS_PER_REV);
            
            uint32_t now_us = micros();
            uint32_t ledc_interval_us = 0;
#if MKS_SERVO_STEP_UPDATE_HZ > 0
            ledc_interval_us = (uint32_t)(1000000UL / (uint32_t)MKS_SERVO_STEP_UPDATE_HZ);
#endif
            bool time_ok = (ledc_interval_us == 0U) || ((int32_t)(now_us - m_left_async.last_ledc_update_us) >= (int32_t)ledc_interval_us);
            if (time_ok && abs((int)freq - (int)m_left_async.last_ledc_freq) > MKS_SERVO_STEP_DEADBAND_HZ) {
                m_left_async.last_ledc_freq = freq;
                m_left_async.last_ledc_update_us = now_us;
                if (!m_telemetry_quiet_pending) {
                    m_stepGenerator.setFrequency(true, freq);
                }
            }
        }
    }
    
    // Right Motor (Channel 2)
    {
        bool right_dir = (right_command >= 0);
        if (m_right.invert) { right_dir = !right_dir; }
#if MKS_SERVO_PULSE_DIR_INVERT
        right_dir = !right_dir;
#endif
#if MKS_SERVO_COMMON_ANODE
        right_dir = !right_dir;
#endif
        digitalWrite(MKS_SERVO_P2_DIR_PIN, right_dir ? HIGH : LOW);

        float right_abs = fabsf(right_command);
        if (right_abs < 0.001f || !m_enabled) {
            m_right_async.last_ledc_freq = 0;
            if (!m_telemetry_quiet_pending) {
                m_stepGenerator.setFrequency(false, 0);
            }
            m_right_async.last_ledc_update_us = micros();
        } else {
            uint32_t freq = (uint32_t)((right_abs * VELOCITY_MAX_SPEED / 60.0f) * MKS_SERVO_STEPS_PER_REV);
            
            uint32_t now_us = micros();
            uint32_t ledc_interval_us = 0;
#if MKS_SERVO_STEP_UPDATE_HZ > 0
            ledc_interval_us = (uint32_t)(1000000UL / (uint32_t)MKS_SERVO_STEP_UPDATE_HZ);
#endif
            bool time_ok = (ledc_interval_us == 0U) || ((int32_t)(now_us - m_right_async.last_ledc_update_us) >= (int32_t)ledc_interval_us);
            if (time_ok && abs((int)freq - (int)m_right_async.last_ledc_freq) > MKS_SERVO_STEP_DEADBAND_HZ) {
                m_right_async.last_ledc_freq = freq;
                m_right_async.last_ledc_update_us = now_us;
                if (!m_telemetry_quiet_pending) {
                    m_stepGenerator.setFrequency(false, freq);
                }
            }
        }
    }
#endif

    // 3. Notify background tasks to process RS485
    // In Hybrid mode (Step/Dir), we don't notify the task for every speed update 
    // because speed is handled by hardware PWM. The task will still run periodically 
    // for telemetry via its internal timeout.
#if !MKS_SERVO_USE_STEP_DIR
    if (m_left_async.task_handle) {
        xTaskNotifyGive(m_left_async.task_handle);
    }
    if (m_right_async.task_handle) {
        xTaskNotifyGive(m_right_async.task_handle);
    }
#endif

    // 3. Update legacy state for monitoring/logging
    m_left.last_command = left_command;
    m_right.last_command = right_command;
}

void MksServoMotorDriver::readEncodersBoth(int32_t& left_out, int32_t& right_out) {
    // Return cached values updated by motorTasks on Core 0
    left_out = m_left_async.encoder_value.load();
    right_out = m_right_async.encoder_value.load();
}

void MksServoMotorDriver::setMotorCommand(MotorSide side, float command) {
    if (side == MotorSide::LEFT) {
        m_left_async.target_speed.store(command);
        m_left_async.speed_dirty.store(true);
#if !MKS_SERVO_USE_STEP_DIR
        if (m_left_async.task_handle) {
            xTaskNotifyGive(m_left_async.task_handle);
        }
#endif
        m_left.last_command = command;

#if MKS_SERVO_USE_STEP_DIR
    // Immediate hardware pulse update for single motor command
    bool left_dir = (command >= 0);
    if (m_left.invert) {
        left_dir = !left_dir;
    }
#if MKS_SERVO_PULSE_DIR_INVERT
    left_dir = !left_dir;
#endif
#if MKS_SERVO_COMMON_ANODE
    left_dir = !left_dir;
#endif
    digitalWrite(MKS_SERVO_P1_DIR_PIN, left_dir ? HIGH : LOW);

    float left_abs = fabsf(command);
    if (left_abs < 0.001f || !m_enabled) {
        m_left_async.last_ledc_freq = 0;
        if (!m_telemetry_quiet_pending) {
            m_stepGenerator.setFrequency(true, 0);
        }
        m_left_async.last_ledc_update_us = micros();
    } else {
        uint32_t freq = (uint32_t)((left_abs * VELOCITY_MAX_SPEED / 60.0f) * MKS_SERVO_STEPS_PER_REV);
        
        uint32_t now_us = micros();
        uint32_t ledc_interval_us = 0;
#if MKS_SERVO_STEP_UPDATE_HZ > 0
        ledc_interval_us = (uint32_t)(1000000UL / (uint32_t)MKS_SERVO_STEP_UPDATE_HZ);
#endif
        bool time_ok = (ledc_interval_us == 0U) || ((int32_t)(now_us - m_left_async.last_ledc_update_us) >= (int32_t)ledc_interval_us);
        if (time_ok && abs((int)freq - (int)m_left_async.last_ledc_freq) > MKS_SERVO_STEP_DEADBAND_HZ) {
            m_left_async.last_ledc_freq = freq;
            m_left_async.last_ledc_update_us = now_us;
            if (!m_telemetry_quiet_pending) {
                m_stepGenerator.setFrequency(true, freq);
            }
        }
    }
#endif
    } else {
        m_right_async.target_speed.store(command);
        m_right_async.speed_dirty.store(true);
#if !MKS_SERVO_USE_STEP_DIR
        if (m_right_async.task_handle) {
            xTaskNotifyGive(m_right_async.task_handle);
        }
#endif
        m_right.last_command = command;

#if MKS_SERVO_USE_STEP_DIR
    // Immediate hardware pulse update for single motor command
    bool right_dir = (command >= 0);
    if (m_right.invert) {
        right_dir = !right_dir;
    }
#if MKS_SERVO_PULSE_DIR_INVERT
    right_dir = !right_dir;
#endif
#if MKS_SERVO_COMMON_ANODE
    right_dir = !right_dir;
#endif
    digitalWrite(MKS_SERVO_P2_DIR_PIN, right_dir ? HIGH : LOW);

    float right_abs = fabsf(command);
    if (right_abs < 0.001f || !m_enabled) {
        m_right_async.last_ledc_freq = 0;
        if (!m_telemetry_quiet_pending) {
            m_stepGenerator.setFrequency(false, 0);
        }
        m_right_async.last_ledc_update_us = micros();
    } else {
        uint32_t freq = (uint32_t)((right_abs * VELOCITY_MAX_SPEED / 60.0f) * MKS_SERVO_STEPS_PER_REV);
        
        uint32_t now_us = micros();
        uint32_t ledc_interval_us = 0;
#if MKS_SERVO_STEP_UPDATE_HZ > 0
        ledc_interval_us = (uint32_t)(1000000UL / (uint32_t)MKS_SERVO_STEP_UPDATE_HZ);
#endif
        bool time_ok = (ledc_interval_us == 0U) || ((int32_t)(now_us - m_right_async.last_ledc_update_us) >= (int32_t)ledc_interval_us);
        if (time_ok && abs((int)freq - (int)m_right_async.last_ledc_freq) > MKS_SERVO_STEP_DEADBAND_HZ) {
            m_right_async.last_ledc_freq = freq;
            m_right_async.last_ledc_update_us = now_us;
            if (!m_telemetry_quiet_pending) {
                m_stepGenerator.setFrequency(false, freq);
            }
        }
    }
#endif
    }
}

void MksServoMotorDriver::setMotorCommandRaw(MotorSide side, int16_t rawSpeed) {
    // Treat raw speed as normalized units / 1000
    float normalized = (float)rawSpeed / 1000.0f;
    setMotorCommand(side, normalized);
}

int32_t MksServoMotorDriver::readEncoder(MotorSide side) {
    return (side == MotorSide::LEFT) ? m_left_async.encoder_value.load() : m_right_async.encoder_value.load();
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
    // Return timestamp of last *hardware* send (not queue time)
    return (side == MotorSide::LEFT) ? m_left_async.last_cmd_send_time_us.load()
                                    : m_right_async.last_cmd_send_time_us.load();
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
            LOG_PRINTF(abbot::log::CHANNEL_DEFAULT, "  [ID 0x%02X] Mode: %d, MStep: %d, Current: %d mA, Hold: %d%%\n",
                       id, params[1], params[2], (params[4] << 8) | params[5], params[14] * 10);
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
    // Determine which bus this ID might be on. If ambiguous (both 1), we'll try both.
    if (id == (uint8_t)m_left.id) sendFunctionCommand(MotorSide::LEFT, MksServoProtocol::FUNC_CALIBRATE_ENCODER, nullptr, 0);
    if (id == (uint8_t)m_right.id) sendFunctionCommand(MotorSide::RIGHT, MksServoProtocol::FUNC_CALIBRATE_ENCODER, nullptr, 0);
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

        for (uint8_t id = 1; id <= 2; ++id) {
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
            queuePeriodicTelemetry(MotorSide::LEFT, (uint16_t)interval_ms);
        } else if (side_token == "RIGHT") {
            queuePeriodicTelemetry(MotorSide::RIGHT, (uint16_t)interval_ms);
        } else if (side_token == "ALL") {
            queuePeriodicTelemetry(MotorSide::LEFT, (uint16_t)interval_ms);
            queuePeriodicTelemetry(MotorSide::RIGHT, (uint16_t)interval_ms);
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
    if (side == MotorSide::LEFT) {
        m_left.enabled = enable;
    } else {
        m_right.enabled = enable;
    }

    // Support for physical EN pins
    bool pin_level = enable ? LOW : HIGH;
#if !MKS_SERVO_COMMON_ANODE
    pin_level = !pin_level;
#endif
    if (side == MotorSide::LEFT) {
        digitalWrite(MKS_SERVO_P1_EN_PIN, pin_level);
    } else {
        digitalWrite(MKS_SERVO_P2_EN_PIN, pin_level);
    }

    uint8_t data = enable ? 0x01 : 0x00;
    sendFunctionCommand(side, MksServoProtocol::FUNC_TORQUE_ENABLE, &data, 1);
}

void MksServoMotorDriver::sendFunctionCommand(MotorSide side, uint8_t function_code, const uint8_t* data, size_t length) {
    uint8_t id = getMotorId(side);
    QueueHandle_t queue = getQueueForMotor(side);
    if (queue) {
        FunctionCommandItem item;
        item.id = id;
        item.function_code = function_code;
        item.length = (length > sizeof(item.data)) ? (uint8_t)sizeof(item.data) : (uint8_t)length;
        for (uint8_t i = 0; i < item.length; ++i) {
            item.data[i] = data[i];
        }
        if (xQueueSend(queue, &item, 0) != pdTRUE) {
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
