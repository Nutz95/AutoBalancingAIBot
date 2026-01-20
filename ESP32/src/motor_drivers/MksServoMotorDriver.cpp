// MksServoMotorDriver.cpp - Implementation for MKS SERVO42D/57D serial bus motors
#include "../../include/motor_drivers/MksServoMotorDriver.h"
#include "../../include/balancer_controller.h"
#include "../../config/balancer_config.h"
#include <Arduino.h>

#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

namespace abbot {
namespace motor {

namespace {

SemaphoreHandle_t g_step_dir_mutex = nullptr;

struct AckParser {
    uint8_t buffer[5] = {0, 0, 0, 0, 0};
    uint8_t received = 0;
};

uint8_t computeChecksum(const uint8_t *data, size_t length) {
    uint8_t checksum = 0;
    for (size_t index = 0; index < length; ++index) {
        checksum = (uint8_t)(checksum + data[index]);
    }
    return checksum;
}

bool tryConsumeAckNonBlocking(MksServoMotorDriver &driver,
                              HardwareSerial &serial,
                              uint8_t expected_id,
                              uint8_t expected_function,
                              AckParser &parser,
                              uint8_t &status_out) {
    int available_bytes = serial.available();
    if (available_bytes <= 0) {
        return false;
    }

    // Bound work per tick to avoid spending too long in this task.
    int bytes_to_read = (available_bytes > 16) ? 16 : available_bytes;
    while (bytes_to_read-- > 0) {
        int v = serial.read();
        if (v < 0) {
            break;
        }
        uint8_t byteRead = (uint8_t)v;

        if (parser.received == 0) {
            if (byteRead == 0xFB) {
                parser.buffer[0] = byteRead;
                parser.received = 1;
            }
            continue;
        }

        if (parser.received == 1) {
            if (byteRead != expected_id) {
                parser.received = 0;
                if (byteRead == 0xFB) {
                    parser.buffer[0] = 0xFB;
                    parser.received = 1;
                }
                continue;
            }
            parser.buffer[1] = byteRead;
            parser.received = 2;
            continue;
        }

        if (parser.received == 2) {
            if (byteRead != expected_function) {
                parser.received = 0;
                if (byteRead == 0xFB) {
                    parser.buffer[0] = 0xFB;
                    parser.received = 1;
                }
                continue;
            }
            parser.buffer[2] = byteRead;
            parser.received = 3;
            continue;
        }

        parser.buffer[parser.received++] = byteRead;
        if (parser.received >= 5) {
            uint8_t expectedChecksum = computeChecksum(parser.buffer, 4);
            uint8_t gotChecksum = parser.buffer[4];
            if (expectedChecksum == gotChecksum) {
                status_out = parser.buffer[3];
                parser.received = 0;
                return true;
            }

            // Bad checksum, resync.
            parser.received = 0;
            if (byteRead == 0xFB) {
                parser.buffer[0] = 0xFB;
                parser.received = 1;
            }
        }
    }

    return false;
}

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

void MksServoMotorDriver::initMotorDriver() {
    // Initialize both serial ports at the configured baud rate
    // Motor 1 (Left) on Serial2
    Serial2.begin(MKS_SERVO_BAUD, SERIAL_8N1, MKS_SERVO_P1_RX_PIN, MKS_SERVO_P1_TX_PIN);
    // Motor 2 (Right) on Serial1
    Serial1.begin(MKS_SERVO_BAUD, SERIAL_8N1, MKS_SERVO_P2_RX_PIN, MKS_SERVO_P2_TX_PIN);

#if MKS_SERVO_USE_STEP_DIR
    // Initialize Step/Dir pins
    // Cablage : 
    //   - Step (PUL+) : Fil VERT
    //   - Dir (DIR+) : Fil BLEU
    //   - COM : 3.3V (Common Anode)
    pinMode(MKS_SERVO_P1_STEP_PIN, OUTPUT);
    pinMode(MKS_SERVO_P1_DIR_PIN, OUTPUT);
    pinMode(MKS_SERVO_P2_STEP_PIN, OUTPUT);
    pinMode(MKS_SERVO_P2_DIR_PIN, OUTPUT);
    pinMode(MKS_SERVO_P1_EN_PIN, OUTPUT);
    pinMode(MKS_SERVO_P2_EN_PIN, OUTPUT);

#if MKS_SERVO_COMMON_ANODE
    // Common Anode: pins must be HIGH to keep optos OFF
    digitalWrite(MKS_SERVO_P1_STEP_PIN, HIGH);
    digitalWrite(MKS_SERVO_P1_DIR_PIN, HIGH);
    digitalWrite(MKS_SERVO_P2_STEP_PIN, HIGH);
    digitalWrite(MKS_SERVO_P2_DIR_PIN, HIGH);
    digitalWrite(MKS_SERVO_P1_EN_PIN, HIGH); // Disabled
    digitalWrite(MKS_SERVO_P2_EN_PIN, HIGH); // Disabled
#else
    digitalWrite(MKS_SERVO_P1_STEP_PIN, LOW);
    digitalWrite(MKS_SERVO_P1_DIR_PIN, LOW);
    digitalWrite(MKS_SERVO_P2_STEP_PIN, LOW);
    digitalWrite(MKS_SERVO_P2_DIR_PIN, LOW);
    digitalWrite(MKS_SERVO_P1_EN_PIN, LOW); 
    digitalWrite(MKS_SERVO_P2_EN_PIN, LOW); 
#endif

    // Setup LEDC (PWM) channels for high-precision step generation
    ledcSetup(0, 1000, MKS_SERVO_LEDC_RES);      
    ledcAttachPin(MKS_SERVO_P1_STEP_PIN, 0);
#if MKS_SERVO_COMMON_ANODE
    ledcWrite(0, (1 << MKS_SERVO_LEDC_RES) - 1); // 100% Duty = HIGH = OFF
#else
    ledcWrite(0, 0);                             // 0% Duty = LOW = OFF
#endif
    
    // Use Channel 2 for Right motor to ensure independent freq timer (on S3)
    ledcSetup(2, 1000, MKS_SERVO_LEDC_RES);      
    ledcAttachPin(MKS_SERVO_P2_STEP_PIN, 2);
#if MKS_SERVO_COMMON_ANODE
    ledcWrite(2, (1 << MKS_SERVO_LEDC_RES) - 1);
#else
    ledcWrite(2, 0);
#endif

    LOG_PRINTF(abbot::log::CHANNEL_DEFAULT,
               "mks_servo: Step/Dir Hybrid Enabled (L_STEP:%d, L_DIR:%d | R_STEP:%d, R_DIR:%d)\n",
               MKS_SERVO_P1_STEP_PIN, MKS_SERVO_P1_DIR_PIN, 
               MKS_SERVO_P2_STEP_PIN, MKS_SERVO_P2_DIR_PIN);

    if (!g_step_dir_mutex) {
        g_step_dir_mutex = xSemaphoreCreateMutex();
        if (!g_step_dir_mutex) {
            LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "mks_servo: ERROR failed to create step/dir mutex");
        }
    }
#endif
    
    LOG_PRINTF(abbot::log::CHANNEL_DEFAULT,
               "mks_servo: initialized Serial2 (Left, ID:%d) TX:%d RX:%d, Serial1 (Right, ID:%d) TX:%d RX:%d at %d baud\n",
               LEFT_MOTOR_ID, MKS_SERVO_P1_TX_PIN, MKS_SERVO_P1_RX_PIN,
               RIGHT_MOTOR_ID, MKS_SERVO_P2_TX_PIN, MKS_SERVO_P2_RX_PIN,
               MKS_SERVO_BAUD);

    // Give drivers time to boot
    delay(100);

    // Create background tasks for each motor bus on Core 0.
    // Keep priority high for balancing, but make it configurable.
    xTaskCreatePinnedToCore(motorTaskEntry, "motorL_task", 4096, new std::pair<MksServoMotorDriver*, MotorSide>(this, MotorSide::LEFT), MKS_SERVO_TASK_PRIORITY, &m_left_async.task_handle, 0);
    xTaskCreatePinnedToCore(motorTaskEntry, "motorR_task", 4096, new std::pair<MksServoMotorDriver*, MotorSide>(this, MotorSide::RIGHT), MKS_SERVO_TASK_PRIORITY, &m_right_async.task_handle, 0);

    // Initial configuration for both motors
    uint8_t ids[] = {(uint8_t)m_left.id, (uint8_t)m_right.id};
    for (uint8_t id : ids) {
        LOG_PRINTF(abbot::log::CHANNEL_DEFAULT, "mks_servo: configuring motor ID 0x%02X...\n", id);
        
        setMode(id, MKS_SERVO_DEFAULT_MODE);
        delay(20);
        setMStep(id, MKS_SERVO_DEFAULT_MSTEP);
        delay(20);
        setCurrent(id, MKS_SERVO_MA);
        delay(20);
        setHoldCurrent(id, MKS_SERVO_HOLD_PCT);
        delay(20);

        LOG_PRINTF(abbot::log::CHANNEL_DEFAULT,
                   "mks_servo: motor ID 0x%02X configuration commands sent\n", id);
    }

    // Ensure motors are initially disabled and stopped for safety
    // This will now trigger an explicit Torque OFF packet because last_reported_enabled=true initially.
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
    SemaphoreHandle_t mutex = (side == MotorSide::LEFT) ? m_leftBusMutex : m_rightBusMutex;

    const uint32_t telemetry_interval_ms = (MKS_SERVO_ENCODER_UPDATE_HZ > 0) ? (1000 / MKS_SERVO_ENCODER_UPDATE_HZ) : 10;
    uint32_t last_latency_log_ms = 0;
    uint8_t loop_divider = 0;
    uint32_t next_tx_allowed_us = 0;
    bool ack_pending = false;
    uint32_t ack_deadline_us = 0;
    uint32_t ack_start_us = 0;
    uint8_t ack_expected_function = 0xF6;
    AckParser ack_parser;
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
                if (tryConsumeAckNonBlocking(*this, serial, (uint8_t)state.id, ack_expected_function, ack_parser, ack_status)) {
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
                QueueHandle_t queue = getQueueForMotor((uint8_t)state.id);
                if (queue) {
                    if (xQueueReceive(queue, &pending_function, 0) == pdTRUE) {
                        function_pending = true;
                    }
                }
            }

            if (function_pending) {
                if (canTransmitNow()) {
                    uint8_t frame[16];
                    frame[0] = 0xFA;
                    frame[1] = pending_function.id;
                    frame[2] = pending_function.function_code;
                    for (uint8_t i = 0; i < pending_function.length; ++i) {
                        frame[3 + i] = pending_function.data[i];
                    }
                    size_t total_len = 3 + pending_function.length + 1;
                    frame[total_len - 1] = computeChecksum(frame, total_len - 1);
                    writeFrame(serial, frame, total_len);
                    markTransmitted();

                    if (m_wait_for_ack.load()) {
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

                    // Send Enable/Disable command via RS485 (Redundant but safe)
                    uint8_t data = target_en ? 0x01 : 0x00;
                    sendFunctionCommand(state.id, 0xF3, &data, 1);

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
                    sendSpeedCommand(state.id, cmd, state.invert, waitAck);
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
                        sendSpeedCommand(state.id, 0.0f, state.invert, m_wait_for_ack.load());
                        markTransmitted();
                        async.last_zero_ms = now_ms;
                    }
                }
            }

            // 3. Telemetry read
            uint32_t now = millis();
            if (now - async.last_telemetry_ms >= telemetry_interval_ms) {
                // Update timer IMMEDIATELY to prevent high-frequency retry loops 
                // if the upcoming readResponse times out or fails.
                async.last_telemetry_ms = now;

#if MKS_SERVO_TELEMETRY_ENABLED

                // Clear any leftover bytes and echoes
                while (serial.available() > 0) {
                    serial.read();
                }

                if (canTransmitNow()) {
                    uint8_t readFrame[4];
                    readFrame[0] = 0xFA;
                    readFrame[1] = state.id;
                    readFrame[2] = 0x31;
                    readFrame[3] = calculateChecksum(readFrame, 3);

                    uint32_t telemetry_start_us = micros();

                    bool frame_written = false;
#if MKS_SERVO_USE_STEP_DIR
                    // Some MKS firmwares fail to answer 0x31 while receiving high-rate
                    // Step/Dir pulses. Optionally pause pulses briefly around the request.
                    if (MKS_SERVO_TELEMETRY_QUIET_WINDOW_US > 0 && g_step_dir_mutex) {
                        if (xSemaphoreTake(g_step_dir_mutex, pdMS_TO_TICKS(2)) == pdTRUE) {
                            uint8_t ledc_channel = (side == MotorSide::LEFT) ? 0 : 2;
                            uint32_t restore_freq = (side == MotorSide::LEFT) ? m_left_async.last_ledc_freq : m_right_async.last_ledc_freq;

#if MKS_SERVO_COMMON_ANODE
                            ledcWrite(ledc_channel, (1 << MKS_SERVO_LEDC_RES) - 1);
#else
                            ledcWrite(ledc_channel, 0);
#endif

                            writeFrame(serial, readFrame, 4);
                            markTransmitted();
                            frame_written = true;

                            delayMicroseconds((uint32_t)MKS_SERVO_TELEMETRY_QUIET_WINDOW_US);

                            if (restore_freq > 0 && m_enabled) {
                                ledcWriteTone(ledc_channel, restore_freq);
                            }

                            xSemaphoreGive(g_step_dir_mutex);
                        }
                    }
#endif

                    if (!frame_written) {
                        writeFrame(serial, readFrame, 4);
                        markTransmitted();
                    }

                    uint8_t response[10]; 
                    bool got_response = readResponse(serial, state.id, 0x31, response, 10, MKS_SERVO_TIMEOUT_DATA_US);
                    uint32_t telemetry_end_us = micros();
                    
                    // Update latency even for telemetry so users can see bus health on graphs
                    async.last_latency_us.store((uint32_t)(telemetry_end_us - telemetry_start_us));
                    async.last_latency_age_ms.store(0);

                    if (got_response) {
                        // Extract position according to spec: [ID][0x31][Sign][P3][P2][P1][P0][VH][VL][CS]
                        int32_t p = ((int32_t)response[3] << 24) | ((int32_t)response[4] << 16) | 
                                    ((int32_t)response[5] << 8) | (int32_t)response[6];
                        if (response[2] == 1) { p = -p; }
                        
                        int32_t corrected_p = state.invert ? -p : p;
                        
                        // Update the speed estimator used for damping (LQR/PID)
                        float s = state.speedEstimator.update(corrected_p, esp_timer_get_time());

                        async.speed_value.store(s);
                        async.encoder_value.store(corrected_p);
                        async.encoder_dirty.store(true);
                        async.last_encoder_time_us.store((uint64_t)esp_timer_get_time());
                        async.encoder_ok.fetch_add(1);
                    }
                    else {
                        async.encoder_timeout.fetch_add(1);
                        if (now - last_latency_log_ms > MKS_SERVO_DIAG_LOG_ms) {
                            LOG_PRINTF(abbot::log::CHANNEL_MOTOR, 
                                       "mks_servo: Motor ID 0x%02X encoder read TIMEOUT\n", state.id);
                            last_latency_log_ms = now;
                        }
                    }
                }
#endif
            }
            xSemaphoreGiveRecursive(mutex);
        }

        // Cooperative yield to avoid Watchdog triggers, especially if telemetry is failing.
        vTaskDelay(1);
    }
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

    if (g_step_dir_mutex && xSemaphoreTake(g_step_dir_mutex, 0) == pdTRUE) {
    uint32_t ledc_interval_us = 0;
#if MKS_SERVO_STEP_UPDATE_HZ > 0
    ledc_interval_us = (uint32_t)(1000000UL / (uint32_t)MKS_SERVO_STEP_UPDATE_HZ);
#endif
    
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
#if MKS_SERVO_COMMON_ANODE
            ledcWrite(0, (1 << MKS_SERVO_LEDC_RES) - 1);
#else
            ledcWrite(0, 0);
#endif
            m_left_async.last_ledc_update_us = micros();
        } else {
            uint32_t freq = (uint32_t)((left_abs * VELOCITY_MAX_SPEED / 60.0f) * MKS_SERVO_STEPS_PER_REV);
            if (freq > MKS_SERVO_STEP_MAX_HZ) freq = MKS_SERVO_STEP_MAX_HZ;
            if (freq < 1) freq = 1;

            uint32_t now_us = micros();
            bool time_ok = (ledc_interval_us == 0U) || ((int32_t)(now_us - m_left_async.last_ledc_update_us) >= (int32_t)ledc_interval_us);
            if (time_ok && abs((int)freq - (int)m_left_async.last_ledc_freq) > MKS_SERVO_STEP_DEADBAND_HZ) {
                m_left_async.last_ledc_freq = freq;
                m_left_async.last_ledc_update_us = now_us;
                ledcWriteTone(0, freq);
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
#if MKS_SERVO_COMMON_ANODE
            ledcWrite(2, (1 << MKS_SERVO_LEDC_RES) - 1);
#else
            ledcWrite(2, 0);
#endif
            m_right_async.last_ledc_update_us = micros();
        } else {
            uint32_t freq = (uint32_t)((right_abs * VELOCITY_MAX_SPEED / 60.0f) * MKS_SERVO_STEPS_PER_REV);
            if (freq > MKS_SERVO_STEP_MAX_HZ) freq = MKS_SERVO_STEP_MAX_HZ;
            if (freq < 1) freq = 1;

            uint32_t now_us = micros();
            bool time_ok = (ledc_interval_us == 0U) || ((int32_t)(now_us - m_right_async.last_ledc_update_us) >= (int32_t)ledc_interval_us);
            if (time_ok && abs((int)freq - (int)m_right_async.last_ledc_freq) > MKS_SERVO_STEP_DEADBAND_HZ) {
                m_right_async.last_ledc_freq = freq;
                m_right_async.last_ledc_update_us = now_us;
                ledcWriteTone(2, freq);
            }
        }
    }

        xSemaphoreGive(g_step_dir_mutex);
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
    if (g_step_dir_mutex && xSemaphoreTake(g_step_dir_mutex, 0) == pdTRUE) {
    uint32_t ledc_interval_us = 0;
#if MKS_SERVO_STEP_UPDATE_HZ > 0
    ledc_interval_us = (uint32_t)(1000000UL / (uint32_t)MKS_SERVO_STEP_UPDATE_HZ);
#endif
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
#if MKS_SERVO_COMMON_ANODE
            ledcWrite(0, (1 << MKS_SERVO_LEDC_RES) - 1);
#else
            ledcWrite(0, 0);
#endif
            m_left_async.last_ledc_update_us = micros();
        } else {
            uint32_t freq = (uint32_t)((left_abs * VELOCITY_MAX_SPEED / 60.0f) * MKS_SERVO_STEPS_PER_REV);
            if (freq > MKS_SERVO_STEP_MAX_HZ) freq = MKS_SERVO_STEP_MAX_HZ;
            if (freq < 1) freq = 1;
            
            uint32_t now_us = micros();
            bool time_ok = (ledc_interval_us == 0U) || ((int32_t)(now_us - m_left_async.last_ledc_update_us) >= (int32_t)ledc_interval_us);
            if (time_ok && abs((int)freq - (int)m_left_async.last_ledc_freq) > 2) {
                m_left_async.last_ledc_freq = freq;
                m_left_async.last_ledc_update_us = now_us;
                ledcWriteTone(0, freq);
            }
        }

            xSemaphoreGive(g_step_dir_mutex);
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
    if (g_step_dir_mutex && xSemaphoreTake(g_step_dir_mutex, 0) == pdTRUE) {
    uint32_t ledc_interval_us = 0;
#if MKS_SERVO_STEP_UPDATE_HZ > 0
    ledc_interval_us = (uint32_t)(1000000UL / (uint32_t)MKS_SERVO_STEP_UPDATE_HZ);
#endif
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
#if MKS_SERVO_COMMON_ANODE
            ledcWrite(2, (1 << MKS_SERVO_LEDC_RES) - 1);
#else
            ledcWrite(2, 0);
#endif
            m_right_async.last_ledc_update_us = micros();
        } else {
            uint32_t freq = (uint32_t)((right_abs * VELOCITY_MAX_SPEED / 60.0f) * MKS_SERVO_STEPS_PER_REV);
            if (freq > MKS_SERVO_STEP_MAX_HZ) freq = MKS_SERVO_STEP_MAX_HZ;
            if (freq < 1) freq = 1;

            uint32_t now_us = micros();
            bool time_ok = (ledc_interval_us == 0U) || ((int32_t)(now_us - m_right_async.last_ledc_update_us) >= (int32_t)ledc_interval_us);
            if (time_ok && abs((int)freq - (int)m_right_async.last_ledc_freq) > 2) {
                m_right_async.last_ledc_freq = freq;
                m_right_async.last_ledc_update_us = now_us;
                ledcWriteTone(2, freq);
            }
        }

            xSemaphoreGive(g_step_dir_mutex);
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

void MksServoMotorDriver::calibrateMotor(uint8_t id) {
    LOG_PRINTF(abbot::log::CHANNEL_MOTOR, "mks_servo: starting calibration for ID 0x%02X", id);
    sendFunctionCommand(id, 0x80, nullptr, 0);
}

void MksServoMotorDriver::scanBus() {
    LOG_PRINTLN(abbot::log::CHANNEL_MOTOR, "mks_servo: starting scan on both Serial1 and Serial2 (IDs 1-15)...");

    int found_count = 0;
    HardwareSerial* ports[] = {&Serial1, &Serial2};
    const char* port_names[] = {"Serial1", "Serial2"};

    for (int portIndex = 0; portIndex < 2; ++portIndex) {
        HardwareSerial& serial = *ports[portIndex];
        LOG_PRINTF(abbot::log::CHANNEL_MOTOR, "Scanning %s...\n", port_names[portIndex]);
        
        for (uint8_t id = 1; id <= 15; ++id) {
            SemaphoreHandle_t mutex = getMutexForMotor(id);
            if (xSemaphoreTakeRecursive(mutex, pdMS_TO_TICKS(50)) != pdTRUE) {
                continue;
            }

            // Clear any leftover data in RX buffer
            while (serial.available() > 0) {
                serial.read();
            }

            // Send read position request (0x31) - it's a safe non-destructive read
            uint8_t frame[4];
            frame[0] = 0xFA;
            frame[1] = id;
            frame[2] = 0x31;
            frame[3] = calculateChecksum(frame, 3);

            writeFrame(serial, frame, 4);

            // Response is 10 bytes for 0x31
            uint8_t responseBuffer[10];
            bool success = readResponse(serial, id, 0x31, responseBuffer, 10, MKS_SERVO_TIMEOUT_HEAVY_US);
            xSemaphoreGiveRecursive(mutex);

            if (success) {
                LOG_PRINTF(abbot::log::CHANNEL_MOTOR, "mks_servo: Found motor at ID 0x%02X on %s", id, port_names[portIndex]);
                found_count++;
            }
            // Small delay to not overwhelm the bus
            delay(5);
        }
    }

    LOG_PRINTF(abbot::log::CHANNEL_MOTOR, "mks_servo: scan finished. Found %d motors total.", found_count);
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
        dumpMotorRegisters(m_left.id);
        dumpMotorRegisters(m_right.id);
        return true;
    }

    if (line.equalsIgnoreCase("ENABLE LEFT") || line.equalsIgnoreCase("MOTOR ENABLE LEFT")) {
        setEnable(m_left.id, true);
        LOG_PRINTLN(abbot::log::CHANNEL_MOTOR, "mks_servo: manual ENABLE LEFT");
        return true;
    }
    if (line.equalsIgnoreCase("ENABLE RIGHT") || line.equalsIgnoreCase("MOTOR ENABLE RIGHT")) {
        setEnable(m_right.id, true);
        LOG_PRINTLN(abbot::log::CHANNEL_MOTOR, "mks_servo: manual ENABLE RIGHT");
        return true;
    }
    if (line.equalsIgnoreCase("DISABLE LEFT") || line.equalsIgnoreCase("MOTOR DISABLE LEFT")) {
        setEnable(m_left.id, false);
        LOG_PRINTLN(abbot::log::CHANNEL_MOTOR, "mks_servo: manual DISABLE LEFT");
        return true;
    }
    if (line.equalsIgnoreCase("DISABLE RIGHT") || line.equalsIgnoreCase("MOTOR DISABLE RIGHT")) {
        setEnable(m_right.id, false);
        LOG_PRINTLN(abbot::log::CHANNEL_MOTOR, "mks_servo: manual DISABLE RIGHT");
        return true;
    }

    if (line.equalsIgnoreCase("MOTOR DUMP LEFT") || line.equalsIgnoreCase("DUMP LEFT")) {
        dumpMotorRegisters(m_left.id);
        return true;
    }
    if (line.equalsIgnoreCase("MOTOR DUMP RIGHT") || line.equalsIgnoreCase("DUMP RIGHT")) {
        dumpMotorRegisters(m_right.id);
        return true;
    }

    return AbstractMotorDriver::processSerialCommand(line);
}

void MksServoMotorDriver::dumpMotorRegisters(uint8_t id) {
    LOG_PRINTF(abbot::log::CHANNEL_DEFAULT, "--- mks_servo: Dumping registers for motor ID 0x%02X ---\n", id);
    
    // Commands to read: Pos(0x31), Speed(0x32), TargetPos(0x33), Pulse(0x34), Error(0x35), IO(0x36)
    struct RegisterDefinition { uint8_t code; const char* name; size_t length; };
    RegisterDefinition registerDefinitions[] = {
        {0x31, "Position ", 10},
        {0x32, "Read Speed", 6},
        {0x33, "Target Pos", 10},
        {0x34, "Pulses   ", 10},
        {0x35, "Error    ", 10},
        {0x36, "IO Status", 6}
    };

    for (const auto& reg : registerDefinitions) {
        SemaphoreHandle_t mutex = getMutexForMotor(id);
        if (xSemaphoreTakeRecursive(mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
            continue;
        }

        HardwareSerial& serial = getSerialForMotor(id);

        // Clear any leftover data in RX buffer
        while (serial.available() > 0) {
            serial.read();
        }

        uint8_t frame[4];
        frame[0] = 0xFA;
        frame[1] = id;
        frame[2] = reg.code;
        frame[3] = calculateChecksum(frame, 3);
        writeFrame(serial, frame, 4);
        
        uint8_t responseBuffer[16];
        bool success = readResponse(serial, id, reg.code, responseBuffer, reg.length, MKS_SERVO_TIMEOUT_HEAVY_US);
        xSemaphoreGiveRecursive(mutex);

        if (success) {
            if (reg.code == 0x31 || reg.code == 0x33 || reg.code == 0x35) {
                // Parse for 0x31/0x33/0x35: [H1] [H0] [P3] [P2] [P1] [P0] (48-bit pos)
                // We show the lower 32-bit as Pos and byte 3 as Status/H1
                int32_t pos = ((int32_t)responseBuffer[5] << 24) | ((int32_t)responseBuffer[6] << 16) | 
                              ((int32_t)responseBuffer[7] << 8) | (int32_t)responseBuffer[8];
                uint8_t status = responseBuffer[3];
                
                // Handle sign from status byte (bit 0)
                if (status & 0x01) {
                    pos = -pos;
                }

                LOG_PRINTF(abbot::log::CHANNEL_DEFAULT, "  [0x%02X] %s: Pos: %ld Status: 0x%02X\n", 
                           reg.code, reg.name, (long)pos, status);
            } else {
                String hexDataOutput = "";
                for (size_t i = 3; i < reg.length - 1; ++i) {
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

void MksServoMotorDriver::sendSpeedCommand(uint8_t id, float normalized_speed, bool invert, bool wait_for_ack) {
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

    AsyncState& async = (id == m_left.id) ? m_left_async : m_right_async;
    (void)wait_for_ack;

    // 3. Construct 0xF6 Frame (7 bytes)
    uint8_t frame[7];
    frame[0] = 0xFA;
    frame[1] = id;
    frame[2] = 0xF6;
    frame[3] = (direction << 7) | ((speedValueUint >> 8) & 0x0F);
    frame[4] = speedValueUint & 0xFF;
    frame[5] = (uint8_t)m_speed_accel.load();
    frame[6] = calculateChecksum(frame, 6);

    HardwareSerial& serial = getSerialForMotor(id);
    writeFrame(serial, frame, 7);

    async.last_cmd_send_time_us.store((uint64_t)esp_timer_get_time());
    async.speed_cmd_sent.fetch_add(1);
}


void MksServoMotorDriver::setMode(uint8_t id, MksServoMode mode) {
    uint8_t data = (uint8_t)mode;
    sendFunctionCommand(id, 0x82, &data, 1);
}

void MksServoMotorDriver::setMStep(uint8_t id, MksServoMicrostep mstep) {
    uint8_t data = (uint8_t)mstep;
    sendFunctionCommand(id, 0x84, &data, 1);
}

void MksServoMotorDriver::setCurrent(uint8_t id, uint16_t current_ma) {
    if (current_ma > 3000) current_ma = 3000;
    uint8_t data[2];
    data[0] = (current_ma >> 8) & 0xFF; // High
    data[1] = current_ma & 0xFF;        // Low
    sendFunctionCommand(id, 0x83, data, 2);
}

void MksServoMotorDriver::setHoldCurrent(uint8_t id, MksServoHoldCurrent hold_pct) {
    uint8_t data = (uint8_t)hold_pct;
    sendFunctionCommand(id, 0x9B, &data, 1);
}

void MksServoMotorDriver::setEnable(uint8_t id, bool enable) {
    if (id == m_left.id) {
        m_left.enabled = enable;
    }
    if (id == m_right.id) {
        m_right.enabled = enable;
    }

    // Support for physical EN pins
    bool pin_level = enable ? LOW : HIGH;
#if !MKS_SERVO_COMMON_ANODE
    pin_level = !pin_level;
#endif
    if (id == m_left.id) digitalWrite(MKS_SERVO_P1_EN_PIN, pin_level);
    else if (id == m_right.id) digitalWrite(MKS_SERVO_P2_EN_PIN, pin_level);

    uint8_t data = enable ? 0x01 : 0x00;
    sendFunctionCommand(id, 0xF3, &data, 1);
}

void MksServoMotorDriver::sendFunctionCommand(uint8_t id, uint8_t function_code, const uint8_t* data, size_t length) {
    QueueHandle_t queue = getQueueForMotor(id);
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
                       "mks_servo: WARN: func queue full (ID=0x%02X FUNC=0x%02X)\n",
                       id, function_code);
        } else {
            // Notify task to process queued command promptly
            if (id == m_left.id && m_left_async.task_handle) {
                xTaskNotifyGive(m_left_async.task_handle);
            }
            if (id == m_right.id && m_right_async.task_handle) {
                xTaskNotifyGive(m_right_async.task_handle);
            }
        }
        return;
    }

    SemaphoreHandle_t mutex = getMutexForMotor(id);
    if (xSemaphoreTakeRecursive(mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        return;
    }

    // Max frame size is approx 7-10 bytes for these commands
    uint8_t frame[16];
    frame[0] = 0xFA;
    frame[1] = id;
    frame[2] = function_code;
    
    for (size_t i = 0; i < length; ++i) {
        frame[3 + i] = data[i];
    }
    
    size_t total_len = 3 + length + 1;
    frame[total_len - 1] = calculateChecksum(frame, total_len - 1);
    
    HardwareSerial& serial = getSerialForMotor(id);

    // Clear any leftover data in RX buffer before sending
    while (serial.available() > 0) {
        serial.read();
    }

    writeFrame(serial, frame, total_len);

    // Consume ACK: 0xFB [ID] [Function] [Status] [Checksum] (5 bytes)
    uint8_t ackBuffer[5];
    if (readResponse(serial, id, function_code, ackBuffer, 5, MKS_SERVO_TIMEOUT_DATA_US)) {
        // Optional: you can log success here if needed
        // LOG_PRINTF(abbot::log::CHANNEL_MOTOR, "mks_servo: ID 0x%02X FUNC 0x%02X ACK status=0x%02X\n", id, function_code, ackBuffer[3]);
    } else {
        LOG_PRINTF(abbot::log::CHANNEL_MOTOR, "mks_servo: WARN: No ACK for FUNC 0x%02X to ID 0x%02X\n", function_code, id);
    }

    xSemaphoreGiveRecursive(mutex);
}

uint8_t MksServoMotorDriver::calculateChecksum(const uint8_t* data, size_t length) {
    uint8_t checksum = 0;
    for (size_t i = 0; i < length; ++i) {
        checksum += data[i];
    }
    return checksum;
}

HardwareSerial& MksServoMotorDriver::getSerialForMotor(uint8_t id) {
    if (id == LEFT_MOTOR_ID) {
        return Serial2;
    }
    if (id == RIGHT_MOTOR_ID) {
        return Serial1;
    }
    // Fallback/Default for unknown IDs
    return Serial1;
}

SemaphoreHandle_t MksServoMotorDriver::getMutexForMotor(uint8_t id) {
    if (id == LEFT_MOTOR_ID) {
        return m_leftBusMutex;
    }
    return m_rightBusMutex;
}

QueueHandle_t MksServoMotorDriver::getQueueForMotor(uint8_t id) {
    if (id == LEFT_MOTOR_ID) {
        return m_leftCommandQueue;
    }
    return m_rightCommandQueue;
}

void MksServoMotorDriver::writeFrame(HardwareSerial& serial, const uint8_t* frame, size_t length) {
    serial.write(frame, length);
    serial.flush();
}

bool MksServoMotorDriver::verifyConfig(uint8_t id, uint8_t function_code, uint8_t expected_value, const char* label) {
    SemaphoreHandle_t mutex = getMutexForMotor(id);
    if (xSemaphoreTakeRecursive(mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        return false;
    }

    HardwareSerial& serial = getSerialForMotor(id);

    // Clear buffer before making a new synchronous request
    while (serial.available() > 0) {
        serial.read();
    }

    uint8_t frame[4];
    frame[0] = 0xFA;
    frame[1] = id;
    frame[2] = function_code;
    frame[3] = calculateChecksum(frame, 3);
    writeFrame(serial, frame, 4);

    uint8_t responseBuffer[16];
    size_t expected_len = (function_code == 0x31 || function_code == 0x34 || function_code == 0x35) ? 10 : 5;

    bool success = readResponse(serial, id, function_code, responseBuffer, expected_len, MKS_SERVO_TIMEOUT_HEAVY_US);
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

bool MksServoMotorDriver::readResponse(HardwareSerial& serial, uint8_t id, uint8_t function_code, uint8_t* out_data, size_t expected_length, uint32_t timeout_us) {
    unsigned long start_us = micros();
    size_t received = 0;
    
    bool use_ack_header = (function_code >= 0x80);
    uint8_t start_byte = use_ack_header ? 0xFB : id;
    size_t echo_skip_remaining = 0;
    size_t request_length = use_ack_header ? 5U : 4U;
    
    while (micros() - start_us < timeout_us) {
        if (serial.available()) {
            uint8_t b = serial.read();

            if (echo_skip_remaining > 0) {
                echo_skip_remaining--;
                continue;
            }
            
            if (received == 0) {
                if (b == 0xFA) {
                    echo_skip_remaining = request_length - 1;
                    continue;
                }

                if (b == start_byte) {
                    out_data[received++] = b;
                }
                // Skip bytes scanning for header
                continue;
            }

            // Data phase
            out_data[received++] = b;
            
            // Header validation (ID + Function)
            if (received == 2) {
                if (!use_ack_header && out_data[1] != function_code) {
                    // Mismatch - maybe part of a missed echo or junk. 
                    // Reset and check if THIS byte could be the start of a real header.
                    received = (b == start_byte) ? 1 : 0;
                    if (received == 1) {
                        out_data[0] = b;
                    }
                    continue;
                }
                if (use_ack_header && b != id) {
                    received = (b == start_byte) ? 1 : 0;
                    if (received == 1) {
                        out_data[0] = b;
                    }
                    continue;
                }
            }
            if (received == 3 && use_ack_header && b != function_code) {
                received = (b == start_byte) ? 1 : 0;
                if (received == 1) {
                    out_data[0] = b;
                }
                continue;
            }
            
            // Frame complete?
            if (received >= expected_length) {
                uint8_t actual_cs = calculateChecksum(out_data, expected_length - 1);
                if (out_data[expected_length - 1] == actual_cs) {
                    return true;
                } else {
                    LOG_PRINTF(abbot::log::CHANNEL_MOTOR,
                               "mks_servo: CS error on ID 0x%02X FUNC 0x%02X (Got 0x%02X, expected 0x%02X)\n",
                               id, function_code, out_data[expected_length - 1], actual_cs);
                    received = 0;
                }
            }
        } else {
            // yielding is important for high priority tasks
            vTaskDelay(1);
        }

    #if !defined(UNIT_TEST_HOST)
        // Avoid busy-waiting at 100% CPU during timeouts.
        // This improves IMU/control scheduling when telemetry is failing.
        taskYIELD();
    #endif
    }
    return false;
}

} // namespace motor
} // namespace abbot
