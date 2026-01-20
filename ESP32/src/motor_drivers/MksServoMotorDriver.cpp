// MksServoMotorDriver.cpp - Implementation for MKS SERVO42D/57D serial bus motors
#include "../../include/motor_drivers/MksServoMotorDriver.h"
#include "../../include/balancer_controller.h"
#include "../../config/balancer_config.h"
#include <Arduino.h>

namespace abbot {
namespace motor {

MksServoMotorDriver::MksServoMotorDriver()
    : m_left{LEFT_MOTOR_ID, LEFT_MOTOR_INVERT != 0, 0.0f, 0, 0, 0, 0, 0, false, SpeedEstimator(MKS_SERVO_SPEED_ALPHA)},
      m_right{RIGHT_MOTOR_ID, RIGHT_MOTOR_INVERT != 0, 0.0f, 0, 0, 0, 0, 0, false, SpeedEstimator(MKS_SERVO_SPEED_ALPHA)},
      m_enabled(false),
      m_leftBusMutex(xSemaphoreCreateRecursiveMutex()),
      m_rightBusMutex(xSemaphoreCreateRecursiveMutex()) {}

void MksServoMotorDriver::initMotorDriver() {
    // Initialize both serial ports at the configured baud rate
    // Motor 1 (Left) on Serial2
    Serial2.begin(MKS_SERVO_BAUD, SERIAL_8N1, MKS_SERVO_P1_RX_PIN, MKS_SERVO_P1_TX_PIN);
    // Motor 2 (Right) on Serial1
    Serial1.begin(MKS_SERVO_BAUD, SERIAL_8N1, MKS_SERVO_P2_RX_PIN, MKS_SERVO_P2_TX_PIN);
    
    LOG_PRINTF(abbot::log::CHANNEL_DEFAULT,
               "mks_servo: initialized Serial2 (Left, ID:%d) TX:%d RX:%d, Serial1 (Right, ID:%d) TX:%d RX:%d at %d baud\n",
               LEFT_MOTOR_ID, MKS_SERVO_P1_TX_PIN, MKS_SERVO_P1_RX_PIN,
               RIGHT_MOTOR_ID, MKS_SERVO_P2_TX_PIN, MKS_SERVO_P2_RX_PIN,
               MKS_SERVO_BAUD);

    // Give drivers time to boot
    delay(100);

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

    // Create background tasks for each motor bus on Core 0.
    // Pinned to Core 0 (PRO_CPU) and use high priority to ensure low command latency,
    // separated from the IMU/PID loop on Core 1 to avoid jitter interference.
    xTaskCreatePinnedToCore(motorTaskEntry, "motorL_task", 4096, new std::pair<MksServoMotorDriver*, MotorSide>(this, MotorSide::LEFT), 21, &m_left_async.task_handle, 0);
    xTaskCreatePinnedToCore(motorTaskEntry, "motorR_task", 4096, new std::pair<MksServoMotorDriver*, MotorSide>(this, MotorSide::RIGHT), 21, &m_right_async.task_handle, 0);
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
    
    while (true) {
        // High reactivity wait (1ms)
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(1)); 

        if (xSemaphoreTakeRecursive(mutex, pdMS_TO_TICKS(2)) == pdTRUE) {
            bool current_enabled = m_enabled;
            bool enabled_changed = (current_enabled != async.last_reported_enabled.load());
            bool speed_dirty = async.speed_dirty.exchange(false);
            
            // 1. Handle Torque State Changes
            if (enabled_changed) {
                uint8_t data = current_enabled ? 0x01 : 0x00;
                sendFunctionCommand(state.id, 0xF3, &data, 1);
                async.last_reported_enabled.store(current_enabled);
                speed_dirty = true;
            }

            // 2. Send Speed Command
            if (speed_dirty || (enabled_changed && current_enabled)) {
                float cmd = current_enabled ? async.target_speed.load() : 0.0f;
                // SPEED ACK: keep disabled during balancing; enable only for diagnostics.
                // Waiting for ACK adds latency and can reduce effective control bandwidth.
                bool waitAck = m_wait_for_ack.load();
                sendSpeedCommand(state.id, cmd, state.invert, waitAck);
                
                // Fixed delay after command to ensure it's processed and clear the bus
                // before any subsequent encoder reads.
                if (MKS_SERVO_BUS_QUIET_US > 0) {
                    delayMicroseconds(MKS_SERVO_BUS_QUIET_US);
                }
            } else if (!current_enabled) {
                // Occasional safety zeroing
                uint32_t now_ms = millis();
                if (now_ms - async.last_zero_ms > 200) {
                    sendSpeedCommand(state.id, 0.0f, state.invert, m_wait_for_ack.load());
                    async.last_zero_ms = now_ms;
                }
            }

            // 3. Telemetry read
            uint32_t now = millis();
            if (now - async.last_telemetry_ms >= telemetry_interval_ms) {
                // Clear any leftover ACKs from the speed commands
                while (serial.available() > 0) { serial.read(); }

                uint8_t readFrame[4];
                readFrame[0] = 0xFA;
                readFrame[1] = state.id;
                readFrame[2] = 0x31;
                readFrame[3] = calculateChecksum(readFrame, 3);
                writeFrame(serial, readFrame, 4);

                uint8_t response[10]; 
                bool got_response = readResponse(serial, state.id, 0x31, response, 10, MKS_SERVO_TIMEOUT_DATA_US);
                
                // DEBUG: Log encoder read failures (throttled to avoid spam)
                static uint32_t last_debug_ms = 0;
                if (!got_response && (now - last_debug_ms > MKS_SERVO_DIAG_LOG_ms)) {
                    LOG_PRINTF(abbot::log::CHANNEL_MOTOR, 
                               "mks_servo: Motor ID 0x%02X encoder read TIMEOUT on %s\n", 
                               state.id, (side == MotorSide::LEFT) ? "Serial2" : "Serial1");
                    last_debug_ms = now;
                }
                
                if (got_response) {
                    int32_t p = ((int32_t)response[5] << 24) | ((int32_t)response[6] << 16) | 
                                ((int32_t)response[7] << 8) | (int32_t)response[8];
                    if (response[3] == 1) { p = -p; }
                    
                    int32_t corrected_p = state.invert ? -p : p;
                    
                    // Update the speed estimator used for damping (LQR/PID)
                    // We use the absolute timestamp from esp_timer for better dt consistency
                    float s = state.speedEstimator.update(corrected_p, esp_timer_get_time());

                    async.speed_value.store(s);
                    async.encoder_value.store(corrected_p);
                    async.encoder_dirty.store(true);
                    async.last_telemetry_ms = now;
                    async.last_encoder_time_us.store((uint64_t)esp_timer_get_time());
                    async.encoder_ok.fetch_add(1);

                    // Diagnostic: Log non-zero speed once in a while to verify atomic store
                    static uint32_t last_speed_check_ms = 0;
                    if (fabsf(s) > 0.1f && (now - last_speed_check_ms > 5000)) {
                        LOG_PRINTF(abbot::log::CHANNEL_MOTOR, "mks_servo: ID 0x%02X speed_ticks_s = %.2f\n", state.id, (double)s);
                        last_speed_check_ms = now;
                    }
                }
                else {
                    async.encoder_timeout.fetch_add(1);
                }
            }
            xSemaphoreGiveRecursive(mutex);
        }
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
    // 1. Update targets (Atomics for background tasks)
    m_left_async.target_speed.store(left_command);
    m_left_async.speed_dirty.store(true);
    
    m_right_async.target_speed.store(right_command);
    m_right_async.speed_dirty.store(true);

    // 2. Notify background tasks to process immediately
    if (m_left_async.task_handle) {
        xTaskNotifyGive(m_left_async.task_handle);
    }
    if (m_right_async.task_handle) {
        xTaskNotifyGive(m_right_async.task_handle);
    }

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
        if (m_left_async.task_handle) {
            xTaskNotifyGive(m_left_async.task_handle);
        }
        m_left.last_command = command;
    } else {
        m_right_async.target_speed.store(command);
        m_right_async.speed_dirty.store(true);
        if (m_right_async.task_handle) {
            xTaskNotifyGive(m_right_async.task_handle);
        }
        m_right.last_command = command;
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

    MotorState& state = (id == m_left.id) ? m_left : m_right;
    AsyncState& async = (id == m_left.id) ? m_left_async : m_right_async;
    unsigned long start_us = micros();

    SemaphoreHandle_t mutex = getMutexForMotor(id);
    if (xSemaphoreTakeRecursive(mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        return;
    }

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
    
    bool ackOk = true;
    if (wait_for_ack) {
        // Consume ACK: 0xFB [ID] 0xF6 [Status] [Checksum] (5 bytes)
        uint8_t ackBuffer[5];
        ackOk = readResponse(serial, id, 0xF6, ackBuffer, 5, MKS_SERVO_TIMEOUT_CONTROL_US);
        
        if (!ackOk) {
            async.speed_ack_timeout.fetch_add(1);
        } else if (ackBuffer[3] != 0x01) {
            async.speed_ack_error.fetch_add(1);
            if ((millis() - state.last_ack_log_ms >= 1000)) {
                LOG_PRINTF(abbot::log::CHANNEL_MOTOR,
                           "mks_servo: ID 0x%02X command error status=0x%02X\n",
                           id, ackBuffer[3]);
                state.last_ack_log_ms = millis();
            }
        }
    }

    unsigned long duration_us = micros() - start_us;
    
    // Update per-motor latency atomic (includes ACK time when enabled)
    async.last_latency_us.store((uint32_t)duration_us);

    xSemaphoreGiveRecursive(mutex);
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

    uint8_t data = enable ? 0x01 : 0x00;
    sendFunctionCommand(id, 0xF3, &data, 1);
}

void MksServoMotorDriver::sendFunctionCommand(uint8_t id, uint8_t function_code, const uint8_t* data, size_t length) {
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

void MksServoMotorDriver::writeFrame(HardwareSerial& serial, const uint8_t* frame, size_t length) {
    serial.write(frame, length);
}

bool MksServoMotorDriver::verifyConfig(uint8_t id, uint8_t function_code, uint8_t expected_value, const char* label) {
    SemaphoreHandle_t mutex = getMutexForMotor(id);
    if (xSemaphoreTakeRecursive(mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        return false;
    }

    HardwareSerial& serial = getSerialForMotor(id);

    // Clear any leftover data in RX buffer
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
    // Blocking read with precision timeout
    unsigned long start_us = micros();
    size_t received = 0;
    uint32_t empty_checks = 0;
    
    while (micros() - start_us < timeout_us) { 
        if (serial.available()) {
            uint8_t byteRead = serial.read();
            empty_checks = 0;
            
            // Look for header 0xFB
            if (received == 0) {
                if (byteRead == 0xFB) {
                    out_data[0] = byteRead;
                    received = 1;
                }
                continue;
            }
            
            out_data[received++] = byteRead;
            
            // Validate ID and Cmd as soon as they are available (indices 1 and 2)
            if (received == 3) {
                if (out_data[1] != id || out_data[2] != function_code) {
                    received = 0;
                    if (byteRead == 0xFB) {
                        out_data[0] = 0xFB;
                        received = 1;
                    }
                    continue;
                }
            }
            
            if (received == expected_length) {
                // Verify checksum
                uint8_t calculatedChecksum = calculateChecksum(out_data, expected_length - 1);
                if (calculatedChecksum == out_data[expected_length - 1]) {
                    return true;
                }
                
                received = 0;
                if (byteRead == 0xFB) {
                    out_data[0] = 0xFB;
                    received = 1;
                }
            }
        } else {
            // No data available.
            empty_checks++;
            // Busy wait for the first 100 empty checks (approx 10-20us) matches typical inter-byte gap.
            // After that, yield to other tasks to prevent CPU starvation.
            if (empty_checks > 100) {
                vTaskDelay(0);
            }
        }
    }
    return false;
}

} // namespace motor
} // namespace abbot
