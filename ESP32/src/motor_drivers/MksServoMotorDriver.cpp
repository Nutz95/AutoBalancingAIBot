// MksServoMotorDriver.cpp - Implementation for MKS SERVO42D/57D serial bus motors
#include "../../include/motor_drivers/MksServoMotorDriver.h"
#include "../../include/balancer_controller.h"
#include <Arduino.h>

namespace abbot {
namespace motor {

MksServoMotorDriver::MksServoMotorDriver()
    : m_left{LEFT_MOTOR_ID, LEFT_MOTOR_INVERT != 0, 0.0f, 0, 0, 0, 0, 0, false, SpeedEstimator(0.1f)},
      m_right{RIGHT_MOTOR_ID, RIGHT_MOTOR_INVERT != 0, 0.0f, 0, 0, 0, 0, 0, false, SpeedEstimator(0.1f)},
      m_enabled(false),
      m_leftBusMutex(xSemaphoreCreateMutex()),
      m_rightBusMutex(xSemaphoreCreateMutex()) {}

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
        setMode(id, MKS_SERVO_DEFAULT_MODE);
        delay(10);
        setMStep(id, MKS_SERVO_DEFAULT_MSTEP);
        delay(10);
        setCurrent(id, MKS_SERVO_MA);
        delay(10);
        setHoldCurrent(id, MKS_SERVO_HOLD_PCT);
        delay(10);

        LOG_PRINTF(abbot::log::CHANNEL_DEFAULT,
                   "mks_servo: motor ID 0x%02X configuration complete\n", id);
    }

    // Ensure motors are initially disabled and stopped for safety
    disableMotors();
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
    m_enabled = true;
    setEnable(m_left.id, true);
    setEnable(m_right.id, true);
    
    LOG_PRINTLN(abbot::log::CHANNEL_MOTOR, "mks_servo: motors ENABLED");
}

void MksServoMotorDriver::disableMotors() {
    m_enabled = false;

    // Send 0 speed first to ensure motors stop before disabling torque
    sendSpeedCommand(m_left.id, 0.0f, false);
    sendSpeedCommand(m_right.id, 0.0f, false);

    setEnable(m_left.id, false);
    setEnable(m_right.id, false);

    LOG_PRINTLN(abbot::log::CHANNEL_MOTOR, "mks_servo: motors DISABLED");
}

bool MksServoMotorDriver::areMotorsEnabled() {
    return m_enabled;
}

void MksServoMotorDriver::printStatus() {
    LOG_PRINTF(abbot::log::CHANNEL_MOTOR,
               "mks_servo: enabled=%d L_cmd=%.2f R_cmd=%.2f\n",
               m_enabled, m_left.last_command, m_right.last_command);
}

void MksServoMotorDriver::dumpConfig() {
    LOG_PRINTF(abbot::log::CHANNEL_MOTOR,
               "mks_servo: L_ID=0x%02X R_ID=0x%02X Baud=%d Ma=%d\n",
               m_left.id, m_right.id, MKS_SERVO_BAUD, MKS_SERVO_MA);
}

void MksServoMotorDriver::setMotorCommandBoth(float left_command, float right_command) {
    // If not enabled at all, just track commands and return
    if (!m_enabled) {
        m_left.last_command = left_command;
        m_right.last_command = right_command;
        return;
    }

    // Helper to prepare frame data for Speed Command (0xF6)
    auto prepareSpeedFrame = [this](float command, MotorState& state, uint8_t* frame) {
        float clamped = command;
        if (clamped > 1.0f) clamped = 1.0f;
        if (clamped < -1.0f) clamped = -1.0f;
        
        state.last_command = clamped;
        state.last_command_time_us = micros();

        float speedValue = clamped;
        if (state.invert) {
            speedValue = -speedValue;
        }

        uint8_t direction = (speedValue >= 0) ? 0 : 1;
        float absoluteSpeed = (speedValue >= 0) ? speedValue : -speedValue;

        uint32_t speedValueCalc = (uint32_t)(absoluteSpeed * (float)VELOCITY_MAX_SPEED);
        if (speedValueCalc > 3000) {
            speedValueCalc = 3000;
        }
        uint16_t speedValueUint = (uint16_t)speedValueCalc;

        frame[0] = 0xFA;
        frame[1] = state.id;
        frame[2] = 0xF6;
        frame[3] = (direction << 7) | ((speedValueUint >> 8) & 0x0F);
        frame[4] = speedValueUint & 0xFF;
        frame[5] = MKS_SERVO_ACCEL;
        frame[6] = calculateChecksum(frame, 6);
    };

    uint8_t frameLeft[7], frameRight[7];
    prepareSpeedFrame(left_command, m_left, frameLeft);
    prepareSpeedFrame(right_command, m_right, frameRight);

    unsigned long start_us = micros();

    // Take both mutexes to ensure exclusive access to both buses
    // We use a fixed order (Left then Right) to avoid circular wait/deadlocks.
    if (xSemaphoreTake(m_leftBusMutex, pdMS_TO_TICKS(MKS_SERVO_BUS_MUTEX_TIMEOUT_MS)) == pdTRUE) {
        if (xSemaphoreTake(m_rightBusMutex, pdMS_TO_TICKS(MKS_SERVO_BUS_MUTEX_TIMEOUT_MS)) == pdTRUE) {
            
            // 1. Dispatch Write frames to both UARTs (Non-blocking writes to buffers)
            if (m_left.enabled) {
                while (Serial2.available() > 0) Serial2.read(); // Clear RX
                Serial2.write(frameLeft, 7);
            }
            if (m_right.enabled) {
                while (Serial1.available() > 0) Serial1.read(); // Clear RX
                Serial1.write(frameRight, 7);
            }

            // 2. Wait for ACKs sequentially. 
            // While we wait for Serial2, Serial1 is already receiving data in background.
            uint8_t ackBuffer[5];
            if (m_left.enabled) {
                readResponse(Serial2, m_left.id, 0xF6, ackBuffer, 5, MKS_SERVO_TIMEOUT_CONTROL_US);
            }
            if (m_right.enabled) {
                readResponse(Serial1, m_right.id, 0xF6, ackBuffer, 5, MKS_SERVO_TIMEOUT_CONTROL_US);
            }

            last_bus_latency_us_ = (uint32_t)(micros() - start_us);
            xSemaphoreGive(m_rightBusMutex);
        }
        xSemaphoreGive(m_leftBusMutex);
    }
}

void MksServoMotorDriver::setMotorCommand(MotorSide side, float command) {
    MotorState& state = (side == MotorSide::LEFT) ? m_left : m_right;
    
    // Clamp command to [-1.0, 1.0]
    if (command > 1.0f) command = 1.0f;
    if (command < -1.0f) command = -1.0f;

    state.last_command = command;
    state.last_command_time_us = micros();

    // Log command entry timestamp for characterization tools
    if (command != 0.0f) {
        // ESP_LOGI("MksServoDriver", ...); // Commented out to reduce spam
    }

    // Only send the command if this specific motor is enabled
    if (state.enabled) {
        // Wait for bus mutex. In the critical control path, we should wait long enough 
        // to not drop frames just because of a telemetry read (up to 20ms).
        SemaphoreHandle_t mutex = getMutexForMotor(state.id);
        if (xSemaphoreTake(mutex, pdMS_TO_TICKS(MKS_SERVO_BUS_MUTEX_TIMEOUT_MS)) == pdTRUE) {
            xSemaphoreGive(mutex); // We just checked availability, sendSpeedCommand takes it internally
            sendSpeedCommand(state.id, command, state.invert);
            // Delay for bus recovery/motor processing
            delayMicroseconds(MKS_SERVO_BUS_QUIET_US);
        } else {
             // Avoid slow logging in the critical loop. Use a static counter if needed.
             // LOG_PRINTLN(abbot::log::CHANNEL_MOTOR, "mks_servo: DROP CMD - Bus Locked");
        }
    }
}

void MksServoMotorDriver::setMotorCommandRaw(MotorSide side, int16_t rawSpeed) {
    // For MKS Servo, we map raw speed to normalized range for simplicity,
    // assuming rawSpeed is roughly centered around the max RPM (0-3000).
    float norm = (float)rawSpeed / (float)VELOCITY_MAX_SPEED;
    setMotorCommand(side, norm);
}

int32_t MksServoMotorDriver::readEncoder(MotorSide side) {
    // If the balancer is active, prioritize the bus 100% for control commands.
    // Telemetry reads (which take ~3ms) cause latency spikes that can destabilize the PID.
    if (abbot::balancer::controller::isActive()) {
        MotorState& state = (side == MotorSide::LEFT) ? m_left : m_right;
        return state.last_encoder;
    }

    MotorState& state = (side == MotorSide::LEFT) ? m_left : m_right;
    uint32_t now = millis();

    // Throttle encoder reads to keep the RS485 bus clear for control commands
    if (now - state.last_encoder_read_ms < MKS_SERVO_ENCODER_READ_MS) {
        return state.last_encoder;
    }

    state.last_encoder_read_ms = now;

    // Telemetry should yield easily. If bus is busy (e.g. control loop), skip read.
    SemaphoreHandle_t mutex = getMutexForMotor(state.id);
    if (xSemaphoreTake(mutex, 0) != pdTRUE) {
        return state.last_encoder;
    }

    HardwareSerial& serial = getSerialForMotor(state.id);

    // Clear any leftover data in RX buffer
    while (serial.available() > 0) {
        serial.read();
    }

    // Send read position request (0x31)
    uint8_t frame[4];
    frame[0] = 0xFA;
    frame[1] = (uint8_t)state.id;
    frame[2] = 0x31;
    frame[3] = calculateChecksum(frame, 3);
    
    writeFrame(serial, frame, 4);

    // Response is 10 bytes: [0xFB] [Addr] [0x31] [Dir] [Pos3] [Pos2] [Pos1] [Pos0] [Status] [Checksum]
    uint8_t responseBuffer[10];
    if (readResponse(serial, state.id, 0x31, responseBuffer, 10, MKS_SERVO_TIMEOUT_DATA_US)) {
        // Position is at indices 5, 6, 7, 8
        int32_t pos = ((int32_t)responseBuffer[5] << 24) |
                      ((int32_t)responseBuffer[6] << 16) |
                      ((int32_t)responseBuffer[7] << 8) |
                      ((int32_t)responseBuffer[8]);
        
        // Handle sign from status byte (bit 0) if provided by motor
        if (responseBuffer[3] & 0x01) {
            pos = -pos;
        }
        
        state.last_encoder = state.invert ? -pos : pos;
        state.speedEstimator.update(state.last_encoder, micros());
    }

    xSemaphoreGive(mutex);
    return state.last_encoder;
}

float MksServoMotorDriver::readSpeed(MotorSide side) {
    MotorState& state = (side == MotorSide::LEFT) ? m_left : m_right;
    return state.speedEstimator.get();
}

void MksServoMotorDriver::resetSpeedEstimator() {
    m_left.speedEstimator.reset();
    m_right.speedEstimator.reset();
}

void MksServoMotorDriver::resetPositionTracking() {
    // MKS Servo doesn't have a simple serial command to zero internal encoder 
    // that is safe to call while running. We just zero our local tracking.
    m_left.last_encoder = 0;
    m_right.last_encoder = 0;
}

uint64_t MksServoMotorDriver::getLastCommandTimeUs(MotorSide side) const {
    if (side == MotorSide::LEFT) {
        return m_left.last_command_time_us;
    }
    return m_right.last_command_time_us;
}

int MksServoMotorDriver::getMotorId(MotorSide side) const {
    if (side == MotorSide::LEFT) {
        return m_left.id;
    }
    return m_right.id;
}

bool MksServoMotorDriver::isMotorInverted(MotorSide side) const {
    if (side == MotorSide::LEFT) {
        return m_left.invert;
    }
    return m_right.invert;
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
            if (xSemaphoreTake(mutex, pdMS_TO_TICKS(50)) != pdTRUE) {
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
            xSemaphoreGive(mutex);

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
    if (line.equalsIgnoreCase("SCAN") || line.startsWith("SCAN ") ||
        line.equalsIgnoreCase("MOTOR SCAN") || line.startsWith("MOTOR SCAN ")) {
        scanBus();
        return true;
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
        if (xSemaphoreTake(mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
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
        xSemaphoreGive(mutex);

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

void MksServoMotorDriver::sendSpeedCommand(uint8_t id, float normalized_speed, bool invert) {
    // normalized_speed is [-1.0, 1.0]
    // 1. Handle inversion and direction
    float speedValue = normalized_speed;
    if (invert) {
        speedValue = -speedValue;
    }

    uint8_t direction = (speedValue >= 0) ? 0 : 1;
    float absoluteSpeed = (speedValue >= 0) ? speedValue : -speedValue;

    // 2. Map to 12-bit speed (0-3000 RPM)
    // Use 32-bit types for calculations before casting to 16-bit
    uint32_t speedValueCalc = (uint32_t)(absoluteSpeed * (float)VELOCITY_MAX_SPEED);
    if (speedValueCalc > 3000) {
        speedValueCalc = 3000;
    }
    uint16_t speedValueUint = (uint16_t)speedValueCalc;

    // Throttled logging for transparency
    MotorState& state = (id == m_left.id) ? m_left : m_right;
    uint32_t now_ms = millis();
    unsigned long start_us = micros();

    SemaphoreHandle_t mutex = getMutexForMotor(id);
    if (xSemaphoreTake(mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        return;
    }

    // 3. Construct 0xF6 Frame (7 bytes)
    // [0xFA] [Addr] [0xF6] [Dir/SpeedH] [SpeedL] [Acc] [Checksum]
    uint8_t frame[7];
    frame[0] = 0xFA;
    frame[1] = id;
    frame[2] = 0xF6;
    frame[3] = (direction << 7) | ((speedValueUint >> 8) & 0x0F);
    frame[4] = speedValueUint & 0xFF;
    frame[5] = MKS_SERVO_ACCEL;
    frame[6] = calculateChecksum(frame, 6);

    HardwareSerial& serial = getSerialForMotor(id);

    // Clear any leftover data in RX buffer before sending
    while (serial.available() > 0) {
        serial.read();
    }

    writeFrame(serial, frame, 7);
    
    // Consume ACK: 0xFB [ID] 0xF6 [Status] [Checksum] (5 bytes)
    uint8_t ackBuffer[5];
    // Reduce timeout for critical speed command ACK.
    // readResponse naturally skips the TX echo because it filters for header 0xFB.
    bool ackOk = readResponse(serial, id, 0xF6, ackBuffer, 5, MKS_SERVO_TIMEOUT_CONTROL_US);
    
    unsigned long duration_us = micros() - start_us;
    last_bus_latency_us_ = (uint32_t)duration_us;

    // Log bus utilization stats occasionally
    static unsigned long total_bus_time_us = 0;
    static unsigned int bus_transaction_count = 0;
    static unsigned long last_stat_log_ms = 0;
    
    total_bus_time_us += duration_us;
    bus_transaction_count++;

    // Guard against overflow
    if (bus_transaction_count > 10000) {
        total_bus_time_us = 0;
        bus_transaction_count = 0;
    }

    if (now_ms - last_stat_log_ms > 2000) {
        float avg_us = (bus_transaction_count > 0) ? ((float)total_bus_time_us / bus_transaction_count) : 0.0f;
        // Estimate utilization: (avg_us * 2 motors * 200Hz loop?) 
        // We don't know the exact loop rate here, but we can report avg transaction time.
        // At 400Hz (2.5ms period), if avg_us is 1250us, we are at 50% capacity per motor -> 100% total!
        LOG_PRINTF(abbot::log::CHANNEL_MOTOR, 
            "RS485 STATS: Avg Tx Time=%.0f us. Last=%.0f us. (Budget per motor @400Hz ~1200us)\n", 
            (double)avg_us, (double)duration_us);
        
        last_stat_log_ms = now_ms;
        total_bus_time_us = 0;
        bus_transaction_count = 0;
    }

    if (ackOk) {
        // Only log if status is not success (0x01) and throttle to 1 second
        if (ackBuffer[3] != 0x01 && (now_ms - state.last_ack_log_ms >= 1000)) {
            LOG_PRINTF(abbot::log::CHANNEL_MOTOR, "mks_servo: ID 0x%02X command status=0x%02X (speed=%.3f)\n", 
                       id, ackBuffer[3], (double)normalized_speed);
            state.last_ack_log_ms = now_ms;
        }
    } else {
        if (now_ms - state.last_ack_log_ms >= 1000) {
            LOG_PRINTF(abbot::log::CHANNEL_MOTOR, "mks_servo: WARN: No ACK for speed command to ID 0x%02X (took %lu us)\n", id, duration_us);
            state.last_ack_log_ms = now_ms;
        }
    }

    xSemaphoreGive(mutex);
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
    if (id == m_left.id) m_left.enabled = enable;
    if (id == m_right.id) m_right.enabled = enable;

    uint8_t data = enable ? 0x01 : 0x00;
    sendFunctionCommand(id, 0xF3, &data, 1);
    // Send twice to ensure it's received, especially for disabling
    delay(5);
    sendFunctionCommand(id, 0xF3, &data, 1);
}

void MksServoMotorDriver::sendFunctionCommand(uint8_t id, uint8_t function_code, const uint8_t* data, size_t length) {
    SemaphoreHandle_t mutex = getMutexForMotor(id);
    if (xSemaphoreTake(mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
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

    xSemaphoreGive(mutex);
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
    if (xSemaphoreTake(mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
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
    xSemaphoreGive(mutex);

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
    
    while (micros() - start_us < timeout_us) { 
        if (serial.available()) {
            uint8_t byteRead = serial.read();
            
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
                    // Wrong frame. Reset and check if this current byte could be the start of a new frame.
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
                
                // Bad checksum - reset and look for next header
                received = 0;
                // Again, check if the last byte read (the checksum byte) was 0xFB
                if (byteRead == 0xFB) {
                    out_data[0] = 0xFB;
                    received = 1;
                }
            }
        }
    }
    return false;
}

} // namespace motor
} // namespace abbot
