// MksServoMotorDriver.h - Driver for MKS SERVO42D/57D serial bus motors
#pragma once

#include "AbstractMotorDriver.h"
#include "../../config/motor_configs/mks_servo_config.h"
#include "speed_estimator.h"
#include <atomic>
#include <cstdint>

#if !defined(UNIT_TEST_HOST)
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/task.h>
#else
// Minimal stubs for host-native unit tests
typedef void* SemaphoreHandle_t;
class HardwareSerial {
public:
    void write(const uint8_t* buf, size_t len) {
    }

    int available() {
        return 0;
    }

    int read() {
        return -1;
    }

    size_t readBytes(uint8_t* buffer, size_t length) {
        return 0;
    }
};
#endif

namespace abbot {
namespace motor {

class MksServoMotorDriver : public AbstractMotorDriver {
public:
    MksServoMotorDriver();
    virtual ~MksServoMotorDriver() override = default;

    // IMotorDriver implementation
    void initMotorDriver() override;
    void clearCommandState() override;
    float getLastMotorCommand(MotorSide side) override;
    void enableMotors() override;
    void disableMotors() override;
    bool areMotorsEnabled() override;
    void printStatus() override;
    void dumpConfig() override;
    void setMotorCommandBoth(float left_command, float right_command) override;
    void readEncodersBoth(int32_t& left, int32_t& right) override;
    void setMotorCommand(MotorSide side, float command) override;
    void setMotorCommandRaw(MotorSide side, int16_t rawSpeed) override;
    int32_t readEncoder(MotorSide side) override;
    float readSpeed(MotorSide side) override;
    void resetSpeedEstimator() override;
    void resetPositionTracking() override;
    uint64_t getLastCommandTimeUs(MotorSide side) const override;

    // Configuration queries
    int getMotorId(MotorSide side) const override;
    bool isMotorInverted(MotorSide side) const override;
    float getVelocityMaxSpeed() const override;
    float getVelocityTargetIncrementScale() const override;
    float getVelocityPositionKp() const override;

    const char *getDriverName() const override {
        return "mks_servo";
    }

    uint32_t getLastBusLatencyUs() const override {
        uint32_t l = m_left_async.last_latency_us.load();
        uint32_t r = m_right_async.last_latency_us.load();
        return (l > r) ? l : r;
    }

    uint32_t getSpeedCommandAccel() const override {
        return (uint32_t)m_speed_accel.load();
    }

    // Serial command interface
    bool processSerialCommand(const String &line) override;

    // Custom calibration and scan commands
    void calibrateMotor(uint8_t id);
    void scanBus();

private:
    struct MotorState {
        int id;
        bool invert;
        float last_command;
        int32_t last_encoder;
        uint64_t last_command_time_us;
        uint32_t last_encoder_read_ms;
        uint32_t last_speed_log_ms;
        uint32_t last_ack_log_ms;
        bool enabled;
        SpeedEstimator speedEstimator;
    };

    MotorState m_left;
    MotorState m_right;
    bool m_enabled;
    SemaphoreHandle_t m_leftBusMutex;
    SemaphoreHandle_t m_rightBusMutex;

    // Async control state for background tasks
    struct AsyncState {
        std::atomic<float> target_speed{0.0f};
        std::atomic<bool> speed_dirty{false};
        std::atomic<int32_t> encoder_value{0};
        std::atomic<float> speed_value{0.0f};
        std::atomic<bool> encoder_dirty{false};
        std::atomic<uint32_t> last_latency_us{0};
        std::atomic<bool> last_reported_enabled{true}; // Init to true to force torque sync at boot

        // Time when a speed frame was last written to the motor (microseconds).
        std::atomic<uint64_t> last_cmd_send_time_us{0};
        // Time when telemetry (encoder) was last successfully read (microseconds).
        std::atomic<uint64_t> last_encoder_time_us{0};

        // Lightweight diagnostics counters
        std::atomic<uint32_t> speed_cmd_sent{0};
        std::atomic<uint32_t> speed_ack_timeout{0};
        std::atomic<uint32_t> speed_ack_error{0};
        std::atomic<uint32_t> encoder_ok{0};
        std::atomic<uint32_t> encoder_timeout{0};

        // Per-task safety zeroing timer (milliseconds). Only accessed by the
        // corresponding motor task, so no atomic needed.
        uint32_t last_zero_ms = 0;

        TaskHandle_t task_handle = nullptr;
        uint32_t last_telemetry_ms = 0;
    };

    AsyncState m_left_async;
    AsyncState m_right_async;

    // When enabled, each speed write waits for an ACK (adds latency).
    std::atomic<bool> m_wait_for_ack{false};

    // Acceleration byte (0-255) included in MKS 0xF6 speed frames.
    // Default comes from config (MKS_SERVO_ACCEL).
    std::atomic<uint8_t> m_speed_accel{(uint8_t)MKS_SERVO_ACCEL};

    static void motorTaskEntry(void* pvParameters);
    void runMotorTask(MotorSide side);
    // Protocol helpers
    void sendSpeedCommand(uint8_t id, float normalized_speed, bool invert, bool wait_for_ack = false);
    void sendFunctionCommand(uint8_t id, uint8_t function_code, const uint8_t* data, size_t length);
    void setMode(uint8_t id, MksServoMode mode);
    void setMStep(uint8_t id, MksServoMicrostep mstep);
    void setCurrent(uint8_t id, uint16_t current_ma);
    void setHoldCurrent(uint8_t id, MksServoHoldCurrent hold_pct);
    void setEnable(uint8_t id, bool enable);
    bool verifyConfig(uint8_t id, uint8_t function_code, uint8_t expected_value, const char* label);
    
    // Telemetry
    uint32_t last_bus_latency_us_ = 0;

    uint8_t calculateChecksum(const uint8_t* data, size_t length);
    void writeFrame(HardwareSerial& serial, const uint8_t* frame, size_t length);
    bool readResponse(HardwareSerial& serial, uint8_t id, uint8_t function_code, uint8_t* out_data, size_t expected_length, uint32_t timeout_us = 2000);
    
    /**
     * @brief Maps a motor ID to its dedicated HardwareSerial port.
     * @param id The motor ID.
     * @return HardwareSerial& reference to Serial1 or Serial2.
     */
    HardwareSerial& getSerialForMotor(uint8_t id);

    /**
     * @brief Gets the mutex associated with a specific motor's bus.
     * @param id The motor ID.
     * @return SemaphoreHandle_t for the specific motor.
     */
    SemaphoreHandle_t getMutexForMotor(uint8_t id);
    
    void dumpMotorRegisters(uint8_t id);
};

} // namespace motor
} // namespace abbot
