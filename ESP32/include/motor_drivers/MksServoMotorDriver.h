// MksServoMotorDriver.h - Driver for MKS SERVO42D/57D serial bus motors
#pragma once

#include "AbstractMotorDriver.h"
#include "../../config/motor_configs/mks_servo_config.h"
#include "speed_estimator.h"
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

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
    const char *getDriverName() const override { return "mks_servo"; }
    uint32_t getLastBusLatencyUs() const override { return last_bus_latency_us_; }

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
    SemaphoreHandle_t m_busMutex;

    // Protocol helpers
    void sendSpeedCommand(uint8_t id, float normalized_speed, bool invert);
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
    
    void dumpMotorRegisters(uint8_t id);
};

} // namespace motor
} // namespace abbot
