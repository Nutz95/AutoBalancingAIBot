// driver_manager_tests.cpp
// Simple unit test for driver_manager set/get behavior.

#include "../include/motor_drivers/driver_manager.h"
#include <iostream>

using namespace abbot::motor;

static int assert_true(bool v, const char *msg) {
    if (v) return 0;
    std::cerr << "Assertion failed: " << msg << "\n";
    return 1;
}

class FakeDriver : public IMotorDriver {
public:
  FakeDriver() { last_left = 0.0f; last_right = 0.0f; enabled = false; }
  virtual ~FakeDriver() {}
  void initMotorDriver() override {}
  void clearCommandState() override { last_left = last_right = 0.0f; }
  float getLastMotorCommand(int id) override {
    return (id == 0) ? last_left : last_right;
  }
  void enableMotors() override { enabled = true; }
  void disableMotors() override { enabled = false; }
  bool areMotorsEnabled() override { return enabled; }
  void printStatus() override {}
  void dumpConfig() override {}
  void setMotorCommandBoth(float left_command, float right_command) override {
    last_left = left_command; last_right = right_command;
  }
  void setMotorCommand(int id, float command) override {
    if (id == 0) last_left = command; else last_right = command;
  }
  void setMotorCommandRaw(int id, int16_t rawSpeed) override { (void)id; (void)rawSpeed; }
  int32_t readEncoder(int id) override { (void)id; return 0; }
  void resetPositionTracking() override {}
  bool processSerialCommand(const String &line) override { (void)line; return false; }

  float last_left, last_right;
  bool enabled;
};

int main() {
    int failures = 0;

    // Initially should be null
    failures += assert_true(getActiveMotorDriver() == nullptr, "initial active driver is nullptr");

    FakeDriver drv;
    setActiveMotorDriver(&drv);
    failures += assert_true(getActiveMotorDriver() == &drv, "active driver set/get mismatch");

    // Exercise some calls via the active driver
    IMotorDriver *active = getActiveMotorDriver();
    active->clearCommandState();
    active->setMotorCommandBoth(0.12f, -0.34f);
    failures += assert_true(std::abs(active->getLastMotorCommand(0) - 0.12f) < 1e-6, "left command stored");
    failures += assert_true(std::abs(active->getLastMotorCommand(1) - -0.34f) < 1e-6, "right command stored");

    active->enableMotors();
    failures += assert_true(active->areMotorsEnabled(), "motors enabled");
    active->disableMotors();
    failures += assert_true(!active->areMotorsEnabled(), "motors disabled");

    // Clear active
    setActiveMotorDriver(nullptr);
    failures += assert_true(getActiveMotorDriver() == nullptr, "active driver cleared");

    if (failures == 0) std::cout << "driver_manager tests: PASS\n";
    else std::cerr << "driver_manager tests: FAIL (" << failures << " failures)\n";
    return failures;
}
