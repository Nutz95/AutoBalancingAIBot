// dc_stepper_interface.h
// Placeholder interface for DC and Stepper drivers.
// This file provides an empty subclass/marker for future DC/stepper-specific
// APIs. It intentionally does not add methods yet; extend it later with
// hardware-specific helpers (encoder config, step generation, PWM channels).
#pragma once
#include "IMotorDriver.h"

namespace abbot {
namespace motor {

// Marker/extension point for DC & stepper drivers. Implementations may
// derive from IMotorDriver directly or from this subclass if DC/stepper
// specific helpers are later added.
class IDCStepperDriver : public IMotorDriver {
public:
  virtual ~IDCStepperDriver() {}
  // No extra methods yet — add hardware-specific APIs here later.
};

} // namespace motor
} // namespace abbot
