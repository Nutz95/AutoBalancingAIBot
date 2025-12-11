// motor_config.h - compatibility shim for older includes
// This file re-exports the new per-driver config header so legacy
// `#include "../config/motor_config.h"` references continue to work.
#pragma once

#include "motor_configs/servo_motor_config.h"

// NOTE: If you moved or renamed more motor configuration headers,
// update this shim to re-export them or include the appropriate files.
// Compatibility shim: prefer `config/motor_configs/servo_motor_config.h`
#pragma once
#include "motor_configs/servo_motor_config.h"

// Note: `motor_config.h` is kept as a thin compatibility header so existing
// includes that expect `motor_config.h` continue to work. Prefer
// `config/motor_configs/servo_motor_config.h` for editing new values.
