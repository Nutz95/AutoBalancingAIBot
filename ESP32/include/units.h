#pragma once

// units.h
// Small utilities for unit conversions used across the firmware.

#ifndef UNITS_H
#define UNITS_H

#include <cmath>

// Internal constants (avoid macro name collisions with Arduino)
constexpr float RAD_TO_DEG_F = 180.0f / 3.14159265358979323846f;
constexpr float DEG_TO_RAD_F = 3.14159265358979323846f / 180.0f;

// Inline conversion helpers - prefer these over magic literals so intent
// is explicit and the function name documents the conversion.
inline constexpr float radToDeg(float r) { return r * RAD_TO_DEG_F; }
inline constexpr float degToRad(float d) { return d * DEG_TO_RAD_F; }

#endif // UNITS_H
