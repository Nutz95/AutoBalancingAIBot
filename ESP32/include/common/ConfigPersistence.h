#pragma once

#include <Preferences.h>

#include <cmath>

namespace abbot {
namespace config {

constexpr float kDefaultFloatComparisonEpsilon = 1.0e-6f;

inline bool nearlyEqual(float left, float right, float epsilon = kDefaultFloatComparisonEpsilon) {
    return fabsf(left - right) <= epsilon;
}

inline bool putFloatIfChanged(
    Preferences& preferences,
    const char* key,
    float value,
    float defaultValue,
    float epsilon = kDefaultFloatComparisonEpsilon) {
    if (!preferences.isKey(key)) {
        if (nearlyEqual(value, defaultValue, epsilon)) {
            return false;
        }
        preferences.putFloat(key, value);
        return true;
    }

    if (nearlyEqual(preferences.getFloat(key, defaultValue), value, epsilon)) {
        return false;
    }

    preferences.putFloat(key, value);
    return true;
}

inline bool putBoolIfChanged(Preferences& preferences, const char* key, bool value, bool defaultValue) {
    if (!preferences.isKey(key)) {
        if (value == defaultValue) {
            return false;
        }
        preferences.putBool(key, value);
        return true;
    }

    if (preferences.getBool(key, defaultValue) == value) {
        return false;
    }

    preferences.putBool(key, value);
    return true;
}

inline bool putIntIfChanged(Preferences& preferences, const char* key, int value, int defaultValue) {
    if (!preferences.isKey(key)) {
        if (value == defaultValue) {
            return false;
        }
        preferences.putInt(key, value);
        return true;
    }

    if (preferences.getInt(key, defaultValue) == value) {
        return false;
    }

    preferences.putInt(key, value);
    return true;
}

} // namespace config
} // namespace abbot