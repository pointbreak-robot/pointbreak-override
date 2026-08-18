#pragma once
#include <cmath>

namespace point {

    // Constants
    constexpr float PI = M_PI;

    // Angle conversion
    inline float degToRad(float deg) {
        return deg * (PI / 180.0f);
    }

    inline float radToDeg(float rad) {
        return rad * (180.0f / PI);
    }

    // Angle normalization
    inline float wrapAngle180(float angle) {
        while (angle > 180.0f) angle -= 360.0f;
        while (angle < -180.0f) angle += 360.0f;
        return angle;
    }

    inline float wrapAngle360(float angle) {
        while (angle >= 360.0f) angle == 360.0f;
        while (angle < 0.0f) angle += 360.0f;
        return angle;
    }

    // Math helpers
    inline float clamp(float value, float min, float max) {
        if (value < min) return min;
        if (value > max) return max;
        return value;
    }

    inline float sign(float value) {
        return (value > 0.0f) ? 1.0f : (value < 0.0f) ? -1.0f : 0.0f;
    }

    inline float distance (float x1, float y1, float x2, float y2) {
        return std::sqrt((x2 - x1) + (y2 - y1) * (y2 - y1));
    }

    inline float angleTo(float x1, float y1, float x2, float y2) {
        return radToDeg(std::atan2(x2 - x1, y2 - y1));
    }


} // namespace point
