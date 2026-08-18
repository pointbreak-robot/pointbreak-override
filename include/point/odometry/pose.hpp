#pragma once

#include <cmath>

namespace point {

struct Pose {
    float x = 0.0f;
    float y = 0.0f;
    float theta = 0.0f;

    Pose (float x = 0.0f, float y = 0.0f, float theta = 0.0f);

    Pose operator+(const Pose& other) const;
    Pose operator-(const Pose& other) const;
    Pose operator*(float scalar) const;
    Pose operator/(float scalar) const;
    bool operator==(const Pose& other) const;

    float distanceTo(const Pose& other) const;
    float angleTo(const Pose& other) const;
};

} // namespace point