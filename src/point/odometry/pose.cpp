#include "main.h"
#include <cmath>

using namespace point;

Pose::Pose(float x, float y, float theta)
    : x(x), y(y), theta(theta) {}

Pose Pose::operator+(const Pose& other) const {
    return {x + other.x, y + other.y, theta + other.theta};
}

Pose Pose::operator-(const Pose& other) const {
    return {x - other.x, y - other.y, theta - other.theta};
}

Pose Pose::operator*(float scalar) const {
    return {x * scalar, y * scalar, theta * scalar};
}

Pose Pose::operator/(float scalar) const {
    return {x / scalar, y / scalar, theta / scalar};
}

bool Pose::operator==(const Pose& other) const {
    return x == other.x && y == other.y && theta == other.theta;
}

float Pose::distanceTo(const Pose& other) const {
    float dx = other.x - x;
    float dy = other.y - y;
    return std::sqrt(dx * dx + dy * dy);
}

float Pose::angleTo(const Pose& other) const {
    float dx = other.x - x;
    float dy = other.y - y;
    return std::atan2(dy, dx) * (180.0f / 3.14159265f);
}