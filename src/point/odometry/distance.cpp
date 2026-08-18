#include "main.h"
#include <cmath>

using namespace point;

DistOdom::DistOdom(int port, float headingOffset,
                    float xOffset, float yOffset)
    : sensor_(port),
      headingOffset_(headingOffset),
      xOffset_(xOffset),
      yOffset_(yOffset) {}

bool DistOdom::isValid(Pose currentPose) {
    if (!enabled_) return false;

    float reading = sensor_.get() / 25.4f;

    if (reading <= 0 || reading >= 100.0f) return false;

    float dPose = currentPose.distanceTo(prevPose_);
    float dReading = std::abs(reading - prevReading_);
    uint32_t dTime = pros::millis() - prevTime_;

    if (dTime < 20) return false;
    if (dReading > 5.0f && dPose < 0.5f) return false;

    return true;
}

Pose DistOdom::update(Pose currentPose) {
    if (!isValid(currentPose)) return currentPose;

    float reading  = sensor_.get() / 25.4f;
    float absAngle = currentPose.theta + headingOffset_;
    float absRad   = absAngle * (3.14159265f / 180.0f);

    float sensorX = currentPose.x + xOffset_ * std::cos(absRad) - yOffset_ * std::sin(absRad);
    float sensorY = currentPose.y + xOffset_ * std::sin(absRad) + yOffset_ * std::cos(absRad);

    float wallX = sensorX + reading * std::cos(absRad);
    float wallY = sensorY + reading * std::sin(absRad);

    Pose corrected = currentPose;

    if (std::abs(std::cos(absRad)) > std::abs(std::sin(absRad))) {
        corrected.x = wallX - reading * std::cos(absRad) - xOffset_;
    } else {
        corrected.y = wallY - reading * std::sin(absRad) - yOffset_;
    }

    prevReading_ = reading;
    prevPose_    = currentPose;
    prevTime_    = pros::millis();

    return corrected;
}

void DistOdom::setEnabled(bool enabled) {
    enabled_ = enabled;
}

bool DistOdom::isEnabled() const {
    return enabled_;
}

bool DistOdom::isConnected() {
    return sensor_.is_installed();
}