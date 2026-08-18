#include "main.h"
#include "pros/rtos.hpp"
#include <cmath>

using namespace point;

IMU::IMU(int port, bool reversed) : sensor_(port), reversed_(reversed), offset_(0.0) {}

bool IMU::calibrate(int timeoutMs) {
    sensor_.reset();
    int elapsed = 0;
    while (sensor_.is_calibrating()) {
        pros::delay(10);
        elapsed += 10;
        if (elapsed >= timeoutMs) return false;
    }
    return true;

}

bool IMU::isCalibrated() {
    return !sensor_.is_calibrating();
}

void IMU::tare() {
    offset_ = sensor_.get_heading();
}

void IMU::setOffset(double degrees) {
    offset_ = degrees;
}

double IMU::getHeading() const {
    double heading = sensor_.get_heading() - offset_;
    if (reversed_) heading = -heading;
    return wrapAngle(heading);
}

double IMU::getRotation() const {
    double rotation = sensor_.get_rotation();
    if (reversed_) rotation = -rotation;
    return rotation;
}

double IMU::getAngularVelocity() const {
    double vel = sensor_.get_gyro_rate().z;
    if (reversed_) vel = -vel;
    return vel;
}

int IMU::getPort() const {
    return sensor_.get_port();
}

bool IMU::isReversed() const {
    return reversed_;
}

bool IMU::isConnected() {
    return sensor_.is_installed();
}


double IMU::wrapAngle(double deg) {
    deg = std::fmod(deg, 360.0);
    if (deg < 0.0) deg += 360.0;
    return deg;
}