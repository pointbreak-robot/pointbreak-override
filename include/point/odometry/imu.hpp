#ifndef IMU_HPP
#define IMU_HPP

#pragma once

#include "pros/imu.hpp"

namespace point {

class IMU {
public:
    explicit IMU (int port, bool reverse = false);

    bool calibrate(int timeoutMS = 3000);
    bool isCalibrated();

    void tare();
    void setOffset(double degrees);

    double getHeading() const;
    double getRotation() const;
    double getAngularVelocity() const;

    int getPort() const;
    bool isReversed() const;
    bool isConnected();

private:
    pros::Imu sensor_;
    bool reversed_;
    double offset_;

    static double wrapAngle(double deg);
};

} // namespace point

#endif