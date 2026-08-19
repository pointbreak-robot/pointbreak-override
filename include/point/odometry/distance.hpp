#ifndef DISTANCE_HPP
#define DISTANCE_HPP

#pragma once

#include "point/odometry/pose.hpp"
#include "pros/distance.hpp"
#include "pros/imu.hpp"
#include "pros/rtos.hpp"
#include <cmath>

namespace point {

class DistOdom {
public:
    DistOdom(int port, float headingOffset, float xOffset, float yOffset);

    Pose update(Pose currentPose);
    bool isValid(Pose currentPose);
    void setEnabled(bool enabled);
    bool isEnabled() const;
    bool isConnected();

private:
    pros::Distance  sensor_;
    bool            enabled_     = false;
    float           headingOffset_;
    float           xOffset_;
    float           yOffset_;

    uint32_t        prevTime_    = pros::millis();
    float           prevReading_ = infinityf();
    Pose            prevPose_    = {infinityf(), infinityf(), infinityf()};
};

} // namespace point

#endif