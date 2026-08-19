#ifndef CONSTANTS_HPP
#define CONSTANTS_HPP

#pragma once

#include "point/odometry/PID.hpp"
#include "point/odometry/exitCondition.hpp"
#include "point/odometry/imu.hpp"
#include "point/odometry/trackingWheel.hpp"
#include "point/odometry/distance.hpp"
#include "point/odometry/odom.hpp"
#include "point/chassis/chassis.hpp"

namespace point {

    // PIDs
    inline PID drivePID(0.0f, 0.0f, 0.0f, 0.0f);
    inline PID turnPID(0.0f, 0.0f, 0.0f, 0.0f);

    // exit conditions
    inline exitCondition lateralExit(0.5f, 300);
    inline exitCondition angularExit(1.0f, 300);

    // sensors
    inline IMU imu(21);
    inline TrackingWheel trackingWheel(7, 2.75, 0.0);
    inline DistOdom distSensor(9, 0.0f, 0.0f, 0.0f); // (port, headingOffset, xOffset, yOffset)

    // odom
    inline OdomConfig odomConfig = {
        .trackingWheel = &trackingWheel,
        .imu            = &imu,
        .distanceSensor = &distSensor
    };
    inline Odometry odom(odomConfig);

    inline Chassis chassis(
        {nullptr, nullptr, SPLIT},  // motors added in config()
        odom,
        drivePID,
        turnPID,
        lateralExit,
        angularExit
    );

} // namespace point

#endif 