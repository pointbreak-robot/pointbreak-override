#ifndef TRACKINGWHEEL_HPP
#define TRACKINGWHEEL_HPP

#pragma once

#include "pros/rotation.hpp"

namespace point {

enum class LengthUnit {
    INCHES,
    MILLIMETERS,
    CENTIMETERS
};

class TrackingWheel {
public:
    TrackingWheel(int port, double wheelDiameter, double offset,
                  double gearRatio = 1.0, bool reversed = false,
                  LengthUnit unit = LengthUnit::INCHES);

    void reset();

    double getDistance() const;
    double getDeltaDistance();
    double getVelocity() const;

    double     getWheelDiameter() const;
    double     getOffset()        const;
    double     getGearRatio()     const;
    bool       isReversed()       const;
    LengthUnit getUnit()          const;
    bool       isConnected()      ;

private:
    pros::Rotation sensor_;

    double     wheelDiameter_;
    double     offset_;
    double     gearRatio_;
    bool       reversed_;
    LengthUnit unit_;

    mutable double prevDistance_;

    double getRawDegrees()              const;
    double ticksToDistance(double deg)  const;
};

} // namespace point

#endif