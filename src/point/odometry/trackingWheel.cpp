#include "main.h"
#include <cmath>

using namespace point;

TrackingWheel::TrackingWheel(int port, double wheelDiameter, double offset,
                             double gearRatio, bool reversed, LengthUnit unit)
    : sensor_(port),
      wheelDiameter_(wheelDiameter),
      offset_(offset),
      gearRatio_(gearRatio),
      reversed_(reversed),
      unit_(unit),
      prevDistance_(0.0) {
    sensor_.set_reversed(reversed);
}

void TrackingWheel::reset() {
    sensor_.reset_position();
    prevDistance_ = 0.0;
}

double TrackingWheel::getDistance() const {
    return ticksToDistance(getRawDegrees());
}

double TrackingWheel::getDeltaDistance() {
    double current = getDistance();
    double delta   = current - prevDistance_;
    prevDistance_  = current;
    return delta;
}

double TrackingWheel::getVelocity() const {
    return sensor_.get_velocity() / 100.0;
}

double TrackingWheel::getWheelDiameter() const { return wheelDiameter_; }
double TrackingWheel::getOffset()         const { return offset_; }
double TrackingWheel::getGearRatio()      const { return gearRatio_; }
bool   TrackingWheel::isReversed()        const { return reversed_; }
LengthUnit TrackingWheel::getUnit()       const { return unit_; }

bool TrackingWheel::isConnected() {
    return sensor_.is_installed();
}

double TrackingWheel::getRawDegrees() const {
    return sensor_.get_position() / 100.0;
}

double TrackingWheel::ticksToDistance(double degrees) const {
    double circumference = wheelDiameter_ * 3.14159265;
    double distance = (degrees / 360.0) * circumference * gearRatio_;

    switch (unit_) {
        case LengthUnit::MILLIMETERS: return distance * 25.4;
        case LengthUnit::CENTIMETERS: return distance * 2.54;
        default:                      return distance;
    }
}

