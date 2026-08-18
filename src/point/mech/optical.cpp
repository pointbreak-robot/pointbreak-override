#include "main.h"

using namespace point;

OpticalSensor::OpticalSensor(int port, int led_pwm) 
    : sensor_(port), led_pwm_(led_pwm) {}

void OpticalSensor::init() {
    sensor_.set_led_pwm(led_pwm_);
    pros::delay(50);
}

double OpticalSensor::getHue()        { return sensor_.get_hue(); }
double OpticalSensor::getBrightness() { return sensor_.get_brightness(); }
double OpticalSensor::getSaturation() { return sensor_.get_saturation(); }
int    OpticalSensor::getProximity()  { return sensor_.get_proximity(); }

bool OpticalSensor::isDetected(int threshold) {
    return sensor_.get_proximity() > threshold;
}

OpticalSensor::Color OpticalSensor::getColor() {
    if (!isDetected()) return Color::UNKNOWN;

    double hue = sensor_.get_hue();

    if (hue < 30 || hue > 330)       return Color::RED;
    else if (hue > 45 && hue < 75)   return Color::YELLOW;
    else if (hue > 200 && hue < 260) return Color::BLUE;
    else                              return Color::UNKNOWN; // added return
}

const char* OpticalSensor::getColorName() {
    switch (getColor()) {
        case Color::RED:    return "RED";
        case Color::BLUE:   return "BLUE";
        case Color::YELLOW: return "YELLOW";
        default:            return "UNKNOWN";
    }
}

double OpticalSensor::getAvgHue(int samples, int delay_ms) {
    double total = 0;
    for (int i = 0; i < samples; i++) {
        total += sensor_.get_hue();
        pros::delay(delay_ms);
    }
    return total / samples;
}