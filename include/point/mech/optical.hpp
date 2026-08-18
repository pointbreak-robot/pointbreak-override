#pragma once

#include "pros/optical.hpp"

namespace point {

class OpticalSensor {
public:
    OpticalSensor(int port, int led_pwm = 100);

    void init();

    double getHue();
    double getBrightness();
    double getSaturation();
    int getProximity();
    bool isDetected(int threshold = 150);

    // Color detection
    enum class Color : uint8_t { RED, BLUE, YELLOW, UNKNOWN };
    Color getColor();
    const char* getColorName();

    double getAvgHue(int sample = 5, int delay_ms = 10);

private:
    pros::Optical sensor_;
    int led_pwm_;
};

} // namespace point