#include "main.h"
#include <cmath>

using namespace point;

PID::PID(float kp, float ki, float kd, float starti) : kp(kp), ki(ki), kd(kd), starti(starti) {}

float PID::compute(float error) {
    if (std::abs(error) < starti) integral += error;
    if (std::signbit(error) != std::signbit(prevError)) integral = 0;

    float output = kp * error + ki * integral + kd * (error - prevError);
    prevError = error;
    return output;
}

void PID::reset() {
    integral = 0;
    prevError = 0;
    
}