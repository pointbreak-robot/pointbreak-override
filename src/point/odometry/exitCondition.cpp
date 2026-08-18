#include "main.h"
#include "pros/rtos.hpp"

using namespace point;

exitCondition::exitCondition(const float range_, const int time_) : range(range_), time(time_) {}

bool exitCondition::update(const float input) {
    if (done) return true;

    if (std::abs(input) <= range) {
        if (startTime == -1) startTime = pros::millis();
        if ((int)(pros::millis() - startTime ) >= time) done = true;
    } else {
        startTime = -1;
    }

    return done;
}

bool exitCondition::getExit() {
    return done;
}

void exitCondition::reset() {
    startTime = -1;
    done = false;
}