#ifndef PNEUMATICS_HPP
#define PNEUMATICS_HPP

#pragma once

#include "main.h"
#include "pros/adi.hpp"

namespace point {

class Pneumatics {
public:
    explicit Pneumatics(char port, bool startExtended = false, bool extendedIsLow = false);

    void extend();
    void retract();
    void toggle();
    bool isExtended() const;

private:
    pros::adi::Pneumatics piston_;
};

} // namespace



#endif