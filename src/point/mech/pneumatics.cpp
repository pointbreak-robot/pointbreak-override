#include "main.h"

using namespace point;

Pneumatics::Pneumatics(char port, bool startExtended, bool extendedIsLow) : piston_(static_cast<std::uint8_t>(port), startExtended, extendedIsLow) {}

void Pneumatics::extend() {
    piston_.extend();
}

void Pneumatics::retract() {
    piston_.retract();
}

void Pneumatics::toggle() {
    piston_.toggle();
}

bool Pneumatics::isExtended() const {
    return piston_.is_extended();
}