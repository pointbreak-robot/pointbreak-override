#ifndef CONFIG_HPP
#define CONFIG_HPP

#include "api.h"
#include "setup/autonomous.hpp"
#include "point/chassis/chassis.hpp"

using namespace point;

// controller
extern pros::Controller master;

// motors
extern pros::MotorGroup leftMotors;
extern pros::MotorGroup rightMotors;

// drive and match type
extern driveType drivetype;
extern matchType currentMatch;
extern volatile bool robotEnabled;
extern volatile bool autonTestRunning;
extern pros::Mutex lcdMutex;

// functions
void auton_selector();
bool opTog();
void buttonInputs();
void selDisp();
void opDisp();
void config();
void compConfig();
bool devOk();
void updateDisplay(std::string name = "");

#endif