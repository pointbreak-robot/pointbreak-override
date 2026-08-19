#include "main.h"
#include "pros/motors.h"
#include "setup/autonomous.hpp"
#include "point/chassis/chassis.hpp"
#include "point/odometry/imu.hpp"

using namespace point;


pros::Controller master(pros::E_CONTROLLER_MASTER); // controller

pros::MotorGroup leftMotors({-9, 6, -8}); // left motors
pros::MotorGroup rightMotors({-5, 1, -7}); // right motors

// --- State Variables ---
driveType drivetype    = SPLIT;
matchType currentMatch = QUALS;
volatile bool robotEnabled = false;
volatile bool autonTestRunning = false;
pros::Task*   autonTestTask    = nullptr;

pros::Mutex lcdMutex;

// --- Display Functions ---

std::string getAutonName(autonID id) {
    switch(id) {
        case none:    return "No Auton      ";
        case awp:     return "AWP           ";
        case red1:    return "Red 1 Neutral ";
        case red2:    return "Red 2 Goal    ";
        case blue1:   return "Blue 1 Neutral";
        case blue2:   return "Blue 2 Goal   ";
        case skills0: return "Skills        ";
        default:      return "Unknown       ";
    }
}

std::string getMatchTypeName(matchType type) {
    switch(type) {
        case QUALS: return "Quals";
        case ELIMS: return "Elims";
        default:    return "Unknown";
    }
}

void updateDisplay(std::string name) {
    lcdMutex.take();
    pros::lcd::set_text(0, " -- POINT BREAK ROBOTICS v1.0.0 -- ");
    pros::lcd::set_text(1, "Match Type: " + getMatchTypeName(currentMatch));
    pros::lcd::set_text(2, "Select Auton: " + name);
    pros::lcd::set_text(3, "                    ");
    lcdMutex.give();
}

void selDisp() {
    updateDisplay(getAutonName(selectedAuton));
    devOk();
}

void opDisp() {
    lcdMutex.take();
    pros::lcd::set_text(0, " -- POINT BREAK ROBOTICS v1.0.0 --");
    pros::lcd::set_text(1, "                    ");
    pros::lcd::set_text(2, "x: ");
    pros::lcd::set_text(3, "y: ");
    pros::lcd::set_text(4, "Rotation Deg: ");
    pros::lcd::set_text(5, "                    ");
    pros::lcd::set_text(6, "                    ");
    pros::lcd::set_text(7, "                    ");
    lcdMutex.give();
}

bool devOk() {
    auto isMotorInstalled = [](int port) {
        int absPort = port < 0 ? -port : port;
        return pros::Motor(absPort).is_installed();
    };

    bool left1Online = isMotorInstalled(9);
    bool left2Online = isMotorInstalled(-6);
    bool left3Online = isMotorInstalled(8);
    bool right1Online = isMotorInstalled(5);
    bool right2Online = isMotorInstalled(-1);
    bool right3Online = isMotorInstalled(7);

    bool imuOnline = imu.isConnected();
    bool trackingWheelOnline = trackingWheel.isConnected();
    bool distOnline = distSensor.isConnected();

    bool allOnline = left1Online && left2Online && left3Online &&
                     right1Online && right2Online && right3Online &&
                     imuOnline && trackingWheelOnline && distOnline;

    lcdMutex.take();
    pros::lcd::set_text(4, std::string("M1:") + (left1Online ? "OK " : "OFF") +
                           " M2:" + (left2Online ? "OK " : "OFF") +
                           " M3:" + (left3Online ? "OK" : "OFF"));
    pros::lcd::set_text(5, std::string("M4:") + (right1Online ? "OK " : "OFF") +
                           " M5:" + (right2Online ? "OK " : "OFF") +
                           " M6:" + (right3Online ? "OK" : "OFF"));
    pros::lcd::set_text(6, std::string("IMU:") + (imuOnline ? "OK" : "OFF") +
                           " TW:" + (trackingWheelOnline ? "OK" : "OFF") +
                           " D:" + (distOnline ? "OK" : "OFF"));
    pros::lcd::set_text(7, allOnline ? "Device Check: ALL ONLINE" : "Device Check: OFFLINE");
    lcdMutex.give();

    return allOnline;
}

void runAutonTest() {
    robotEnabled = true;
    autonomousSelector(selectedAuton);
    robotEnabled = false;
    autonTestRunning = false;

    updateDisplay(getAutonName(selectedAuton));
    devOk();
}

// auton selector task

void auton_selector() {
    master.rumble("."); // confirms the task actually started

    bool confirmed = false;
    bool testingMode = !pros::competition::is_connected();
    updateDisplay(getAutonName(selectedAuton));
    devOk();

    bool     upWasHeld   = false;
    uint32_t upHoldStart = 0;
    bool     upTriggered = false;
    const uint32_t upHoldMs = 800;

    while (true) {
        // hold UP to start a test run, hold UP again to abort and return here
        if (testingMode) {
            bool upHeldNow = master.get_digital(pros::E_CONTROLLER_DIGITAL_UP);

            if (upHeldNow && !upWasHeld) {
                upHoldStart = pros::millis();
                upTriggered = false;
            } else if (!upHeldNow) {
                upHoldStart = 0;
                upTriggered = false;
            }

            if (upHeldNow && !upTriggered && upHoldStart > 0 &&
                (pros::millis() - upHoldStart) >= upHoldMs) {
                upTriggered = true;

                if (!autonTestRunning) {
                    // start the test
                    master.rumble("...");
                    autonTestRunning = true;

                    lcdMutex.take();
                    pros::lcd::set_text(3, "TESTING AUTON: " + getAutonName(selectedAuton));
                    lcdMutex.give();

                    autonTestTask = new pros::Task(runAutonTest, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "auton_test");
                } else {
                    // second hold - abort and return to selector
                    master.rumble("..");
                    chassis.stop(); // halt motors, clear queued movements

                    if (autonTestTask) {
                        autonTestTask->remove();
                        delete autonTestTask;
                        autonTestTask = nullptr;
                    }

                    robotEnabled = false;
                    autonTestRunning = false;

                    updateDisplay(getAutonName(selectedAuton));
                    devOk();
                }
            }

            upWasHeld = upHeldNow;
        }

        // while a test is running, don't process any other selector input
        if (autonTestRunning || robotEnabled) {
            pros::delay(50);
            continue;
        }

        bool l1Pressed = master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1);
        bool r1Pressed = master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R1);
        bool aPressed  = master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A);
        bool bPressed  = master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B);
        bool xPressed  = master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X);

        if (!confirmed) {
            if (l1Pressed) {
                if (selectedAuton > none) {
                    selectedAuton = (autonID)(selectedAuton - 1);
                } else {
                    selectedAuton = skills0;
                }
                updateDisplay(getAutonName(selectedAuton));
            }

            if (r1Pressed) {
                if (selectedAuton < skills0) {
                    selectedAuton = (autonID)(selectedAuton + 1);
                } else {
                    selectedAuton = none;
                }
                updateDisplay(getAutonName(selectedAuton));
            }

            if (aPressed) {
                updateDisplay(getAutonName(selectedAuton));
                lcdMutex.take();
                pros::lcd::set_text(3, "Confirmed! ALL SYSTEMS GO!");
                lcdMutex.give();
                master.rumble(".");
                confirmed = true;
            }
        }

        if (confirmed && bPressed) {
            confirmed = false;
            master.rumble("..");
            updateDisplay(getAutonName(selectedAuton));
            lcdMutex.take();
            pros::lcd::set_text(3, "                    ");
            lcdMutex.give();
        }

        if (xPressed) {
            currentMatch = static_cast<matchType>((currentMatch + 1) % 2);
            updateDisplay(getAutonName(selectedAuton));
            devOk();
        }

        pros::delay(50);
    }
}

// opcontrol toggle
bool opTog() {
    static bool     yWasHeld = false;
    static uint32_t yHoldStart = 0;
    static bool     triggered = false;
    const uint32_t  yHoldMs = 800;

    bool yHeldNow = master.get_digital(pros::E_CONTROLLER_DIGITAL_Y);

    if (yHeldNow && !yWasHeld) {
        yHoldStart = pros::millis();
        triggered = false;
    } else if (!yHeldNow) {
        yHoldStart = 0;
        triggered = false;
    }

    if (yHeldNow && !triggered && yHoldStart > 0 && (pros::millis() - yHoldStart) >= yHoldMs) {
        robotEnabled = !robotEnabled;
        triggered = true;

        if (robotEnabled) {
            master.rumble(".");
        } else {
            chassis.stop();
            master.rumble("..");
        }
    }

    yWasHeld = yHeldNow;
    return robotEnabled;
}

// button inputs (driver control)

void buttonInputs() {

}

// robot config

void config() {
    robotEnabled = false;

    chassis.setMotors(&leftMotors, &rightMotors);

    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);

    updateDisplay(getAutonName(selectedAuton));
    devOk();

    odom.reset({0, 0, 0});
 
}

void compConfig() {
    odom.reset({0, 0, 0});

    robotEnabled = false;

    chassis.setMotors(&leftMotors, &rightMotors);

    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);

    updateDisplay(getAutonName(selectedAuton));
    devOk();
}