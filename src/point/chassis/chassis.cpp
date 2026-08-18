#include "main.h"
#include "point/chassis/chassis.hpp"

using namespace point;

// Constructors

Chassis::Chassis(ChassisConfig chassisConfig, Odometry& odometry,
                 PID& lateralPID, PID& angularPID,
                 exitCondition& lateralExit, exitCondition& angularExit)
    : leftMotors_(chassisConfig.leftMotors),
      rightMotors_(chassisConfig.rightMotors),
      driveType_(chassisConfig.drivetype_),
      odometry_(odometry),
      lateralPID_(lateralPID),
      angularPID_(angularPID),
      lateralExit_(lateralExit),
      angularExit_(angularExit) {}

void Chassis::setMotors(pros::MotorGroup* left, pros::MotorGroup* right) {
    leftMotors_  = left;
    rightMotors_ = right;
}

// Queue

Chassis &Chassis::queue(movementType type, Movement movement) {
    movement.type = type;
    return queue(movement);
}

Chassis &Chassis::queue(Movement movement) {
    if (movement.exitRange > 0 && movement.voltMin == 0)
        movement.voltMin = 10;
    movementQueue_.push(movement);
    return *this;
}

// Movement commands

Chassis &Chassis::moveDistance(float distance, Movement movement) {
    Pose pose = odometry_.getPose();
    return moveDistance(distance, movement, pose.x, pose.y);
}

Chassis &Chassis::moveDistance(float distance, Movement movement, float startX, float startY) {
    return moveDistance(distance, movement, startX, startY, odometry_.getPose().theta);
}

Chassis &Chassis::moveDistance(float distance, Movement movement, float startX, float startY, float startTheta) {
    movement.type     = MOVE_TO_POINT;
    movement.distance = distance;
    movement.x        = startX + distance * std::sin(degToRad(startTheta));
    movement.y        = startY + distance * std::cos(degToRad(startTheta));
    return queue(movement);
}

Chassis &Chassis::moveToPoint(float x, float y, Movement movement) {
    movement.type = MOVE_TO_POINT;
    movement.x    = x;
    movement.y    = y;
    return queue(movement);
}

Chassis &Chassis::turnToPoint(float x, float y, Movement movement) {
    movement.type = TURN_TO_POINT;
    movement.x    = x;
    movement.y    = y;
    return queue(movement);
}

Chassis &Chassis::turnToHeading(float theta, Movement movement) {
    movement.type  = TURN_TO_HEADING;
    movement.theta = theta;
    return queue(movement);
}

// Movement enabled

void Chassis::setMovementsEnabled(bool enabled) {
    movementsEnabled_ = enabled;
}

// Power / brake

void Chassis::power(int volt) {
    leftMotors_->move(volt);
    rightMotors_->move(volt);
}

void Chassis::power(int leftVolt, int rightVolt) {
    leftMotors_->move(leftVolt);
    rightMotors_->move(rightVolt);
}

void Chassis::stop() {
    power(0);
    isRunning_ = false;
    while (!movementQueue_.empty())
        movementQueue_.pop();
}

void Chassis::setBrakeMode(pros::motor_brake_mode_e brakeMode) {
    brakeMode_ = brakeMode;
    leftMotors_->set_brake_mode_all(brakeMode);
    rightMotors_->set_brake_mode_all(brakeMode);
}

// Wait helpers

void Chassis::waitUntilDone() {
    while (isRunning_ || !movementQueue_.empty())
        pros::delay(10);
}

void Chassis::waitUntilDistance(float distance) {
    while (distanceTraveled_ < distance && (isRunning_ || !movementQueue_.empty()))
        pros::delay(10);
}

float Chassis::getDistanceTraveled() {
    return distanceTraveled_;
}

// Drive loop

void Chassis::drive() {
    isRunning_ = true;

    while (!movementQueue_.empty()) {
        currentMovement_ = movementQueue_.front();
        movementQueue_.pop();

        lateralPID_.reset();
        angularPID_.reset();
        lateralExit_.reset();
        angularExit_.reset();
        distanceTraveled_ = 0.0f;

        int startTime = pros::millis();

        while (true) {
            if (!movementsEnabled_) {
                power(0);
                pros::delay(10);
                continue;
            }

            Pose pose = odometry_.getPose();

            float dx = currentMovement_.x - pose.x;
            float dy = currentMovement_.y - pose.y;
            float distToTarget = std::sqrt(dx * dx + dy * dy);

            float lateralError = 0.0f;
            float angularError = 0.0f;

            if (currentMovement_.type == MOVE_TO_POINT) {
                float targetAngle = radToDeg(std::atan2(dx, dy));
                angularError  = wrapAngle180(targetAngle - pose.theta);
                lateralError  = distToTarget * (currentMovement_.forwards ? 1.0f : -1.0f);

                if (!currentMovement_.forwards) angularError = wrapAngle180(angularError + 180.0f);

            } else if (currentMovement_.type == TURN_TO_POINT) {
                float targetAngle = radToDeg(std::atan2(dx, dy));
                angularError = wrapAngle180(targetAngle - pose.theta);
                lateralError = 0.0f;

            } else if (currentMovement_.type == TURN_TO_HEADING) {
                angularError = wrapAngle180(currentMovement_.theta - pose.theta);
                lateralError = 0.0f;
            }

            float lateralVolt = lateralPID_.compute(lateralError);
            float angularVolt = angularPID_.compute(angularError);

            // Apply angular correction only for MOVE_TO_POINT
            if (currentMovement_.type != MOVE_TO_POINT || !currentMovement_.angularCorrection)
                angularVolt = (currentMovement_.type == MOVE_TO_POINT) ? 0.0f : angularVolt;

            // Clamp lateral voltage
            lateralVolt = clamp(lateralVolt, -currentMovement_.voltMax, currentMovement_.voltMax);
            if (std::abs(lateralVolt) < currentMovement_.voltMin && lateralError != 0.0f)
                lateralVolt = currentMovement_.voltMin * sign(lateralVolt);

            // Clamp angular voltage
            angularVolt = clamp(angularVolt, -currentMovement_.voltMax, currentMovement_.voltMax);

            float leftVolt  = lateralVolt + angularVolt;
            float rightVolt = lateralVolt - angularVolt;

            // Scale down if either side exceeds voltMax
            float maxVolt = std::max(std::abs(leftVolt), std::abs(rightVolt));
            if (maxVolt > currentMovement_.voltMax) {
                float scale = currentMovement_.voltMax / maxVolt;
                leftVolt  *= scale;
                rightVolt *= scale;
            }

            power(static_cast<int>(leftVolt), static_cast<int>(rightVolt));

            distanceTraveled_ += std::abs(lateralError);

            // Exit conditions
            bool lateralDone = lateralExit_.update(lateralError);
            bool angularDone = angularExit_.update(angularError);
            bool timedOut    = (pros::millis() - startTime) >= currentMovement_.timeout;

            bool done = timedOut;
            if (currentMovement_.type == MOVE_TO_POINT)
                done = done || (lateralDone && (!currentMovement_.angularCorrection || angularDone));
            else
                done = done || angularDone;

            if (done) {
                power(0);
                break;
            }

            pros::delay(10);
        }
    }

    isRunning_ = false;
}

// Stick assignment - change these functions to remap which physical stick
// axis controls throttle vs turn. Every drive mode below reads through
// these, so remapping ever needs to happen in exactly one place.

static int getThrottleStick() {
    return master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
}

static int getSplitTurnStick() {
    return master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
}

static int getArcadeTurnStick() {
    return master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X);
}

// Drive types (opcontrol)

void Chassis::runDrive() {
    switch (driveType_) {
        case TANK:   tank();   break;
        case ARCADE: arcade(); break;
        case SPLIT:  split();  break;
        case HALO:   halo();   break;
    }
}

void Chassis::tank() {
    int left  = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
    int right = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);
    power(left, right);
}

void Chassis::arcade() {
    int throttle = getThrottleStick();
    int turn     = getArcadeTurnStick();
    power(throttle + turn, throttle - turn);
}

void Chassis::split() {
    int throttle = getThrottleStick();
    int turn     = getSplitTurnStick();
    power(throttle + turn, throttle - turn);
}

void Chassis::halo() {
    // Holonomic / halo drive — uses left stick for strafe, right X for rotation
    int throttle = getThrottleStick();
    int strafe   = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X);
    int turn     = getSplitTurnStick();
    // For a standard tank chassis, strafe is ignored
    power(throttle + turn, throttle - turn);
}