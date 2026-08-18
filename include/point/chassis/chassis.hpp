#pragma once

#include "point/odometry/exitCondition.hpp"
#include "point/odometry/odom.hpp"
#include "point/odometry/PID.hpp"
#include "pros/motor_group.hpp"

#include <queue>

namespace point {

    enum movementType {
        MOVE_TO_POINT = 0,
        TURN_TO_POINT,
        TURN_TO_HEADING
    };

    enum driveType {
        TANK,
        ARCADE,
        SPLIT, 
        HALO
    };

    struct Movement {
        movementType type;
        float x = 0;
        float y = 0;
        float theta = 0;
        float distance = 0;
        bool forwards = true;
        float voltMax = 127;
        float voltMin = 0;
        float exitRange = 0;
        bool angularCorrection = true;
        bool driveIntake = true;
        int timeout = 3000;
    };

    struct ChassisConfig {
        pros::MotorGroup* leftMotors = nullptr;
        pros::MotorGroup* rightMotors = nullptr;
        driveType drivetype_ = SPLIT;
    };

    class Chassis {
    public:
        explicit Chassis(ChassisConfig chassisConfig, Odometry& odometry,
                        PID& lateralPID, PID& angularPID,
                        exitCondition& lateralExit, exitCondition& angularExit);
        
        void setMotors(pros::MotorGroup* left, pros::MotorGroup* right); // wire motors in after construction

        void runDrive(); // run driveType
        
        void setMovementsEnabled(bool enabled);
        Chassis &queue(movementType type, Movement movement);
        Chassis &queue(Movement movement);
        void waitUntilDone();

        Chassis &moveDistance(float distance, Movement movement = {});
        Chassis &moveDistance(float distance, Movement movement, float startX, float startY);
        Chassis &moveDistance(float distance, Movement movement, float startX, float startY, float startTheta);

        Chassis &moveToPoint(float x, float y, Movement movement = {});
        Chassis &turnToPoint(float x, float y, Movement movement = {});
        Chassis &turnToHeading(float theta, Movement movement = {});

        float getDistanceTraveled();
        void waitUntilDistance(float distance);

        void stop();
        void power(int volt);
        void power(int leftVolt, int rightVolt);
        void setBrakeMode(pros::motor_brake_mode_e brakeMode);

        void drive();

        // drive types
        void tank();
        void arcade();
        void split();
        void halo();

    private:
        pros::MotorGroup* leftMotors_;
        pros::MotorGroup* rightMotors_;
        
        driveType driveType_;
        Odometry &odometry_;

        PID& lateralPID_;
        PID& angularPID_;
        exitCondition& lateralExit_;
        exitCondition& angularExit_;

        std::queue<Movement> movementQueue_;
        bool movementsEnabled_ = true;
        float distanceTraveled_ = 0.0f;

        pros::motor_brake_mode_e brakeMode_ = pros::E_MOTOR_BRAKE_COAST;
        Movement currentMovement_;
        bool isRunning_ = false;
    };
}