#include "main.h"
#include "setup/autonomous.hpp"
#include "point/chassis/chassis.hpp"

using namespace point;

autonID selectedAuton = none;

// scenarios

void skills() {
    
}

void solo_awp() {
    if (currentMatch == ELIMS) {

    }
}

void neutral_goal_red() {
    if (currentMatch == ELIMS) {

    } 
}

void red_goal() {
    if (currentMatch == ELIMS) {

    } 
}

void neutral_goal_blue() {
    if (currentMatch == ELIMS) {

    } 
}

void blue_goal() {
    if (currentMatch == ELIMS) {

    } 
}

// setup
void autonomousSelector(autonID id) {
    switch (id) {
        case awp:     solo_awp();          break;
        case red1:    neutral_goal_red();  break;
        case red2:    red_goal();          break;
        case blue1:   neutral_goal_blue(); break;
        case blue2:   blue_goal();         break;
        case skills0: skills();            break;
        case none:
        default:
            break;
    }
}

void run_selected_auton(autonID id) {
    autonomousSelector(id);
}