#ifndef AUTONOMOUS_HPP
#define AUTONOMOUS_HPP
#include "api.h"
#include <string>



enum autonNames {
    none,
    awp,
    red1,
    red2,
    blue1,
    blue2,
    skills0
};

enum matchType {
    QUALS,
    ELIMS
};

typedef autonNames autonID;

extern autonID selectedAuton;

extern matchType currentMatch;

extern pros::Mutex lcdMutex; // guards all pros::lcd:: calls across tasks

void autonomousSelector(autonID id);
void run_selected_auton(autonID id);

void skills();
void solo_awp();
void neutral_goal_red();
void red_goal();
void neutral_goal_blue();
void blue_goal();

// Display / selector helpers (defined in main.cpp)
std::string getAutonName(autonID id);
std::string getMatchTypeName(matchType type);
void updateDisplay(std::string name);
void auton_selector();
bool opTog();
void toggleMatchType();
void config();

#endif