#include "main.h"

using namespace point;

/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
// void on_center_button() {
	// static bool pressed = false;
	// pressed = !pressed;
	// if (pressed) {
		// pros::lcd::set_text(2, "I was pressed!");
	// } else {
		// pros::lcd::clear_line(2);
	// }
// }

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
    robotEnabled = false;

    pros::lcd::initialize();
    pros::delay(100);

    config();

    static pros::Task selectorTask(auton_selector, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT * 2, "auton_selector");

    selDisp();
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {
    chassis.stop();

    robotEnabled = false;
    updateDisplay(getAutonName(selectedAuton));
    devOk();
}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {
    robotEnabled = false;
    updateDisplay(getAutonName(selectedAuton));
    devOk();

    compConfig();
}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {
    robotEnabled = true;

    lcdMutex.take();
    pros::lcd::set_text(0, "                    ");
    pros::lcd::print(1, "RUNNING AUTON: %s", getAutonName(selectedAuton).c_str());
    pros::lcd::set_text(2, "                    ");
    pros::lcd::set_text(3, "                    ");
    pros::lcd::set_text(4, "                    ");
    pros::lcd::set_text(5, "                    ");
    pros::lcd::set_text(6, "                    ");
    pros::lcd::set_text(7, "                    ");
    lcdMutex.give();

    autonomousSelector(selectedAuton);

    lcdMutex.take();
    pros::lcd::print(3, "AUTON COMPLETED. AWAITING DRIVER CONTROL.");
    lcdMutex.give();
}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol() {
    bool testingMode = !pros::competition::is_connected();
    robotEnabled = testingMode ? robotEnabled : true;

    if (robotEnabled) opDisp();
    bool wasEnabled = robotEnabled;

    while (true) {
        if (testingMode) opTog();

        if (robotEnabled != wasEnabled) {
            if (robotEnabled) {
                opDisp();
            } else {
                selDisp();
            }
            wasEnabled = robotEnabled;
        }

        if (robotEnabled) chassis.runDrive();
        pros::delay(20);
    }
}