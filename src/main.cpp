#include "main.h"
#include "lemlib/api.hpp"

#include "chassis_config.h"
#include "port_config.h"

const int forward = 127;
const int reverse = -127;
const int wing_released = 1;
const int wing_contracted = 0;
const int stop = 0;
const double potentiometer_stop = 130.0;

pros::Controller controller(pros::E_CONTROLLER_MASTER);
pros::Motor slapperLeft(SLAPPER_LEFT_PORT, false), slapperRight(SLAPPER_RIGHT_PORT, true);
pros::ADIDigitalOut wings(WING_PORT);
pros::ADIPotentiometer potentiometer(POTENTIOMETER_PORT);

bool auto_position = false;

void toggle_auto_position() {
	auto_position = !auto_position;
}

void wing_release() {
	wings.set_value(wing_released);
}

void wing_retract() {
	wings.set_value(wing_contracted);
}

void screen() {
	while (true) {
        lemlib::Pose pose = chassis.getPose(); // get the current position of the robot
        pros::lcd::print(0, "x: %f", pose.x); // print the x position
        pros::lcd::print(1, "y: %f", pose.y); // print the y position
        pros::lcd::print(2, "heading: %f", pose.theta); // print the heading
        pros::delay(10);
    }
}

void drive_loop() {
	while (true) {
		int drive = controller.get_analog(ANALOG_LEFT_Y);
		int turn = controller.get_analog(ANALOG_RIGHT_X);
        
		chassis.arcade(drive, turn);
		
		if (controller.get_digital_new_press(DIGITAL_R1)) {
			wing_retract();
		} else if (controller.get_digital_new_press(DIGITAL_R2)) {
			wing_release();
		}
		
		if (controller.get_digital_new_press(DIGITAL_L1)) {
			toggle_auto_position();
		}
        
        if ((potentiometer.get_angle() > potentiometer_stop && auto_position) || controller.get_digital(DIGITAL_L2)) {
			slapperLeft.move(forward);
			slapperRight.move(forward);
        } else {
			slapperLeft.move(stop);
			slapperRight.move(stop);
        }
		
		pros::delay(10);
	}
}

void initialize() {
	pros::lcd::initialize(); // initialize brain screen
    chassis.calibrate(); // calibrate the chassis
    pros::Task screen_task(screen); // create a task to print the position to the screen
}

void disabled() {}


void competition_initialize() {}

void autonomous() {}

void opcontrol() {
	pros::Task drive_task(drive_loop);
}
