#include "main.h"
#include "lemlib/api.hpp"

#include "chassis_config.h"
#include "port_config.h"

const int forward = 127;
const int reverse = -127;
const int matchload_forward = 127;
const int slapper_slow_forward = 40;
const int wing_released = 1;
const int wing_contracted = 0;
const int stop = 0;
const double potentiometer_stop = 130.0;
const int timeout = 5000;

pros::Controller controller(pros::E_CONTROLLER_MASTER);
pros::Motor slapperLeft(SLAPPER_LEFT_PORT, false), slapperRight(SLAPPER_RIGHT_PORT, true);
pros::ADIDigitalOut wing_left(WING_LEFT_PORT), wing_right(WING_RIGHT_PORT);
pros::ADIPotentiometer potentiometer(POTENTIOMETER_PORT);

bool auto_position = false;
bool climb_position = false;

void toggle_auto_position() {
	auto_position = !auto_position;
}

void wing_release(bool left=true, bool right=true) {
	if (left) wing_left.set_value(wing_released);
	if (right) wing_right.set_value(wing_released);
}

void wing_retract(bool left=true, bool right=true) {
	if (left) wing_left.set_value(wing_contracted);
	if (right) wing_right.set_value(wing_contracted);
}

void climb() {
	while (potentiometer.get_angle() < 240.0) {
		slapperLeft.move(slapper_slow_forward);
		slapperRight.move(slapper_slow_forward);
		pros::delay(10.0);
	}
	slapperLeft.move(stop);
	slapperRight.move(stop);
}

void prep_climb() {
	auto_position = false;
	climb_position = false;
	pros::Task climb_task(climb);
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

void basic_autonomous() {
	chassis.tank(50, 50);
	pros::delay(4000.0);
	chassis.tank(0, 0);
	pros::delay(100.0);
	chassis.tank(-50, -50);
	pros::delay(750.0);
	chassis.tank(0, 0);
	pros::delay(100.0);
	chassis.tank(100, 100);
	pros::delay(850.0);
	chassis.tank(0, 0);
	pros::delay(100.0);
	chassis.tank(-50, -50);
	pros::delay(400.0);
	chassis.tank(0, 0);
}

void autonomous_match_load() {
	chassis.setPose({-49, -58, -23.45});
	
	// triball
	chassis.tank(50.0, 50.0);
	pros::delay(800.0);
	chassis.tank(0.0, 0.0);
	pros::delay(100.0);
	chassis.tank(100.0, 100.0);
	pros::delay(350.0);
	chassis.tank(0.0, 0.0);
	pros::delay(150.0);

	// return to start
	chassis.moveToPose(-48, -56, 180, timeout);
	chassis.waitUntilDone();

	// idfk
	wing_release(false, true);
	pros::delay(100);
	chassis.turnTo(0, -56, timeout);
	chassis.waitUntilDone();
	wing_retract();
	pros::delay(100);
	chassis.moveToPoint(-35, -57, timeout);
	chassis.waitUntilDone();
	chassis.moveToPoint(-12, -57, timeout);
	chassis.waitUntilDone();
	wing_release(false, true);
	pros::delay(100);
	chassis.turnTo(0, 0, timeout);
	chassis.waitUntilDone();
	pros::delay(100);
	wing_retract();
	pros::delay(100);
	chassis.moveToPoint(-10, -57, timeout);
	chassis.waitUntilDone();
	chassis.turnTo(0, -57, timeout);
	chassis.waitUntilDone();
	wing_release(true, false);
	pros::delay(100);
	chassis.moveToPoint(-6, -58, timeout);
	chassis.waitUntilDone();
}

void autonomous_goal() {
	chassis.setPose({49, -58, 23.45});
	
	// triball
	chassis.tank(50.0, 50.0);
	pros::delay(800.0);
	chassis.tank(0.0, 0.0);
	pros::delay(100.0);
	chassis.tank(100.0, 100.0);
	pros::delay(350.0);
	chassis.tank(0.0, 0.0);
	pros::delay(150.0);

	chassis.tank(-70.0, -70.0);
	pros::delay(350.0);
	chassis.tank(0.0, 0.0);
	pros::delay(100.0);
	
	chassis.moveToPoint(51.413, -39.716, timeout);
	chassis.waitUntilDone();
	chassis.moveToPoint(27.027, -25.243, timeout);
	chassis.waitUntilDone();
	chassis.moveToPoint(19.752, -5.166, timeout);
	chassis.waitUntilDone();
	chassis.turnTo(70, -5.166, timeout);
	chassis.waitUntilDone();
	
	pros::delay(150.0);
	chassis.tank(-20.0, -20.0);
	pros::delay(250.0);
	chassis.tank(0, 0);
	wing_release(true, false);
	pros::delay(150.0);
	chassis.tank(70.0, 70.0);
	pros::delay(800.0);
	chassis.tank(0, 0);
	pros::delay(250.0);
	chassis.tank(100, 100);
	pros::delay(250.0);
	chassis.tank(0, 0);
	pros::delay(150.0);
	chassis.tank(-20.0, -20.0);
	pros::delay(150.0);
	chassis.tank(0.0, 0.0);
	wing_retract();
	
	chassis.tank(-60.0, -60.0);
	pros::delay(250);
	chassis.tank(0.0, 0.0);
	pros::delay(100);
	
	chassis.moveToPoint(14, -33.0, timeout);
	chassis.waitUntilDone();

	chassis.turnTo(14, -50, timeout);
	chassis.waitUntilDone();
	
	wing_release(false, true);
	chassis.tank(30, 30);
	pros::delay(300);
	chassis.tank(-20, 20);
	pros::delay(2000);
	chassis.tank(0, 0);
}

void autonomous_skills() {
	chassis.tank(-20, -20);
	int shots = 0;
	while (shots < 49) {
		if (potentiometer.get_angle() > potentiometer_stop) {
			slapperLeft.move(matchload_forward);
			slapperRight.move(matchload_forward);
		} else {
			shots++;
			slapperLeft.move(stop);
			slapperRight.move(stop);
			pros::delay(250);
			slapperLeft.move(matchload_forward);
			slapperRight.move(matchload_forward);
			pros::delay(270);
		}
	}
	chassis.tank(0, 0);

	chassis.calibrate();
	slapperLeft.move(stop);
	slapperRight.move(stop);
	pros::delay(3000.0);
	chassis.setPose({-48, -57, 45.0});
	
	chassis.moveToPoint(-24, -58, timeout);
	while (potentiometer.get_angle() < 240.0) {
		slapperLeft.move(slapper_slow_forward);
		slapperRight.move(slapper_slow_forward);
	}
	slapperLeft.move(stop);
	slapperRight.move(stop);
	chassis.waitUntilDone();
	
	chassis.turnTo(-35, -58, timeout);
	chassis.waitUntilDone();

	chassis.tank(-35, -35);
	pros::delay(3000.0);
	chassis.tank(0, 0);
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
		
		if (controller.get_digital(DIGITAL_LEFT)) {
			slapperLeft.move(slapper_slow_forward);
			slapperRight.move(slapper_slow_forward);
		}
		else {
			if ((potentiometer.get_angle() > potentiometer_stop && auto_position) || controller.get_digital(DIGITAL_L2)) {
				slapperLeft.move(forward);
				slapperRight.move(forward);
			} else if (!climb_position) {
				slapperLeft.move(stop);
				slapperRight.move(stop);
			}
		}
		
		if (controller.get_digital_new_press(DIGITAL_RIGHT)) {
			prep_climb();
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


void competition_initialize() {

}

void autonomous() {
	// autonomous_match_load();
	// autonomous_goal();
	// autonomous_skills();
}

void opcontrol() {
	pros::Task drive_task(drive_loop);
	// autonomous_match_load();
	// autonomous_goal();
	// while (1) {
	// 	screen();
	// }
	// autonomous_skills();
}
