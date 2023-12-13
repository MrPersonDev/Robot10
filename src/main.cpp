#include "main.h"

pros::Controller controller(pros::E_CONTROLLER_MASTER);
pros::Motor leftTop(9, false), leftBottom(17, true);
pros::Motor rightTop(2, true), rightBottom(13, true);
pros::Motor slapperLeft(20, false), slapperRight(11, true);
pros::ADIDigitalOut leftWing('A'), rightWing('B');
pros::ADIPotentiometer potentiometer('C');

void wing_release() {
	leftWing.set_value(1);
	rightWing.set_value(1);
}

void wing_retract() {
	leftWing.set_value(0);
	rightWing.set_value(0);
}

void initialize() {
	pros::lcd::initialize();
	pros::lcd::set_background_color(LV_COLOR_BLACK);
}

void disabled() {}

void competition_initialize() {}

void autonomous() {}

void opcontrol() {
	while (true) {
		int drive = controller.get_analog(ANALOG_LEFT_Y);
		int turn = controller.get_analog(ANALOG_RIGHT_X);
        
        int left = drive + turn;
        int right = drive - turn;
        
		leftTop.move(left);
		leftBottom.move(left);
		rightTop.move(right);
		rightBottom.move(right);
		
		if (controller.get_digital_new_press(DIGITAL_R1)) {
			wing_retract();
		} else if (controller.get_digital_new_press(DIGITAL_R2)) {
			wing_release();
		}
        
        if (potentiometer.get_angle() > 130.0 || controller.get_digital(DIGITAL_L2)) {
			slapperLeft.move(127);
			slapperRight.move(127);
        } else {
			slapperLeft.move(127);
			slapperRight.move(127);
        }
    }
}
