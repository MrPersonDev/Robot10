#include "main.h"

using namespace okapi;

const int forward = 127;
const int reverse = -127;
const int stop = 0;

pros::Controller controller(pros::E_CONTROLLER_MASTER);
pros::Motor leftTop(9, true), leftBottom(8, false);
pros::Motor rightTop(20, false), rightBottom(19, true);
pros::Motor slapperLeft(7, false), slapperRight(10, true);
pros::ADIDigitalOut wings('A');
pros::ADIPotentiometer potentiometer('C');

void wing_release() {
	wings.set_value(1);
}

void wing_retract() {
	wings.set_value(0);
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
			slapperLeft.move(forward);
			slapperRight.move(forward);
        } else {
			slapperLeft.move(stop);
			slapperRight.move(stop);
        }
    }
}
