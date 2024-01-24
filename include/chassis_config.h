#ifndef ROBOT10_CHASSIS_CONFIG
#define ROBOT10_CHASSIS_CONFIG

#include "lemlib/api.hpp"
#include "port_config.h"

// motors
pros::Motor left_top_motor(LEFT_TOP_DRIVE_PORT, true);
pros::Motor left_bottom_motor(LEFT_BOTTOM_DRIVE_PORT, false);
pros::Motor left_front_motor(LEFT_FRONT_DRIVE_PORT, false);
pros::Motor right_top_motor(RIGHT_TOP_DRIVE_PORT, false);
pros::Motor right_bottom_motor(RIGHT_BOTTOM_DRIVE_PORT, true);
pros::Motor right_front_motor(RIGHT_FRONT_DRIVE_PORT, true);

pros::Imu imu(INERTIAL_PORT);

pros::MotorGroup leftMotors({left_top_motor, left_bottom_motor, left_front_motor}); // left motor group
pros::MotorGroup rightMotors({right_top_motor, right_bottom_motor, right_front_motor}); // right motor group

// drivetrain settings
lemlib::Drivetrain drivetrain(&leftMotors, // left motor group
                              &rightMotors, // right motor group
                              10, // 10 inch track width
                              lemlib::Omniwheel::NEW_325, // using new 3.25" omnis
                              200, // drivetrain rpm is 360
                              2 // chase power is 2. If we had traction wheels, it would have been 8
);

// lateral motion controller
lemlib::ControllerSettings linearController(4, // proportional gain (kP)
                                            0, // integral gain (kI)
                                            3, // derivative gain (kD)
                                            3, // anti windup
                                            1, // small error range, in inches
                                            100, // small error range timeout, in milliseconds
                                            3, // large error range, in inches
                                            500, // large error range timeout, in milliseconds
                                            20 // maximum acceleration (slew)
);

// angular motion controller
lemlib::ControllerSettings angularController(1, // proportional gain (kP)
                                             0, // integral gain (kI)
                                             10, // derivative gain (kD)
                                             3, // anti windup
                                             1, // small error range, in degrees
                                             100, // small error range timeout, in milliseconds
                                             3, // large error range, in degrees
                                             500, // large error range timeout, in milliseconds
                                             0 // maximum acceleration (slew)
);

// sensors for odometry
// note that in this example we use internal motor encoders (IMEs), so we don't pass vertical tracking wheels
lemlib::OdomSensors sensors(nullptr, // vertical tracking wheel 1, set to null
                            nullptr, // vertical tracking wheel 2, set to nullptr as we are using IMEs
                            nullptr, // horizontal tracking wheel 1
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            &imu // inertial sensor
);

// create the chassis
lemlib::Chassis chassis(drivetrain, linearController, angularController, sensors);

#endif // ROBOT10_CHASSIS_CONFIG