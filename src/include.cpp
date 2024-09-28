#include "include.hpp"
#include "pros/motors.hpp"

pros::Controller controller(pros::E_CONTROLLER_MASTER);

pros::MotorGroup leftMotors({-11, -12, -13});
pros::MotorGroup rightMotors({20, 19, 18});

pros::Motor intake(-10);

pros::Motor arm(1);

pros::Imu imu(9);

pros::adi::DigitalOut clampPis('A');
pros::adi::DigitalOut tiltPis('C');
pros::adi::DigitalOut lockPis('B');