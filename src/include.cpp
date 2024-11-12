#include "include.hpp"
#include "pros/motors.hpp"
#include "pros/optical.hpp"

pros::Controller controller(pros::E_CONTROLLER_MASTER);

pros::MotorGroup leftMotors({-11, -13, 12});
pros::MotorGroup rightMotors({18, 20, -19});

pros::Motor intake(-10);

pros::Motor arm(1);

pros::Imu imu(17);
pros::Optical optical(4);

pros::adi::DigitalOut clampPis('A');
pros::adi::DigitalOut sweeper('C');
pros::adi::DigitalOut intakePis('B');