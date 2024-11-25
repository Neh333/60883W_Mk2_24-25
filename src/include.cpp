#include "include.hpp"
#include "pros/motors.hpp"
#include "pros/optical.hpp"
#include "pros/rotation.hpp"

pros::Controller controller(pros::E_CONTROLLER_MASTER);

pros::MotorGroup leftMotors({-11, -13, 12});
pros::MotorGroup rightMotors({18, 20, -19});

pros::Motor intake(-10);

pros::Motor arm(1);

pros::Imu imu(17);
pros::Rotation armPot(2);
pros::Optical optical(9);

pros::adi::DigitalOut clampPis('H');
pros::adi::DigitalOut mogoArm('G');
pros::adi::DigitalOut mogoArmClamp('F');

pros::adi::DigitalOut intakePis('B');