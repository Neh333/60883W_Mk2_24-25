#include "include.hpp"

pros::Controller controller(pros::E_CONTROLLER_MASTER);

pros::MotorGroup leftMotors({1, 2, 3});
pros::MotorGroup rightMotors({4, 5, 6});

pros::Motor intake(7);

pros::Imu imu(8);

pros::adi::DigitalOut mogoMechPisses('A');