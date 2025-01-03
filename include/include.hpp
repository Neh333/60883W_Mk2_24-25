#pragma once
#include "main.h"
#include "math.h"
#include "pros/optical.hpp"
#define MIN(a,b) ((a)<(b)?(a):(b)) /* takes param "A" & "B" if A is less than B then A if not then B */
#define MAX(a,b) ((a)>(b)?(a):(b)) /* takes param "A" & "B" if A is greather than than B then A if not then B */

#define AUTO_COUNT 12 /* Can't used static types for array len or const uint soo womp cope */

enum autoColor{
    red,
    blue
};

struct autonTextTuple
{
    std::string autoName;
    std::function<void()> autonomous;
};


/* Global Variable Declaration */
extern uint auton;
extern autonTextTuple autos[];

extern pros::Controller controller;

/* Declare motors */
extern pros::MotorGroup leftMotors;
extern pros::MotorGroup rightMotors;

extern pros::Motor intake;

extern pros::Motor arm;

/* Declare V5 sensors */
extern pros::Imu imu;
extern pros::Optical optical;
extern pros::Rotation armPot;

/* ADI Digital out */
extern pros::adi::DigitalOut clampPis;
extern pros::adi::DigitalOut intakePis;
extern pros::adi::DigitalOut mogoArm;
extern pros::adi::DigitalOut mogoArmClamp;

/* Global vars */
extern bool backClampTog;
extern autoColor currentColor;

