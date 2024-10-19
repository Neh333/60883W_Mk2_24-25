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
extern uint8_t auton;
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

/* ADI Digital out */
extern pros::adi::DigitalOut clampPis;
extern pros::adi::DigitalOut tiltPis;
extern pros::adi::DigitalOut intakePis;

/* Global vars */
extern bool backClampTog;

