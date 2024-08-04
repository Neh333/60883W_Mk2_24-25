#pragma once
#include "main.h"
#include "math.h"
#define MIN(a,b) ((a)<(b)?(a):(b)) /* takes param "A" & "B" if A is less than B then A if not then B */
#define MAX(a,b) ((a)>(b)?(a):(b)) /* takes param "A" & "B" if A is greather than than B then A if not then B */

#define AUTO_NUMBER 12 /* Can't used static types for array len or const uint soo womp cope */
extern uint8_t auton; 

extern pros::Controller controller;

/* Declare motors */
extern pros::MotorGroup leftMotors;
extern pros::MotorGroup rightMotors;

extern pros::Motor intake;

/* Declare V5 sensors */
extern pros::Imu imu;

/* ADI Digital out */
extern pros::adi::DigitalOut mogoMechPisses;

/* Global vars */
extern bool backClampTog;