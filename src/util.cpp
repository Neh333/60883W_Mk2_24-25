#include "include.hpp"
#include "pros/optical.h"
#include "pros/rtos.hpp"
#include "util.hpp"

/* Returns 1 if x is positive or zero or -1 if x is negative */
int8_t sgn(double x){
  return (x >= 0) - (x < 0);
}

/* Return degrees given radians */
double degrees(double radians)
{
  return(radians * (180/M_PI));
}

/* Return radians given degrees */
double radians(double degrees)
{
  return(degrees * (M_PI/180));
}

/*
 *1800 ticks/rev with 100 rpm cartridge
 *900 ticks/rev with 200 rpm cartridge
 *300 ticks/rev with 600 rpm cartridge

 *400 ticks for one full wheel rotation (300 * (48/36) or (4/3)) (reversed gear ratio)
 *Circumference of 3.25" omni = 10.2101761242 (3.25*pi)

 *400 ticks / 10.2101761242 inches = 39.1766013763 ticks per inch
 */
double inchToTick(double inch) {
  return (inch * 39.1766013763);
}

double tickToInch(double tick) {
  return (tick / 39.1766013763);
}

/* Return voltage given percent */
double percentToVoltage(double percent)
{
  return(percent * 120);
}

/* Return velocity given voltage (assumes cartridge is not specified in motor instantiation) */
double voltageToVelocity(double voltage)
{
  return(voltage / 60);
}

/* Return voltage given velocity (assumes cartridge is not specified in motor instantiation) */
double velocityToVoltage(double percent)
{
  return(percent * 60);
}

/* Find the shortest distance between two angles */
double distBetweenAngles(double targetAngle, double currentAngle)
{
    return std::remainder(targetAngle-currentAngle,360);
}

/* Find the shortest distance between current angle and a desired angle */
double imuTarget(double target)
{
    return std::fabs(distBetweenAngles(target, imu.get_heading()));
}

/**
 * Does trigonometry to assign values to attributes of a given Triangle struct
 *
 * \param obj
 *        Reference to the Triangle object who's attributes will be set
 * \param a
 *        Leg A of the triangle
 * \param reference
 *        Angle to subtract from the current IMU heading. 
 *        Will set hyp equal to A if reference and IMU heading are the same.   
 */
void findTri(struct Triangle *obj, double a, double reference)
{
  obj->beta = radians(imu.get_heading() - reference);
  obj->alpha = M_PI/2 - obj->beta;
  obj->a = a;
  obj->b =  a * tan(obj->beta);
  obj->hyp = sqrt(a*a + obj->b*obj->b);
}

/* Print the currently selected auto, and some other information, to the controller */
void controllerPrintAuto()
{
	controller.print(2, 0, "%s, %.2f            ", autos[auton%AUTO_COUNT].autoName, imu.get_heading());
}

/* Print a message informing of calibration, then calibrate */
void pauseAndCalibrateIMU()
{
    imu.reset();
    int iter = 0;
    while(imu.is_calibrating())
    {
        controller.print(2,0,"Calibrating: %.2f    ", iter/1000.f);
        iter += 20;
        pros::delay(20);
    }
}