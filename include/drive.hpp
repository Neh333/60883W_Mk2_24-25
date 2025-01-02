#pragma once
#include "main.h"
#include "util.hpp"
#include <tuple>
#include <utility>

//better than a macro soo
const double INTEGRAL_MAX = 800.0;

#define DEFAULT_STANDSTILL_EXIT 7

#define NO_SCHEDULING -1.f

enum PID_dir{
  forward,
  backward,
  left,
  right,
  shortest,
  forwardRight,
  forwardLeft,
  forwardShortest,
  backwardRight,
  backwardLeft,
  backwardShortest
};

enum movement_Type{
  lateral_t,
  turn_t
};

struct errorFuncTuple{
  std::function<void()> func;
  double onError;
  bool called;

  errorFuncTuple(std::function<void()> func, double onError, bool called) : 
   func(func), onError(onError), called(called){}
};

void onError_fn(void* param);

struct PIDprofile{
  double kP;
  double kP_a;
  double kI;
  double kI_a;
  double kD;
  double kD_a;
  double kP_d;
};

struct slewProfile{
  double slew;
  double slew_lower_thresh;
  double slew_upper_thresh;
};


class Drive{
  private:
  const PIDprofile PIDConstants[9] = {
  /*{kP, kPa, kI, kIa, kD,  kDa,  kPd}*/
    {30, 87,  0,  13,  120, 100,  150},/*60+ degree turns (70%) / 5-48in lat*/

    {31, 82,  0, 14.5, 100,  227,  500}, /*75+ degree mogo turns (90%) / 5-48 mogo lat*/ 
    
    {16, 140,  0, 12,  30,  400,  250}, /*10-55 deg turn (100%) / 49-72in lat(verify at some point lol)*/ 
    
    {20, 145, 0,  8,   50,  350,  500}, /*20-70 degree mogo turns (100%) / 50-72 mogo lat*/    
    
    {20,  100, 0,  0,   60,  130,  0}, /*goal rush swerve 1 (red)*/    

    {18,  58, 0,  2,   38,  190,  0} /*goal rush swerve 2(red)*/    
  };

  double kP, kP_a, kI, kI_a, kD, kD_a, kP_d;
  PIDprofile scheduledConstants;
  double scheduleThreshold_l, scheduleThreshold_a; 

  double error;
  
  int maxVolt, maxVolt_a;

  unsigned short standstillExitCount;

  double update(double KP, double KI, double KD, double error, double lastError, double *integral, double integralActive);
  void adjustForSlew(int *voltage, double actualVelocity, struct slewProfile *profile);
  bool updateStandstill(unsigned short *standstillCounter, double error, double lastError, double errorThreshold);

  /* "Virtual" Drivetrain methods */
  double leftDriveAvgPos(); 
  double rightDriveAvgPos();
  double driveAvgPos();

  /* Returns a float between 0 and 100, representing how much energy is being used compared to actual movement */
  double driveEfficiencyAll();

  /* Returns a float between -200 and 200 */
  double actualVelocityLeft();
  double actualVelocityRight();
  double actualVelocityAll();

  bool runningPID = false;

  const double integralActive = inchToTick(3);
  const double integralActive_a = 15;
  
  struct slewProfile slewProf;
  struct slewProfile slewProf_a;

  std::tuple<double, double, double> pose; //{x, y, theta/headiing}

  public:
  Drive(pros::MotorGroup &leftMotors, pros::MotorGroup &rightMotors, pros::Imu &imu);

  /* "Virtual" Drivetrain attributes and methods */ 
  pros::MotorGroup *rightMotors;
  pros::MotorGroup *leftMotors;
  pros::Imu        *imu;

  void setBrakeMode(pros::motor_brake_mode_e brakeMode);
  void moveLeftDriveVoltage(int voltage);
  void moveRightDriveVoltage(int voltage);
  void moveDriveVoltage(int voltage); 

  void moveDriveTrain(int voltage, float time);

  std::vector<errorFuncTuple> onErrorVector;
  void addErrorFunc(double onError, void input());

  /* Setters */
  void setPID(char n);
  void setScheduledConstants(char n);

  void setCustomPID(PIDprofile constants);
  void setScheduleThreshold_l(double error);
  void setScheduleThreshold_a(double error);

  void setMaxVoltage(double pctVoltage);
  void setMaxTurnVoltage(double pctVoltage);
  
  void setSlew(slewProfile profile);
  void setSlew_a(slewProfile profile);

  void setStandstillExit(int exitOn);

  /* Getters */
  const double getError();
  const bool PIDisActive();
  
  /*Tracking fn */
  void tracking();
  
  /* Movement Functions, Return error after movement is finished */
  double move(PID_dir dir, double target, double timeOut, double maxVelocity);
  double turn(PID_dir dir, double target, double timeOut, double maxVelocity);
  double swerve(PID_dir dir, double target, double target_a, double timeOut, double maxVel, double maxVel_a);
  double hardStop(PID_dir dir, double targetCutOff, double target, double timeOut, double maxVelocity);
  double hardStopSwerve(PID_dir dir, double targetCutOff, double target, double target_a, double timeOut, double maxVel, double maxVel_a);
  double brake(double timeOut);
};

/* Declare, but do not create, an instance of Drive */
extern Drive drive;

void startTracking_fn(void* param);