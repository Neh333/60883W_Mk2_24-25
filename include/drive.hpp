#pragma once
#include "main.h"
#include "util.hpp"

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

struct MoveParams {
  int minSpeed = 0;
  /** distance between the robot and target point where the movement will exit. Only has an effect if minSpeed is
   * non-zero.*/
  float earlyExitRange = 0;
};

struct TurnParams {
  int minSpeed = 0;
  /** angle between the robot and target point where the movement will exit. Only has an effect if minSpeed is
   * non-zero.*/
  float earlyExitRange = 0;
};


class Drive{
  private:
  const PIDprofile PIDConstants[7] = {
  /*{kP, kPa, kI, kIa, kD,  kDa,  kPd}*/
    {14, 87,  0,  10,  30,   90,  0},/*50+ degree turns (70%) / gen lat*/

    {20, 90,   0, 13,  36,  193, 0}, /*70+ degree mogo turns (90%) / gen mogo lat*/ 
    
    {20, 143,  0, 8,   36,  300, 0}, /*40-60 degree mogo turns (90%) / gen mogo lat*/

    {20, 237,  0, 8,   36,  70,  0}, /* random turn (70%) / gen mogo lat*/  
      
    {20, 210,  0, 0,   36,  450,  0}, /* 30 deg mogo turns (70%) / gen mogo lat*/ 

    {20, 87,   0, 0,   30,  490,  0}, /*25 deg turn (mogo?) (70%) / gen lat*/

    {20, 88,   0, 0,   30,  257,  0} /*180 deg turn (mogo?) (90%) / gen mogo  lat*/
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

  struct MoveParams moveParams;
  struct TurnParams turnParams;

  public:
  Drive(pros::MotorGroup &leftMotors, pros::MotorGroup &rightMotors, pros::Imu &imu){
   setPID(1);
   setScheduleThreshold_l(NO_SCHEDULING);
   setScheduleThreshold_a(NO_SCHEDULING);

   setSlew({0, 0, 0});
   setSlew_a({0, 0, 0});

   setMoveParams({0,0});
   setTurnParams({0,0});

   this->leftMotors      = &leftMotors;
   this->rightMotors     = &rightMotors;
   this->imu             = &imu;

   setStandstillExit(DEFAULT_STANDSTILL_EXIT);
  }



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

  void setMoveParams(MoveParams params);
  void setTurnParams(TurnParams params);
  
  void setStandstillExit(int exitOn);

  /* Getters */
  const double getError();
  const bool PIDisActive();
  
  /* Movement Functions, Return error after movement is finished */
  double move(PID_dir dir, double target, double timeOut, double maxVelocity, Triangle tri);
  double turn(PID_dir dir, double target, double timeOut, double maxVelocity);
  double swerve(PID_dir dir, double target, double target_a, double timeOut, double maxVel, double maxVel_a);
  double hardStop(PID_dir dir, double targetCutOff, double target, double timeOut, double maxVelocity);
  double hardStopSwerve(PID_dir dir, double targetCutOff, double target, double target_a, double timeOut, double maxVel, double maxVel_a);
  double brake(double timeOut);
};

/* Declare, but do not create, an instance of Drive */
extern Drive drive;