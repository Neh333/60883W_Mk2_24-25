#include "drive.hpp"
#include "util.hpp"
#include "include.hpp"

#define STANDSTILL_DISTANCE_THRESHOLD 2
#define STANDSTILL_TURN_THRESHOLD 0.2

#define MS_DELTA_TIME 20

/* Set PID constants */
void Drive::setPID(char n)
{
  n--;
  kP = PIDConstants[n].kP;
  kP_a = PIDConstants[n].kP_a;
  kI = PIDConstants[n].kI;
  kI_a = PIDConstants[n].kI_a;
  kD = PIDConstants[n].kD;
  kD_a = PIDConstants[n].kD_a;
  kP_d = PIDConstants[n].kP_d;
}


void Drive::setScheduledConstants(char n)
{
  n--;
  scheduledConstants.kP = PIDConstants[n].kP;
  scheduledConstants.kP_a = PIDConstants[n].kP_a;
  scheduledConstants.kI = PIDConstants[n].kI;
  scheduledConstants.kI_a = PIDConstants[n].kI_a;
  scheduledConstants.kD = PIDConstants[n].kD;
  scheduledConstants.kD_a = PIDConstants[n].kD_a;
  scheduledConstants.kP_d = PIDConstants[n].kP_d;
}


/* Use specified PID constants */
void Drive::setCustomPID(PIDprofile profile)
{
  kP = profile.kP;
  kP_a = profile.kP_a;
  kI = profile.kI; 
  kI_a = profile.kI_a;
  kD = profile.kD;
  kD_a = profile.kD_a;
  kP_d = profile.kP_d;
}

void Drive::setScheduleThreshold_l(double error)
{
  scheduleThreshold_l = error;
}


void Drive::setScheduleThreshold_a(double error)
{
  scheduleThreshold_a = error;
}

/* Set max linear velocity */
void Drive::setMaxVoltage(double pctVoltage)
{
  maxVolt = percentToVoltage(pctVoltage);
}


/* Set max angular voltage by percent */
void Drive::setMaxTurnVoltage(double pctVoltage)
{
  maxVolt_a = percentToVoltage(pctVoltage);
}


/* Update slewProf given profile */
void Drive::setSlew(slewProfile profile)
{
  slewProf = profile;
}


/* Update slewProf given profile */
void Drive::setSlew_a(slewProfile profile)
{
  slewProf_a = profile;
}

/* Set standstillExitCount, will not execute standstill if 0 */
void Drive::setStandstillExit(int exitOn)
{
  standstillExitCount = exitOn;
}

/* Returns the result of the PID calculation, updates integral */
double Drive::update(double KP, double KI, double KD, double error, double lastError, double *integral, double integralActive)
{
  /* Reset integral to zero if the robot has crossed zero-error or is outside of integral range, otherwise integrate */
  if((fabs(error) > integralActive) || (sgn(error) != sgn(lastError))) *integral = 0;
  else *integral += error;

  const double derivative = error - lastError;

  return KP*error + KI*(*integral) + KD*derivative;
}

/* Updates voltage given the slewProfile */
void Drive::adjustForSlew(int *voltage, double actualVelocity, struct slewProfile *profile)
{
  if(profile->slew && (fabs(actualVelocity) > profile->slew_lower_thresh) && (fabs(actualVelocity) < profile->slew_upper_thresh))
  {
    double deltaVelocity = voltageToVelocity(*voltage) - actualVelocity;
    if(fabs(deltaVelocity) > profile->slew)
    {
      *voltage = velocityToVoltage(actualVelocity + (profile->slew * sgn(deltaVelocity)));
    }
  }
}

/* Update standstillCounter, return the new value for standstill */
bool Drive::updateStandstill(unsigned short *standstillCounter, double error, double lastError, double errorThreshold)
{
  /* standstillExitCount being 0 indicates standstill should not be calculated */
  if(standstillExitCount)
  {
    /* Increment the counter and maybe return true if derivative is low enough, otherwise reset it to 0 */
    if(fabs(error-lastError) <= errorThreshold)
    {
      (*standstillCounter)++;
      return *standstillCounter > standstillExitCount;
    }
    else *standstillCounter = 0;
  }
  return false;
}


/* Basic linear PID movement function */
double Drive::move(PID_dir dir, double target, double timeOut, double maxVelocity){
  /* Error values */
  double lastError;
  double errorDrift;
  double proportionDrift;
  const double initialHeading = imu->get_heading();
  const double initialMotorAvg = driveAvgPos();
  const double tickTarget = inchToTick(target);
  /* Scheduling variables */
  bool scheduled = (scheduleThreshold_l == NO_SCHEDULING);
  double myKP = kP, myKI = kI, myKD = kD;
  /* Integral declaration */
  double integral = 0;
  /* Motor output variable declarations */
  setMaxVoltage(maxVelocity);
  int finalVolt;
  /* Drive output multiplier */
  const int8_t reverseVal = (dir == backward)?(-1):(1);
  /* Standstill variable declarations */
  unsigned short standstillCounter = 0;
  bool standstill = false;
  /* Tell the onError task that a new PID has begun, and set endTime */
  runningPID = true;
  const uint32_t endTime = pros::millis() + timeOut*1000;

  /* Begin PID */
  while(pros::millis() < endTime && !standstill)
  {
    /* Maybe schedule constants */
    if(!scheduled && fabs(error) < scheduleThreshold_l)
    {
      myKP = scheduledConstants.kP;
      myKI = scheduledConstants.kI;
      myKD = scheduledConstants.kD;
      scheduled = true;
    }

    /* Update error, do PID calculations, adjust for slew, and clamp the resulting value */
    error = tickTarget - fabs(driveAvgPos() - initialMotorAvg);
    finalVolt = update(myKP, myKI, myKD, error, lastError, &integral, integralActive);
    adjustForSlew(&finalVolt, actualVelocityAll(), &slewProf);
    finalVolt = std::clamp(finalVolt, -maxVolt, maxVolt);

    /* Print statement used for testing */
    controller.print(2, 0, "Error: %.4f", tickToInch(error));

    //controller.print(2, 0, "Final Volt: %.4f", finalVolt);
    
    /* Calculate standstill */
    standstill = updateStandstill(&standstillCounter, error, lastError, STANDSTILL_DISTANCE_THRESHOLD);
    
    /* Update lastError */
    lastError = error;
  
    /* Calculate the product of heading drift and kP_d */
    errorDrift = fmod((initialHeading-(imu->get_heading())+540),360) - 180;
    proportionDrift = errorDrift * kP_d;

    /* Move Drivetrain */
    moveRightDriveVoltage((reverseVal * finalVolt) + proportionDrift);
    moveLeftDriveVoltage((reverseVal * finalVolt) - proportionDrift);
    
    /* Give PROS time to keep itself in order */
    pros::delay(MS_DELTA_TIME);
  }
  /* Tell the onError task that the PID is over, then return the error at time of exit */
  moveDriveVoltage(0);
  runningPID = false;
  return tickToInch(error);
}


/* Basic angular PID movement function */
double Drive::turn(PID_dir dir, double target, double timeOut, double maxVelocity)
{
  /* Error values */
  double lastError;
  const double initialAngle = imu->get_rotation() + 360; 
  /* Scheduling variables */
  bool scheduled = (scheduleThreshold_a == NO_SCHEDULING);
  double myKP = kP_a, myKI = kI_a, myKD = kD_a;
  /* Integral declarations */
  double integral = 0;
  /* Motor output variable declarations */
  setMaxTurnVoltage(maxVelocity);
  int finalVolt;
  /* Drive output multiplier */
  int8_t reverseVal = (dir == right)?(1):(-1);
  /* Standstill variable declarations */
  unsigned short standstillCounter = 0;
  bool standstill = false;

  /* Change the reverseVal and target if the direction input is shortest */
  if(dir == shortest)
  {
    target = fabs(fmod((target-imu->get_heading()+540),360) - 180);
    reverseVal = sgn(target);
    target = fabs(target);
  }
  
  /* Tell the onError task that a new PID has begun, and set endTime */
  runningPID = true;
  const uint32_t endTime = pros::millis() + timeOut*1000;

  /* Begin PID */
  while((pros::millis() < endTime && !standstill))
  {
    /* Maybe schedule constants */
    if(!scheduled && fabs(error) < scheduleThreshold_a)
    {
      myKP = scheduledConstants.kP_a;
      myKI = scheduledConstants.kI_a;
      myKD = scheduledConstants.kD_a;
      scheduled = true;
    }

    /* Update error, do PID calculations, adjust for slew, and clamp the resulting value */
    error = target - fabs(imu->get_rotation() + 360 - initialAngle);
    finalVolt = update(myKP, myKI, myKD, error, lastError, &integral, integralActive_a);
    //adjustForSlew(&finalVolt, actualVelocityLeft() - actualVelocityRight(), &slewProf_a);
    finalVolt = std::clamp(finalVolt, -maxVolt_a, maxVolt_a);

    /* Print statement used for testing */
    controller.print(2, 0, "Error: %.4f", error);
    //controller.print(2, 0, "Final Volt: %.4f", finalVolt);
    
    /* Calculate standstill */
    standstill = updateStandstill(&standstillCounter, error, lastError, STANDSTILL_TURN_THRESHOLD);
    
    /* Update lastError */
    lastError = error;

    /* Move Drivetrain */
    moveRightDriveVoltage((-reverseVal * finalVolt));
    moveLeftDriveVoltage((reverseVal * finalVolt));
    
    /* Give PROS time to keep itself in order */
    pros::delay(MS_DELTA_TIME);
  }

  /* Tell the onError task that the PID is over, then return the error at time of exit */
  moveDriveVoltage(0);
  runningPID = false;
  return error;
}


/* Swerve movement */
double Drive::swerve(PID_dir dir, double target, double target_a, double timeOut, double maxVel, double maxVel_a){
  /* Error values */
  double error_a;
  double lastError;
  double lastError_a;
  const double initialMotorAvg = driveAvgPos();
  const double tickTarget = inchToTick(target);
  const double initialAngle = imu->get_rotation() + 360;

  /* Scheduling variables */
  bool scheduled_l = (scheduleThreshold_l == NO_SCHEDULING);
  bool scheduled_a = (scheduleThreshold_a == NO_SCHEDULING);
  double myKP_l = kP, myKI_l = kI, myKD_l = kD;
  double myKP_a = kP_a, myKI_a = kI_a, myKD_a = kD_a;
  /* Integral declarations */
  double integral = 0;
  double integral_a = 0;
  /* Motor output variable declarations */
  setMaxVoltage(maxVel);
  setMaxTurnVoltage(maxVel_a);
  int workingVolt;
  int finalVoltLeft;
  int finalVoltRight;
  /* Drive output multipliers */
  const int8_t reverseVal = (dir == backwardLeft || dir == backwardRight || dir == backwardShortest)?(-1):(1);
  int8_t reverseVal_a = (dir == backwardRight || dir == forwardRight)?(1):(-1);

  /* Change the reverseVal and target if the direction input is shortest */
  if(dir == forwardShortest || dir == backwardShortest)
  {
    target = fabs(fmod((target-imu->get_heading()+540),360) - 180);
    reverseVal_a = sgn(target);
    target_a = fabs(target_a);
  }

  /* Standstill variable declarations */
  unsigned short standstillCounter = 0;
  unsigned short standstillCounter_a = 0;
  bool standstill = false;
  bool standstill_a = false;

  /* Tell the onError task that a new PID has begun, and set endTime */
  runningPID = true;
  const uint32_t endTime = pros::millis() + timeOut*1000;

  /* Begin PID */
  while(pros::millis() < endTime && !(standstill && standstill_a))
  {
    /********************************************DRIVE***************************************************/
    /* Maybe schedule linear constants */
    if(!scheduled_l && fabs(error) < scheduleThreshold_l)
    {
      myKP_l = scheduledConstants.kP;
      myKI_l = scheduledConstants.kI;
      myKD_l = scheduledConstants.kD;
      scheduled_l = true;
    }

    /* Update error, do PID calculations, adjust for slew, and clamp the resulting value */
    error = tickTarget - fabs(driveAvgPos() - initialMotorAvg);
    workingVolt = update(myKP_l, myKI_l, myKD_l, error, lastError, &integral, integralActive);
    adjustForSlew(&workingVolt, actualVelocityAll(), &slewProf);
    workingVolt = std::clamp(workingVolt, -maxVolt, maxVolt);

    /* Calculate standstill */
    standstill = updateStandstill(&standstillCounter, error, lastError, STANDSTILL_DISTANCE_THRESHOLD);
    
    /* Update lastError */
    lastError = error;
  
    finalVoltRight = reverseVal * workingVolt;
    finalVoltLeft = reverseVal * workingVolt;
    /********************************************TURN****************************************************/
    /* Maybe schedule angular constants */
    if(!scheduled_a && fabs(error_a) < scheduleThreshold_a)
    {
      myKP_a = scheduledConstants.kP_a;
      myKI_a = scheduledConstants.kI_a;
      myKD_a = scheduledConstants.kD_a;
      scheduled_a = true;
    }

    /* Update error, do PID calculations, adjust for slew, and clamp the resulting value */
    error_a = target - fabs(imu->get_rotation() + 360 - initialAngle);
    workingVolt = update(myKP_a, myKI_a, myKD_a, error_a, lastError_a, &integral_a, integralActive_a);
    adjustForSlew(&workingVolt, actualVelocityLeft() - actualVelocityRight(), &slewProf_a);
    workingVolt = std::clamp(workingVolt, -maxVolt_a, maxVolt_a);

    /* Calculate standstill */
    standstill_a = updateStandstill(&standstillCounter_a, error_a, lastError_a, STANDSTILL_TURN_THRESHOLD);
    
    /* Update lastError */
    lastError_a = error_a;
    
    finalVoltRight += reverseVal_a * (-workingVolt);
    finalVoltLeft += reverseVal_a * (workingVolt);
    /********************************************MOVE****************************************************/
    controller.print(2,0,"%.2f, %.2f            ", tickToInch(error), error_a);

    /* Move Drivetrain */
    moveRightDriveVoltage(finalVoltRight);
    moveLeftDriveVoltage(finalVoltLeft);

    /* Give PROS time to keep itself in order */
    pros::delay(MS_DELTA_TIME);
  }
  
  /* Tell the onError task that the PID is over, then return the error at time of exit */
  moveDriveVoltage(0);
  runningPID = false;
  return tickToInch(error);
}


/* Movement that exits immediately upon reaching targetCutOff */
double Drive::hardStop(PID_dir dir, double targetCutOff, double target, double timeOut, double maxVelocity)
{
  /* Error values */
  double lastError;
  double errorDrift;
  double proportionDrift;
  const double initialHeading = imu->get_heading();
  const double initialMotorAvg = driveAvgPos();
  const double tickTarget = inchToTick(target);
  const double tickTargetCutOff = inchToTick(targetCutOff);
  double integral = 0;
  /* Motor output variable declarations */
  setMaxVoltage(maxVelocity);
  int finalVolt;
  /* Drive output multiplier */
  const int8_t reverseVal = (dir == backward)?(-1):(1);
  /* Tell the onError task that a new PID has begun, and set endTime */
  runningPID = true;
  const uint32_t endTime = pros::millis() + timeOut*1000;

  /* Begin PID */
  while(tickTargetCutOff > fabs(driveAvgPos()-initialMotorAvg) && (pros::millis() < endTime))
  {
    /* Update error, do PID calculations, adjust for slew, and clamp the resulting value */
    error = tickTarget - fabs(driveAvgPos() - initialMotorAvg);
    finalVolt = update(kP, 0, kD, error, lastError, &integral, 0);
    adjustForSlew(&finalVolt, actualVelocityAll(), &slewProf);
    finalVolt = std::clamp(finalVolt, -maxVolt, maxVolt);

    /* Update lastError */
    lastError = error;
  
    /* Calculate the product of heading drift and kP_d */
    errorDrift = fmod((initialHeading-(imu->get_heading())+540),360) - 180;
    proportionDrift = errorDrift * kP_d;

    /* Move Drivetrain */
    moveRightDriveVoltage((reverseVal * finalVolt) + proportionDrift);
    moveLeftDriveVoltage((reverseVal * finalVolt) - proportionDrift);
    
    /* Give PROS time to keep itself in order */
    pros::delay(MS_DELTA_TIME);
  }

  /* Tell the onError task that the PID is over, then return the error at time of exit */
  moveDriveVoltage(0);
  runningPID = false;
  return tickToInch(error);
}

/*CONSTRUCTOR*/
Drive::Drive(pros::MotorGroup &leftMotors, pros::MotorGroup &rightMotors, pros::Imu &imu){
 setPID(1);
 setScheduleThreshold_l(NO_SCHEDULING);
 setScheduleThreshold_a(NO_SCHEDULING);

 setSlew({0, 0, 0});
 setSlew_a({0, 0, 0});

 this->leftMotors      = &leftMotors;
 this->rightMotors     = &rightMotors;
 this->imu             = &imu;

 setStandstillExit(DEFAULT_STANDSTILL_EXIT);
}

/*********************************************************************************************************/
/*DRIVE METHODS*/

double Drive::leftDriveAvgPos(){
  double value = 0;
  for (int i = 0; i<(leftMotors->size()); i++) {
    value += this->leftMotors->get_position(i);
  }
  return value/leftMotors->size();
}

double Drive::rightDriveAvgPos(){
   double value = 0;
   for (int i = 0; i<(rightMotors->size()); i++) 
   {
    value += this->rightMotors->get_position(i);
   }
  return value/rightMotors->size();
}

double Drive::driveAvgPos(){
 return (leftDriveAvgPos()+rightDriveAvgPos())/2;
}

double Drive::actualVelocityLeft(){
 double value = 0;
 for (int i = 0; i<(leftMotors->size()); i++) {
    value += this->leftMotors->get_actual_velocity(i);
 }
 return value/leftMotors->size();
}

double Drive::actualVelocityRight(){
   double value = 0;
   for (int i = 0; i<rightMotors->size(); i++) {
    value += this->rightMotors->get_actual_velocity(i);
  }
  return value/rightMotors->size();
}

double Drive::actualVelocityAll(){
   return (actualVelocityLeft()+actualVelocityRight())/2;
}

void Drive::moveLeftDriveVoltage(int voltage){
  leftMotors->move_voltage(voltage);
}

void Drive::moveRightDriveVoltage(int voltage){
  rightMotors->move_voltage(voltage);
}

void Drive::moveDriveVoltage(int voltage){
  moveLeftDriveVoltage(voltage);
  moveRightDriveVoltage(voltage);
}

void Drive::moveDriveTrain(int voltage, float time){
  moveDriveVoltage(voltage);
  pros::delay(time*1000);
  moveDriveVoltage(0);
}

void Drive::setBrakeMode(pros::motor_brake_mode_e brakeMode){
  leftMotors->set_brake_mode_all(brakeMode);
  rightMotors->set_brake_mode_all(brakeMode);
}