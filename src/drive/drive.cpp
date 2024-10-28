#include "drive.hpp"
#include "util.hpp"
#include "include.hpp"
#include <optional>

#define STANDSTILL_DISTANCE_THRESHOLD 2
#define STANDSTILL_TURN_THRESHOLD 0.2

#define MS_DELTA_TIME 20

/* Basic linear PID movement function */
double Drive::move(PID_dir dir, double target, double timeOut, double maxVelocity, Triangle tri){
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

  /*motion chaining and early exit variables*/
  double prevOut;
  bool close = false;
  const bool side = false;
  std::optional<bool> prevSide = std::nullopt;

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
    prevOut = finalVolt;

    /* Print statement used for testing */
    controller.print(2, 0, "Error: %.4f", tickToInch(error));

    //controller.print(2, 0, "Final Volt: %.4f", finalVolt);
    
    /* Calculate standstill */
    standstill = updateStandstill(&standstillCounter, error, lastError, STANDSTILL_DISTANCE_THRESHOLD);
    
    /* update distance traveled */
    double distTraveled = target-lastError; //gotta think about this 

    /* Update lastError */
    lastError = error;
    
    /* check if the robot is close enough to the target to start settling */
    if (target < 7.5 && close == false) {
        close = true;
        maxVolt = fmax(fabs(prevOut), 60);
    }

    // motion chaining
    const bool side =
              (tri.a - tri.b) * -sin(tri.alpha) <= (tri.a - tri.b) * cos(tri.alpha) + moveParams.earlyExitRange;
    if (prevSide == std::nullopt) prevSide = side;
    const bool sameSide = side == prevSide;
    // exit if close
    if (!sameSide && moveParams.minSpeed != 0) break;
    prevSide = side;
  
    /* Calculate the product of heading drift and kP_d */
    errorDrift = fmod((initialHeading-(imu->get_heading())+540),360) - 180;
    proportionDrift = errorDrift * kP_d;

    /* Move Drivetrain */
    moveRightDriveVoltage((reverseVal * finalVolt) - proportionDrift);
    moveLeftDriveVoltage((reverseVal * finalVolt) + proportionDrift);
    
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
  if(dir == shortest){
    target = fabs(fmod((target-imu->get_heading()+540),360) - 180);
    reverseVal = sgn(target);
    target = fabs(target);
  }
  
  /* Tell the onError task that a new PID has begun, and set endTime */
  runningPID = true;
  const uint32_t endTime = pros::millis() + timeOut*1000;

  /* Begin PID */
  while((pros::millis() < endTime && !standstill)){
    /* Maybe schedule constants */
    if(!scheduled && fabs(error) < scheduleThreshold_a){
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
    //controller.print(2, 0, "Sche. kP_A: %.4f", scheduledConstants.kP_a);
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

/*********************************************************************************************************/

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

