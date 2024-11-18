#include "drive.hpp"
/* Set PID constants */
void Drive::setPID(char n){
  n--;
  kP = PIDConstants[n].kP;
  kP_a = PIDConstants[n].kP_a;
  kI = PIDConstants[n].kI;
  kI_a = PIDConstants[n].kI_a;
  kD = PIDConstants[n].kD;
  kD_a = PIDConstants[n].kD_a;
  kP_d = PIDConstants[n].kP_d;
}


void Drive::setScheduledConstants(char n){
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
void Drive::setCustomPID(PIDprofile profile){
  kP = profile.kP;
  kP_a = profile.kP_a;
  kI = profile.kI; 
  kI_a = profile.kI_a;
  kD = profile.kD;
  kD_a = profile.kD_a;
  kP_d = profile.kP_d;
}

void Drive::setScheduleThreshold_l(double error){
  scheduleThreshold_l = error;
}


void Drive::setScheduleThreshold_a(double error){
  scheduleThreshold_a = error;
}

/* Set max linear velocity */
void Drive::setMaxVoltage(double pctVoltage){
  maxVolt = percentToVoltage(pctVoltage);
}

/* Set max angular voltage by percent */
void Drive::setMaxTurnVoltage(double pctVoltage){
  maxVolt_a = percentToVoltage(pctVoltage);
}


/* Update slewProf given profile */
void Drive::setSlew(slewProfile profile){
  slewProf = profile;
}


/* Update slewProf given profile */
void Drive::setSlew_a(slewProfile profile){
  slewProf_a = profile;
}

/* Set standstillExitCount, will not execute standstill if 0 */
void Drive::setStandstillExit(int exitOn){
  standstillExitCount = exitOn;
}