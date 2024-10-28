#include "drive.hpp"
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