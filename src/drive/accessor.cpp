#include "drive.hpp"

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
