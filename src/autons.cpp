#include "drive.hpp"
#include "include.hpp"
#include "pros/rtos.hpp"
#include "autons.hpp"
#include "intake.hpp"
/* Create an array of auton wrappers  to be used with the auton-selector*/
autonTextTuple autos[AUTO_COUNT] = {
  {"WPR", winPointRed},
  {"WPB", winPointBlue},

  {"RSR",ringSideRed},
  {"RSB",ringSideBlue},

  {"GSR",goalSideRed},
  {"GSB",goalSideBlue},

  {"RER",ringElimRed},
  {"REB",ringElimBlue},

  {"GER",goalElimRed},
  {"GEB",goalElimBlue},

  {"Skills",skills},
  {"tune",tune}
};

Drive drive(leftMotors, rightMotors, imu);
slewProfile mogoProfile{90, 30, 70};
IntakeControl conveyor;

//weird profiles for reuse 
PIDprofile deg90RingConsts{ 0,   250,  0,   0,   0,  200,    0};
PIDprofile deg90RingScheduledConsts{ 0,  190,  0,  15,   0,  600,  0};
PIDprofile deg60RingConsts{ 0,   200,  0,   0,   0,  200,    0};
PIDprofile deg60RingSchedulesConsts{0,  125,  0,  15,   0,  320,  0};
PIDprofile deg110RingConsts{ 0,   250,  0,   0,   0,  200,    0};
PIDprofile deg110RingScheduledConsts{ 0,  190,  0,  15,   0,  600,  0};

void winPointRed(){
 pros::Task runOnError(onError_fn);
 pros::Task runIntakeControl(IntakeControlSystem_fn);

 runOnError.remove();
 runIntakeControl.remove();
 drive.onErrorVector.clear();
}

void winPointBlue(){
 pros::Task runOnError(onError_fn);
 pros::Task runIntakeControl(IntakeControlSystem_fn);

 runOnError.remove();
 runIntakeControl.remove();
 drive.onErrorVector.clear();
}

void ringSideRed(){
 pros::Task runOnError(onError_fn);
 pros::Task runIntakeControl(IntakeControlSystem_fn);

 runOnError.remove();
 runIntakeControl.remove();
 drive.onErrorVector.clear();
}

void ringSideBlue(){
 pros::Task runOnError(onError_fn);
 pros::Task runIntakeControl(IntakeControlSystem_fn);

 runOnError.remove();
 runIntakeControl.remove();
 drive.onErrorVector.clear();  
}

void goalSideRed(){
  pros::Task runOnError(onError_fn);
 pros::Task runIntakeControl(IntakeControlSystem_fn);

 runOnError.remove();
 runIntakeControl.remove();
 drive.onErrorVector.clear();
}

void goalSideBlue(){
 pros::Task runOnError(onError_fn);
 pros::Task runIntakeControl(IntakeControlSystem_fn);

 runOnError.remove();
 runIntakeControl.remove();
 drive.onErrorVector.clear();
}

void ringElimRed(){
 pros::Task runOnError(onError_fn);
 pros::Task runIntakeControl(IntakeControlSystem_fn);

 runOnError.remove();
 runIntakeControl.remove();
 drive.onErrorVector.clear();
}

void ringElimBlue(){
 pros::Task runOnError(onError_fn);
 pros::Task runIntakeControl(IntakeControlSystem_fn);

 runOnError.remove();
 runIntakeControl.remove();
 drive.onErrorVector.clear();
}

void goalElimRed(){
 pros::Task runOnError(onError_fn);
 pros::Task runIntakeControl(IntakeControlSystem_fn);

 runOnError.remove();
 runIntakeControl.remove();
 drive.onErrorVector.clear();
}

void goalElimBlue(){
 pros::Task runOnError(onError_fn);
 pros::Task runIntakeControl(IntakeControlSystem_fn);

 runOnError.remove();
 runIntakeControl.remove();
 drive.onErrorVector.clear();
}
           
void skills(){
 pros::Task runOnError(onError_fn);
 pros::Task runIntakeControl(IntakeControlSystem_fn);

 runOnError.remove();
 runIntakeControl.remove();
 drive.onErrorVector.clear();
}

void tune(){
 pros::Task runOnError(onError_fn);
 /*
 drive.setScheduleThreshold_l(10);
 drive.setPID(4);
 drive.setScheduledConstants(PIDConstants[5]);
 drive.setScheduleThreshold_a(20);

 drive.setSlew(mogoProfile);
 */
  //  drive.setPID(8);
  //  drive.setScheduleThresholds_s(0, 10);
  //  drive.setScheduledSwerveConstants(PIDConstants[6]);
  
  //  //170 degree angular movememt
  //  drive.swerve(forwardRight, 52, 40, 3, 60, 10);
  //  pros::delay(1000);
 drive.setPID(1);
 drive.setScheduledConstants(PIDConstants[4]);
 drive.setScheduleThreshold_a(15);
 drive.setScheduleThreshold_l(10);

 drive.turn(right, 158, 2, 70);
 pros::delay(2000);
 /*
 drive.move(forward, 22, 1, 100);
 pros::delay(1000);
 drive.move(forward, 32, 2, 100);
 pros::delay(1000);
 drive.move(forward, 42, 2, 100);
 pros::delay(2000);
 
 
 
 //  drive.turn(right, 50, 1, 70);
 //  pros::delay(1000);

 drive.turn(right, 60, 1, 70);
 pros::delay(1000);

 drive.turn(right, 75, 1, 70);
 pros::delay(1000);

 drive.turn(right, 80, 1, 70);
 pros::delay(1000);

 drive.turn(right, 100, 1, 70);
 pros::delay(1000);

 drive.turn(right, 120, 2, 70);
 pros::delay(1000);
 
 drive.turn(right, 135, 2, 70);
 pros::delay(1000);


 drive.turn(right, 150, 2, 70);
 pros::delay(1000);

 drive.turn(right, 175, 2, 70);
 pros::delay(1000);

 drive.turn(right, 180, 2, 70);
 pros::delay(1000);

 drive.turn(right, 195, 2, 70);
 pros::delay(1000);

 drive.turn(right, 205, 2, 70);
 pros::delay(1000);
    */

 runOnError.remove();
 drive.onErrorVector.clear();
}