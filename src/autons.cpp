#include "drive.hpp"
#include "include.hpp"
#include "pros/rtos.hpp"
#include "autons.hpp"
#include "intake.hpp"
#include "util.hpp"
/* Create an array of auton wrappers  to be used with the auton-selector*/
autonTextTuple autos[AUTO_COUNT] = {
  {"WinP R", winPointRed},
  {"WinP B", winPointBlue},

  {"RingS R",ringSideRed},
  {"RingS B",ringSideBlue},

  {"GoalsS R",goalSideRed},
  {"GoalS B",goalSideBlue},

  {"RignE R",ringElimRed},
  {"RingE B",ringElimBlue},

  {"GoalE R",goalElimRed},
  {"GoalE B",goalElimBlue},

  {"Skills",skills},
  {"Tune",tune}
};

Drive drive(leftMotors, rightMotors, imu);
slewProfile mogoProfile{90, 30, 70};
IntakeControl conveyor;

//weird profiles for reuse 
              /*{kP, kPa, kI, kIa, kD,  kDa,  kPd}*/
//drive.setCustomPID({14, 157,  0,  0,  30,   56,  0}); //30 deg turn

void winPointRed(){
 pros::Task runOnError(onError_fn);

 runOnError.remove();
 drive.onErrorVector.clear();
}

void winPointBlue(){
 pros::Task runOnError(onError_fn);
 clampPis.set_value(true); //pull the clamp up 

 drive.move(backward, 22, 1, 100);

 clampPis.set_value(false);
 pros::delay(250);
 tiltPis.set_value(true);
 pros::delay(200);
 intake.move_voltage(12000);
 pros::delay(300);

 drive.setPID(2);
 drive.setScheduledConstants(5);
 drive.turn(left, imuTarget(257), 1, 70);

 drive.addErrorFunc(10, LAMBDA(drive.setMaxVoltage(40))); 
 drive.move(forward, 29, 2, 100);

                  /*{kP, kPa, kI, kIa, kD,  kDa,  kPd}*/
 drive.setCustomPID({0,  77,  0,  0,    0,  82,  0}), /*70-100 degree mogo turns / gen mogo lat*/ 
 drive.turn(right, imuTarget(66), 2, 70);
 pros::delay(1000); //bring down to 300ms

 clampPis.set_value(true);
 pros::delay(300);
 tiltPis.set_value(false);
 pros::delay(300);
 
 drive.setPID(1);
 drive.move(forward, 41, 2, 100);
 
 intakePis.set_value(true);
 pros::delay(500);
 
 drive.addErrorFunc(4, LAMBDA(intakePis.set_value(false)));
 drive.move(forward, 12, 3, 60);

 intake.move_voltage(0);
 drive.turn(right, imuTarget(180), 2, 70);

 drive.move(backward, 22, 1, 100);

 intake.move_voltage(12000);

 pros::delay(1200);

 drive.swerve(forwardLeft, 30, imuTarget(180), 3, 90, 50);

 runOnError.remove();
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
 Triangle tri;

 setIntake(400);
 startIntake();
 clampPis.set_value(true); // Pull the clamp up 

 pros::delay(100);

 drive.moveDriveVoltage(-6000);
 pros::delay(1500);
 drive.moveDriveVoltage(0);

 drive.move(forward, 13, 1, 100);

 drive.turn(right, imuTarget(90), 1, 70);

 drive.move(backward, 17, 1, 100);

 clampPis.set_value(false);
 pros::delay(250);
 clampPis.set_value(false);
 tiltPis.set_value(true);
 pros::delay(200);
 
 // Get rings
 drive.move(backward, 9, 1, 100);
 
 drive.setPID(2);
 drive.turn(left, imuTarget(0), 1, 90);
 
 drive.move(forward, 24, 1, 100);

 drive.turn(left, imuTarget(270), 1, 90);

 drive.move(forward, 24, 1, 100);
 
 //go to ring in front of wall stake 
 drive.setPID(2);
 drive.turn(right, imuTarget(330), 1, 90);

 findTri(&tri, 17, 330);

 drive.move(forward, tri.hyp, 1, 100);

 drive.move(backward, tri.hyp,  1, 100);

 drive.turn(left, imuTarget(212), 1, 100);
 
 //out of line ring
 findTri(&tri, 20, 212);

 drive.move(forward, tri.hyp, 1, 100);

 drive.move(backward, tri.hyp+2, 1, 100); 
 
 //get last two rings onto 1st mogo
 //needs tuned
 drive.setPID(4);  
 drive.turn(left, imuTarget(180), 1, 70);

 drive.move(forward, 22, 1, 90);
 pros::delay(500);

 drive.move(forward, 12, 1, 100);
 pros::delay(400);
  
 //drop off 1st mogo
 drive.setPID(2);
 drive.turn(left, imuTarget(50), 1, 100);
 findTri(&tri, 10, 50);
 drive.move(backward, tri.hyp, 1, 100);

 clampPis.set_value(true);
 pros::delay(300);
 tiltPis.set_value(false);
 pros::delay(300);
 
 //Go to 2nd mogo 
 drive.move(forward, 22-tri.b, 1, 100);

                  /*{kP, kPa, kI, kIa, kD,  kDa,  kPd}*/
 drive.setCustomPID({14, 82,  0,  6,  30,   50,  0});
 drive.turn(left, imuTarget(270), 2, 100); //156 degree turn with custom prof cause needs to be less than 1 error everytime
 
 drive.move(backward, 49-tri.b, 10, 80);

 pros::delay(300); //let the mogo settle if its shaking
 
 clampPis.set_value(false);
 pros::delay(250);
 tiltPis.set_value(true);
 pros::delay(200);
 
 drive.setPID(2);
 findTri(&tri, 7, 270);
 drive.move(backward, tri.hyp, 1, 100);
 
 //Get offset ring
 drive.turn(left, imuTarget(115), 1, 90);
 
 //rings line
 drive.move(forward, 24-tri.b, 1, 100);
 pros::delay(100);

 drive.move(backward, 25-tri.b, 1, 100);

 //22 deg turn needs tuned 
 drive.setPID(4);
 drive.turn(left, imuTarget(90), 1, 70);

 drive.move(forward, 16, 1, 100);
 pros::delay(300);

 drive.move(forward, 16, 1, 100);
 pros::delay(300);

 drive.move(backward, 8, 1, 100);
 
 drive.setPID(3);
 drive.turn(left, imuTarget(0), 1, 90);
 
 drive.move(forward, 16, 3, 80);
 pros::delay(100);
 
 drive.turn(left, imuTarget(270), 1, 90);
 
 drive.move(forward, 34, 2, 100);

 drive.setPID(3);
 drive.turn(right, imuTarget(315), 1, 90);

//  drive.move(forward, 24, 1, 100);

//  drive.move(backward, 72, 3, 100);

//  clampPis.set_value(true);
//  pros::delay(300);
//  tiltPis.set_value(false);
//  pros::delay(300);

 //Go to 1st wall stake


 runOnError.remove();
 runIntakeControl.remove();
 drive.onErrorVector.clear();
}

void tune(){
 pros::Task runOnError(onError_fn);

 //drive.setScheduledConstants(5);
 //drive.setScheduleThreshold_a(15);

 drive.turn(right, 50, 1, 70);
 pros::delay(1000);



 drive.turn(right, 65, 1, 70);
 pros::delay(1000);

 drive.turn(right, 80, 1, 70);
 pros::delay(1000);

 drive.turn(right, 95, 1, 70);
 pros::delay(1000);

 drive.turn(right, 115, 2, 70);
 pros::delay(1000);

 drive.turn(right, 130, 2, 70);
 pros::delay(1000);

 drive.turn(right, 145, 2, 70);
 pros::delay(1000);

 drive.turn(right, 160, 2, 70);
 pros::delay(1000);

 drive.turn(right, 175, 2, 70);
 pros::delay(1000);

 drive.turn(right, 180, 2, 70);
 pros::delay(1000);

 runOnError.remove();
 drive.onErrorVector.clear();
}
