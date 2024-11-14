#include "drive.hpp"
#include "include.hpp"
#include "pros/misc.h"
#include "pros/rtos.hpp"
#include "autons.hpp"
#include "intake.hpp"
#include "util.hpp"
#include "arm.hpp"
#include <optional>

/* Create an array of auton wrappers to be used with the auton-selector*/
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
slewProfile genSlewProfile{85, 30, 80};
slewProfile mogoSlewProfile{70, 30, 70};
IntakeControl conveyor;

void winPointRed(){
 pros::Task runOnError(onError_fn); 
 pros::Task runIntakeControl(IntakeControlSystem_fn);
 Triangle tri;

 clampPis.set_value(true); //pull the clamp up 
 
 //score preload
 drive.move(backward, 15, 1, 100);

 drive.turn(right, imuTarget(90), 1, 70);

 drive.moveDriveVoltage(-12000);
 
 pros::delay(200);
 
 setIntake(400, std::nullopt);
 startIntake();
 pros::delay(800);

 drive.moveDriveVoltage(0);

 stopIntake();

 drive.move(forward, 6, 1, 100);
 
 //go to mogo 
 drive.turn(right, imuTarget(180), 1, 70);

 startIntake();
 
 findTri(&tri, 25, 180);
 drive.move(backward, tri.hyp, 2, 100);
 
 drive.turn(right, imuTarget(270), 1, 70);

 //get mogo
 drive.addErrorFunc(20, LAMBDA(drive.setMaxVoltage(70)));
 drive.move(backward, 22-tri.b, 4, 100);

 clampPis.set_value(false);
 pros::delay(250);
 clampPis.set_value(false);
 
 drive.move(backward, 8-tri.b, 4, 100);
 
 //go get 1st ring
 drive.turn(right, imuTarget(0), 2, 90);
 
 findTri(&tri, 18, 0);
 drive.move(forward, tri.hyp, 2, 70);
 
 //go get 2nd ring
 drive.turn(right, imuTarget(85), 2, 90);

 drive.move(forward, 14-tri.b, 1, 100);
 pros::delay(200);

 //go get 3rd ring
 findTri(&tri, 10, 105);
 drive.move(backward, tri.hyp, 1, 100);
 
 drive.turn(right, imuTarget(180), 2, 90);

 drive.move(forward, 32, 3, 90);
 
 clampPis.set_value(true);
 pros::delay(200);

 runOnError.remove();
 runIntakeControl.remove();
 drive.onErrorVector.clear();

}

void winPointBlue(){
 pros::Task runOnError(onError_fn); 
 pros::Task runIntakeControl(IntakeControlSystem_fn);
 Triangle tri;

 clampPis.set_value(true); //pull the clamp up 

 drive.move(backward, 14, 1, 100);

 drive.turn(left, imuTarget(270), 1, 70);

 drive.moveDriveVoltage(-12000);
 
 pros::delay(200);
 
 setIntake(400, std::nullopt);
 startIntake();
 
 pros::delay(950);

 drive.moveDriveVoltage(0);

 stopIntake();

 drive.move(forward, 6, 1, 100);
 
 //go to mogo 
 drive.turn(left, imuTarget(180), 1, 70);

 startIntake();
 
 findTri(&tri, 24, 180);
 drive.move(backward, tri.hyp, 2, 100);
 
 drive.turn(left, imuTarget(90), 1, 70);

 //get mogo
 drive.addErrorFunc(20, LAMBDA(drive.setMaxVoltage(70)));
 drive.move(backward, 30-tri.b, 4, 100);

 clampPis.set_value(false);
 pros::delay(250);
 clampPis.set_value(false);

 //go get 1st ring
 drive.turn(left, imuTarget(360), 2, 90);
 
 findTri(&tri, 18, 180);
 drive.move(forward, tri.hyp, 2, 70);
 
 //go get 2nd ring
 drive.turn(left, imuTarget(275), 2, 90);

 drive.move(forward, 14-tri.b, 1, 100);
 pros::delay(200);

 //go get 3rd ring
 findTri(&tri, 10, 95);
 drive.move(backward, tri.hyp, 1, 100);
 
 drive.turn(left, imuTarget(180), 2, 90);

 drive.move(forward, 30, 3, 70);
 
 clampPis.set_value(true);
 pros::delay(200);

 runOnError.remove();
 runIntakeControl.remove();
 drive.onErrorVector.clear();
}

void ringSideRed(){
 pros::Task runOnError(onError_fn);
 pros::Task runIntakeControl(IntakeControlSystem_fn);
 Triangle tri;

 stopIntake(); // weird issue starting before start intake is called
 
 //get mogo
 drive.move(backward, 26, 2, 100);

 clampPis.set_value(false);
 pros::delay(250);
 clampPis.set_value(false);

 setIntake(400, std::nullopt);
 startIntake();
 
 //go get 1st ring
 drive.turn(right, imuTarget(90), 2, 90);
 
 findTri(&tri, 27, 90);
 drive.move(forward, tri.hyp, 2, 70);
 
 //go get 2nd ring
 drive.turn(right, imuTarget(195), 2, 90);

 drive.move(forward, 14-tri.b, 1, 100);
 pros::delay(200);

 //go get 3rd ring
 findTri(&tri, 6, 195);
 drive.move(backward, tri.hyp, 1, 100);
 
 drive.setPID(6);
 drive.turn(left, imuTarget(150), 1, 90);

 drive.move(forward, 7-tri.b, 1, 100);
 pros::delay(200);
 
 //go touch bar
 drive.setPID(2);
 drive.move(backward, 10, 1, 100);

 drive.turn(right, imuTarget(270), 2, 90);

 drive.move(forward, 36, 5, 60);
 

 clampPis.set_value(true);
 pros::delay(200);

 
 runOnError.remove();
 runIntakeControl.remove();
 drive.onErrorVector.clear();
}

void ringSideBlue(){
 pros::Task runOnError(onError_fn);
 pros::Task runIntakeControl(IntakeControlSystem_fn);
 Triangle tri;
 
 //get mogo
 drive.move(backward, 26, 2, 100);

 clampPis.set_value(false);
 pros::delay(250);


 setIntake(400, std::nullopt);
 startIntake();
 
 //go get 1st ring
 drive.turn(left, imuTarget(270), 2, 90);
 
 findTri(&tri, 27, 90);
 drive.move(forward, tri.hyp, 2, 70);
 
 //go get 2nd ring
 drive.turn(left, imuTarget(165), 2, 90);

 drive.move(forward, 14-tri.b, 1, 100);
 pros::delay(200);

 //go get 3rd ring
 findTri(&tri, 6, 195);
 drive.move(backward, tri.hyp, 1, 100);
 
 drive.setPID(6);
 drive.turn(right, imuTarget(210), 1, 90);

 drive.move(forward, 7-tri.b, 1, 100);
 pros::delay(200);
 
 //go touch bar
 drive.setPID(2);
 drive.move(backward, 10, 1, 100);

 drive.turn(left, imuTarget(90), 2, 90);

 drive.move(forward, 36, 5, 60);
 
 clampPis.set_value(true);
 pros::delay(200);

 runOnError.remove();
 runIntakeControl.remove();
 drive.onErrorVector.clear();  
}

void goalSideRed(){
 pros::Task runOnError(onError_fn);
 pros::Task runIntakeControl(IntakeControlSystem_fn);
 Triangle tri;

 stopIntake(); // weird issue starting before start intake is called
 
 //get mogo
 drive.move(backward, 28, 2, 100);

 clampPis.set_value(false);
 pros::delay(250);
 clampPis.set_value(false);

 setIntake(400, std::nullopt);
 startIntake();

 //go get 1st ring
 drive.setPID(2);
 drive.turn(left, imuTarget(270), 2, 90);
 
 findTri(&tri, 27, 270);
 drive.move(forward, tri.hyp, 2, 70);
 
 drive.setPID(7);
 drive.turn(left, imuTarget(90), 2, 90);

 drive.move(forward, 39, 3, 50);

 clampPis.set_value(true);
 pros::delay(200);

 runOnError.remove();
 runIntakeControl.remove();
 drive.onErrorVector.clear();
}

void goalSideBlue(){
 pros::Task runOnError(onError_fn);
 pros::Task runIntakeControl(IntakeControlSystem_fn);

 Triangle tri;

 stopIntake(); // weird issue starting before start intake is called
 
 //get mogo
 drive.move(backward, 28, 2, 100);

 clampPis.set_value(false);
 pros::delay(250);
 clampPis.set_value(false);

 setIntake(400, std::nullopt);
 startIntake();

 //go get 1st ring
 drive.setPID(2);
 drive.turn(right, imuTarget(90), 2, 90);
 
 findTri(&tri, 27, 270);
 drive.move(forward, tri.hyp, 2, 70);
 
 drive.setPID(7);
 drive.turn(right, imuTarget(270), 2, 90);

 drive.move(forward, 39, 3, 50);

 clampPis.set_value(true);
 pros::delay(200);

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
 pros::Task intakeControlTask(IntakeControlSystem_fn);
 pros::Task armControlTask(armControl_fn);
 Triangle tri;
 
 pros::delay(20); //dealy before intake can start for some reason (prob task related)

 drive.moveDriveVoltage(-6000);

 setIntake(400, std::nullopt);
 startIntake();

 pros::delay(700); //score on allince stake with preload 
 
 drive.moveDriveVoltage(0);
 stopIntake();
 
 drive.setSlew(genSlewProfile);
 drive.move(forward, 15, 1, 100);

 drive.turn(left, imuTarget(270), 1, 70);
  
                 
                  /*{kP, kPa, kI, kIa, kD,  kDa,  kPd}*/
 drive.setCustomPID({50, 87,  0,  13,  160, 100,  150});
 drive.setSlew({0,35,70});
 drive.move(backward, 17, 1, 50);

 clampPis.set_value(true); //clamp goal one 
 pros::delay(100);
 
 drive.setPID(1);
 drive.move(backward, 5, 1, 100);
 
 drive.setPID(2);
 drive.turn(right, imuTarget(0), 1, 90);

 startIntake();

 drive.setSlew(mogoSlewProfile);
 findTri(&tri, 22, 360);
 drive.move(forward, tri.hyp, 1, 100);
 
 drive.setPID(4);
 drive.turn(right, imuTarget(40), 1, 90);
 
 drive.setPID(5);
 drive.swerve(forwardLeft, 58-tri.b, imuTarget(355), 3, 75, 45); //skills swerve 1 (tripple test/check this )

 pros::delay(1000); // for checking error (remove after testing)

 drive.setPID(2);
 drive.turn(right, imuTarget(155), 2, 90); //turn to ring in front of wall stake 

 armControl.setTarget(load); //prime the wall stake mech
 
 findTri(&tri, 22, 155);
 drive.move(forward, tri.hyp, 1, 100);

 drive.setPID(4);
 drive.turn(left, imuTarget(115), 1, 100);

 drive.setPID(2);
 drive.move(forward, 5, 1, 100);
 //pros::delay(200); //let ring load 

 armControl.setTarget(score);  //score wall stake
 pros::delay(1000);

 drive.move(backward, 12-tri.b, 1, 100);

 drive.setPID(4);
 drive.turn(right, imuTarget(180), 1, 100);
 armControl.setTarget(standby);

 drive.setPID(4);
 findTri(&tri, 49, 180);
 drive.addErrorFunc(tri.hyp-25, LAMBDA(drive.setMaxVoltage(80)));
 drive.move(forward, tri.hyp, 3, 100);

 pros::delay(500); //let first rings get themselves together 

 drive.move(forward, 22-tri.b, 1, 100);

 drive.setPID(2);
 drive.turn(left, imuTarget(67), 2, 90); //get out of line ring 

 drive.move(forward, 13-tri.b, 1, 100);

 drive.turn(left, imuTarget(322), 2, 90);

 findTri(&tri, 12, 320);
 drive.move(backward, tri.hyp, 1, 100);

 clampPis.set_value(false); //drop off 1st mogo 
 pros::delay(150);
 
 drive.setPID(1);
 drive.setSlew(genSlewProfile);

 drive.move(forward, 15-tri.b, 1, 100);

 drive.turn(right, imuTarget(90), 1, 70); //go get 2nd mogo 

 findTri(&tri, 63, 90);
 drive.setPID(4);
 drive.setSlew({0,0,0});
 drive.move(backward, tri.hyp, 3, 100); 

 clampPis.set_value(true); //second mogo 
 pros::delay(100);

 drive.setPID(2);
 drive.setSlew(mogoSlewProfile);
 drive.move(backward, 5-tri.b, 3, 100);

 drive.turn(left, imuTarget(0), 1, 90);

 findTri(&tri, 25, 360);
 drive.move(forward, tri.hyp, 1, 100);
 
 drive.setPID(4);
 drive.turn(right, imuTarget(33), 1, 100);

 drive.move(forward, 30-tri.b, 2, 100); //get ring under elevation tower 


 runOnError.remove();
 intakeControlTask.remove();
 armControlTask.remove();
 drive.onErrorVector.clear();

}

void tune(){
 pros::Task runOnError(onError_fn);
 
//  drive.setPID(5);
//  drive.setSlew(mogoSlewProfile);
//  drive.swerve(forwardLeft, 66, 17, 3, 100, 70); //skills swerve 1 
//  pros::delay(2000);

//  Triangle tri;
//  findTri(&tri, 72, 90);
//  drive.setPID(4);
//  drive.move(backward, 72, 7, 100); //very slow
 
                  /*{kP, kPa, kI, kIa, kD,  kDa,  kPd}*/
 drive.setCustomPID({50, 87,  0,  13,  160, 100,  150});
 drive.setSlew({0,35,70});
 drive.move(backward, 19, 1, 50);
 pros::delay(1000);
 /* drive.setPID(4);
 drive.setSlew(mogoSlewProfile);
 
 drive.move(forward, 50, 3, 100);
 pros::delay(1000);

 drive.move(forward, 75, 3, 100);
 pros::delay(1000); */
 
 
  // drive.turn(right, 20, 1, 100);
  // pros::delay(1000);

  // drive.turn(right, 25, 1, 100);
  // pros::delay(1000);
  
  // drive.turn(right, 40, 1, 100);
  // pros::delay(1000);

  // drive.turn(right, 55, 1, 100);
  // pros::delay(1000);
  
  // drive.turn(right, 70, 1, 100);
  // pros::delay(1000);
  
  
//  drive.turn(right, 60, 1, 70);
//  pros::delay(1000);
                     
//  drive.turn(right, 65, 1, 70);
//  pros::delay(1000);

//  drive.turn(right, 80, 1, 70);
//  pros::delay(1000);

//   drive.turn(right, 95, 1, 70);
//   pros::delay(1000);

//   drive.turn(right, 115, 2, 70);
//   pros::delay(1000);

//   drive.turn(right, 130, 2, 70);
//   pros::delay(1000);

//   drive.turn(right, 145, 2, 70);
//   pros::delay(1000);

//   drive.turn(right, 160, 2, 70);
//   pros::delay(1000);

//   drive.turn(right, 175, 2, 70);
//   pros::delay(1000);


 runOnError.remove();
 drive.onErrorVector.clear();
}
