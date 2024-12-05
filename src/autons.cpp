#include "drive.hpp"
#include "include.hpp"
#include "pros/device.hpp"
#include "pros/rtos.hpp"
#include "autons.hpp"
#include "intake.hpp"
#include "util.hpp"
#include "arm.hpp"
#include <optional>

/* Create an array of auton wrappers to be used with the auton-selector*/
autonTextTuple autos[AUTO_COUNT] = {
  {"WinP Red", winPointRed},
  {"WinP Blu", winPointBlue},

  {"RinS Red",ringSideRed},
  {"RinS Blu",ringSideBlue},

  {"GoaS Red",goalSideRed},
  {"GoaS Blu",goalSideBlue},

  {"RinE Red",ringElimRed},
  {"RinE Blu",ringElimBlue},

  {"GoaE Red",goalElimRed},
  {"GoaE Blu",goalElimBlue},

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
 setIntake(400, std::nullopt);

 currentColor = blue;

 drive.move(backward, 14.5, 1, 100);

 drive.turn(right, imuTarget(90), 1, 70);

 drive.move(backward, 4, 1, 100);

 startIntake();
 
 pros::delay(400);
 
 drive.move(forward, 4, 1, 100);
 
 drive.turn(right, imuTarget(180), 1, 70);

 drive.move(backward, 10, 1, 100);

 drive.turn(right, imuTarget(243), 1, 70); //turn to mogo

 drive.addErrorFunc(18, LAMBDA(drive.setMaxVoltage(20)));
 drive.addErrorFunc(2.5, LAMBDA(clampPis.set_value(true)));
 drive.move(backward, 30, 2, 100);
 pros::delay(200); //let clamp lock 
 
 drive.setSlew(mogoSlewProfile);

 drive.setPID(2);
 drive.move(backward, 6, 1, 100);
 
 drive.setPID(4);
 drive.turn(right, imuTarget(0), 1, 90);

 drive.setPID(2);
 drive.move(forward, 22, 2, 80);
 pros::delay(100); //get 1st ring on mogo 
 
 drive.setPID(2);
 drive.turn(right, imuTarget(95), 1, 90);

 setIntake(400, blue);

 drive.move(forward, 13, 1, 100); //get 2nd ring
 pros::delay(150);

 //drive.move(backward, 5, 1, 100);
 
 drive.turn(right, imuTarget(170), 2, 90);

 drive.move(forward, 10, 2, 100); // touch elevation tower 
 
 runOnError.remove();
 runIntakeControl.remove();
 drive.onErrorVector.clear();
}

void winPointBlue(){
 pros::Task runOnError(onError_fn); 
 pros::Task runIntakeControl(IntakeControlSystem_fn);
 Triangle tri;
 setIntake(400, std::nullopt);

 currentColor = red;

 drive.move(backward, 14.5, 1, 100);

 drive.turn(left, imuTarget(270), 1, 70);

 drive.move(backward, 4, 1, 100);

 startIntake();
 
 pros::delay(400);
 
 drive.move(forward, 4, 1, 100);
 
 drive.turn(left, imuTarget(0), 1, 70);

 drive.move(backward, 10, 1, 100);

 drive.turn(left, imuTarget(243), 1, 70); //turn to mogo

 drive.addErrorFunc(18, LAMBDA(drive.setMaxVoltage(20)));
 drive.addErrorFunc(2.5, LAMBDA(clampPis.set_value(true)));
 drive.move(backward, 30, 2, 100);
 pros::delay(200); //let clamp lock 
 
 drive.setSlew(mogoSlewProfile);

 drive.setPID(2);
 drive.move(backward, 6, 1, 100);
 
 drive.setPID(4);
 drive.turn(left, imuTarget(0), 1, 90);

 drive.setPID(2);
 drive.move(forward, 22, 2, 80);
 pros::delay(100); //get 1st ring on mogo 
 
 drive.setPID(2);
 drive.turn(left, imuTarget(95), 1, 90);

 setIntake(400, red);

 drive.move(forward, 13, 1, 100); //get 2nd ring
 pros::delay(150);
 
 drive.turn(left, imuTarget(190), 2, 90);

 drive.move(forward, 10, 1, 100); // touch elevation tower 
 
 runOnError.remove();
 runIntakeControl.remove();
 drive.onErrorVector.clear();

}

void ringSideRed(){
 pros::Task runOnError(onError_fn);
 pros::Task runIntakeControl(IntakeControlSystem_fn);
 Triangle tri;
 currentColor = blue;
 
 //get mogo
 drive.addErrorFunc(2, LAMBDA(clampPis.set_value(true)));
 drive.move(backward, 26, 2, 100);

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
 
 runOnError.remove();
 runIntakeControl.remove();
 drive.onErrorVector.clear();
}

void ringSideBlue(){
 pros::Task runOnError(onError_fn);
 pros::Task runIntakeControl(IntakeControlSystem_fn);
 Triangle tri;

 currentColor = red;
 
 //get mogo
 drive.addErrorFunc(2, LAMBDA(clampPis.set_value(true)));
 drive.move(backward, 26, 2, 100);

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
 
 runOnError.remove();
 runIntakeControl.remove();
 drive.onErrorVector.clear();  
}

void goalSideRed(){
 pros::Task runOnError(onError_fn);
 pros::Task runIntakeControl(IntakeControlSystem_fn);
 Triangle tri;
 currentColor = blue;
 setIntake(400, currentColor);
 
 //get mogo
 drive.addErrorFunc(18, LAMBDA(drive.setMaxVoltage(25)));
 drive.addErrorFunc(2, LAMBDA(clampPis.set_value(true)));
 drive.move(backward, 31, 2, 100);

 pros::delay(100); //let goal clamp

 drive.setPID(2);
 drive.setSlew(mogoSlewProfile);
 drive.move(backward, 5, 1, 100);

 startIntake();

 //go get 1st ring
 drive.turn(left, imuTarget(270), 2, 90);
 
 findTri(&tri, 27, 270);
 drive.move(forward, tri.hyp, 2, 70);
 
 drive.turn(right, imuTarget(0), 2, 90);

 setIntake(-400, std::nullopt);
 
 drive.move(forward, 28-tri.b, 3, 50);

 drive.turn(right, imuTarget(90), 2, 90);
 
 setIntake(400, currentColor);
 
 drive.addErrorFunc(24, LAMBDA(drive.setMaxVoltage(80)));
 drive.move(forward, 50, 3, 100);
 
 drive.move(forward, 24, 1, 40);
 pros::delay(300);
 
 drive.turn(right, imuTarget(225), 2, 90); 

 drive.move(forward, 29, 2, 100);
 
 runOnError.remove();
 runIntakeControl.remove();
 drive.onErrorVector.clear();
}

void goalSideBlue(){
 pros::Task runOnError(onError_fn);
 pros::Task runIntakeControl(IntakeControlSystem_fn);
 Triangle tri;
 currentColor = red;
 setIntake(400, currentColor);
 
 //get mogo
 drive.addErrorFunc(18, LAMBDA(drive.setMaxVoltage(25)));
 drive.addErrorFunc(2, LAMBDA(clampPis.set_value(true)));
 drive.move(backward, 31, 2, 100);

 pros::delay(100); //let goal clamp

 drive.setPID(2);
 drive.setSlew(mogoSlewProfile);
 drive.move(backward, 5, 1, 100);

 startIntake();

 //go get 1st ring
 drive.turn(right, imuTarget(90), 2, 90);
 
 findTri(&tri, 27, 270);
 drive.move(forward, tri.hyp, 2, 70);
 
 drive.turn(left, imuTarget(0), 2, 90);

 setIntake(-400, std::nullopt);
 
 drive.move(forward, 28-tri.b, 3, 50);

 drive.turn(left, imuTarget(270), 2, 90);
 
 setIntake(400, currentColor);
 
 drive.addErrorFunc(24, LAMBDA(drive.setMaxVoltage(80)));
 drive.move(forward, 50, 3, 100);
 
 drive.move(forward, 24, 1, 40);
 pros::delay(300);
 
 drive.turn(left, imuTarget(45), 2, 90); 

 drive.move(forward, 29, 2, 100);
 
 runOnError.remove();
 runIntakeControl.remove();
 drive.onErrorVector.clear();
}

void ringElimRed(){
 pros::Task runOnError(onError_fn);
 pros::Task runIntakeControl(IntakeControlSystem_fn);
 currentColor = blue;

 runOnError.remove();
 runIntakeControl.remove();
 drive.onErrorVector.clear();
}

void ringElimBlue(){
 pros::Task runOnError(onError_fn);
 pros::Task runIntakeControl(IntakeControlSystem_fn);
 currentColor = red;

 runOnError.remove();
 runIntakeControl.remove();
 drive.onErrorVector.clear();
}

void goalElimRed(){
 pros::Task runOnError(onError_fn);
 pros::Task runIntakeControl(IntakeControlSystem_fn);
 currentColor = blue;

 runOnError.remove();
 runIntakeControl.remove();
 drive.onErrorVector.clear();
}

void goalElimBlue(){
 pros::Task runOnError(onError_fn);
 pros::Task runIntakeControl(IntakeControlSystem_fn);
 currentColor = red;

 runOnError.remove();
 runIntakeControl.remove();
 drive.onErrorVector.clear();
}
           
void skills(){
 pros::Task runOnError(onError_fn); 
 pros::Task intakeControlTask(IntakeControlSystem_fn);
 pros::Task armControlTask(armControl_fn);
 Triangle tri;
 
 pros::delay(1); //dealy before intake can start for some reason (prob task related)

 setIntake(400, std::nullopt);
 startIntake();

 pros::delay(450); //score on allince stake with preload 
 
 drive.moveDriveVoltage(0);
 
 drive.setSlew(genSlewProfile);
 drive.move(forward, 14, 1, 100);

 drive.turn(left, imuTarget(270), 1, 70);
  
                 
                  /*{kP, kPa, kI, kIa, kD,  kDa,  kPd}*/
 drive.setCustomPID({50, 87,  0,  13,  160, 100,  150});
 drive.setSlew({0,35,70});
 
 drive.move(backward, 15, 3, 20);

 clampPis.set_value(true);
 pros::delay(300);

 drive.setPID(1);
 drive.move(backward, 7, 1, 100);
 
 drive.setPID(2);
 drive.turn(right, imuTarget(0), 1, 90);

 drive.setSlew(mogoSlewProfile);
 findTri(&tri, 22, 360);
 drive.move(forward, tri.hyp, 1, 100);
 
 drive.turn(right, imuTarget(75), 1, 90);
 
 //  drive.setPID(5);
 //  drive.swerve(forwardLeft, 58-tri.b, imuTarget(355), 3, 75, 45); //skills swerve 1 (tripple test/check this )
 //pros::delay(1000); //for testing
 
 drive.setPID(2);
 drive.move(forward, 24-tri.b, 2, 100);

 drive.setPID(3);
 drive.turn(left, imuTarget(360), 1, 90);

 drive.setPID(2);
 findTri(&tri, 41, 360);
 drive.move(forward, tri.hyp, 3, 100);
 
 drive.setPID(2);
 drive.turn(right, imuTarget(148), 2, 90); //turn to ring in front of wall stake 

 armControl.setTarget(load); //prime the wall stake mech
 
 drive.move(forward, 23-tri.b, 1, 100); //drive.move(forward, 23.5-tri.b, 1, 100); 

 pros::delay(500);

 drive.setPID(4);
 drive.turn(left, imuTarget(90), 1, 100);
 
 drive.setPID(2);
 drive.addErrorFunc(4, LAMBDA(stopIntake()));
 drive.move(forward, 6, 1, 100);

 //drive.moveDriveTrain(12000, 0.5);
 armControl.setTarget(score);  //score wall stake
 pros::delay(800);

 drive.moveDriveTrain(5000, 0.5);

 setIntake(-400, std::nullopt);
 startIntake();

  drive.moveDriveTrain(-8000, 0.2);
 //drive.move(backward, 13-tri.b, 1, 100);

 drive.setPID(4);
 drive.turn(right, imuTarget(180), 1, 100);

 setIntake(400, std::nullopt);

 armControl.setTarget(standby);

 drive.setPID(4);
 findTri(&tri, 49, 180);
 drive.addErrorFunc(tri.hyp-30, LAMBDA(drive.setMaxVoltage(20)));
 drive.move(forward, tri.hyp, 3, 100);

 pros::delay(200); //let first ring get itself together 

 drive.move(forward, 14-tri.b, 1, 100);

 drive.setPID(2);
 drive.turn(left, imuTarget(67), 2, 90); //get out of line ring 

 drive.move(forward, 15-tri.b, 1, 100);

 drive.turn(left, imuTarget(330), 2, 90);//turn to cornner 

 findTri(&tri, 10, 330);
 drive.addErrorFunc(3, LAMBDA(clampPis.set_value(false)));
 drive.move(backward, tri.hyp, 1, 100); //drop off 1st mogo

 pros::delay(400);

 drive.setPID(1);
 drive.setSlew(genSlewProfile);

 drive.move(forward, 12-tri.b, 1, 100);  //go get 2nd mogo

 drive.turn(right, imuTarget(90), 1, 70);  //turn to mogo 

 findTri(&tri, 68, 90);
 drive.setPID(4);
 drive.setSlew({0,0,0});
 drive.addErrorFunc(tri.hyp-40, LAMBDA(drive.setMaxVoltage(20)));
 drive.move(backward, tri.hyp, 5, 100); //go to 2nd mogo 
 
 clampPis.set_value(true); //2nd mogo secured 
 pros::delay(300);

 drive.setPID(2);
 drive.setSlew(mogoSlewProfile);
 drive.move(backward, 3-tri.b, 3, 100);

 drive.turn(left, imuTarget(360), 1, 90);

 findTri(&tri, 25, 360);
 drive.move(forward, tri.hyp, 1, 100);
 
 drive.setPID(4);
 drive.turn(right, imuTarget(50), 1, 100);

 drive.move(forward, 30-tri.b, 2, 100); //get ring under elevation tower 
 pros::delay(100); //wait to not hit elvation tower 
 
 findTri(&tri, 32, 40);
 drive.move(backward, tri.hyp, 2, 100); 
 
 drive.setPID(2);
 drive.turn(left, imuTarget(270), 1, 90);

 drive.move(forward, 32-tri.b, 2, 100); //get 3rd ring  

 drive.turn(right, imuTarget(335), 1, 90);
 
 drive.addErrorFunc(24, LAMBDA(armControl.setTarget(load)));
 drive.move(forward, 32, 1, 100);

 drive.turn(left, imuTarget(270), 1, 90);

 armControl.setTarget(score);
 pros::delay(1050);

 drive.move(backward, 14-tri.b, 1, 100);

 drive.setPID(4);
 drive.turn(left, imuTarget(0), 1, 100);
 armControl.setTarget(standby);

 drive.setPID(4);
 findTri(&tri, 49, 0);
 drive.addErrorFunc(tri.hyp-35, LAMBDA(drive.setMaxVoltage(25)));
 drive.move(forward, tri.hyp, 3, 100);

 pros::delay(500); //let first rings get themselves together 

 drive.move(forward, 18-tri.b, 1, 100);

 drive.setPID(2);
 drive.turn(right, imuTarget(247), 2, 90); //get out of line ring 

 drive.move(forward, 12-tri.b, 1, 100);

 drive.turn(right, imuTarget(145), 2, 90);
 findTri(&tri, 11, 145);
 drive.addErrorFunc(3, LAMBDA(clampPis.set_value(false)));
 drive.move(backward, tri.hyp, 1, 100); //drop off 1st mogo

 pros::delay(400);

 drive.setPID(1);
 drive.setSlew(genSlewProfile);


 
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

//                   /*{kP, kPa, kI, kIa, kD,  kDa,  kPd}*/
    //drive.setCustomPID({35,   0,  0,   0, 35,  100,  500});
    drive.setPID(1);
    drive.setSlew(genSlewProfile);
    drive.move(forward, 5, 2, 50);
    pros::delay(1000);
    drive.move(forward, 27, 2, 50);
    pros::delay(1000);
    drive.move(forward, 48, 2, 50);
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
