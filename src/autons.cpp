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

 drive.move(backward, 4.8, 1, 100);

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
 
 drive.turn(right, imuTarget(170), 2, 90);

 drive.move(forward, 16, 2, 100); // touch elevation tower 
 
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

 drive.move(backward, 4.8, 1, 100);

 startIntake();
 
 pros::delay(400);
 
 drive.move(forward, 4, 1, 100);
 
 drive.turn(left, imuTarget(180), 1, 70);

 drive.move(backward, 10, 1, 100);

 drive.turn(left, imuTarget(117), 1, 70); //turn to mogo

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
 drive.turn(left, imuTarget(265), 1, 90);

 setIntake(400, red);

 drive.move(forward, 13, 1, 100); //get 2nd ring
 pros::delay(150);
 
 drive.turn(left, imuTarget(190), 2, 90);

 drive.move(forward, 16, 1, 100); // touch elevation tower 
 
 runOnError.remove();
 runIntakeControl.remove();
 drive.onErrorVector.clear();

}

void ringSideRed(){
 pros::Task runOnError(onError_fn);
 pros::Task runIntakeControl(IntakeControlSystem_fn);
 Triangle tri;
 currentColor = blue;
 
 drive.addErrorFunc(20, LAMBDA(drive.setMaxVoltage(20)));
 drive.addErrorFunc(2.5, LAMBDA(clampPis.set_value(true)));
 drive.move(backward, 34, 2, 100);
 pros::delay(100); //let clamp lock 

 setIntake(400, currentColor);
 startIntake();
 
 drive.setPID(2);
 drive.setSlew(mogoSlewProfile);
 drive.turn(right, imuTarget(140), 1, 90);

 drive.move(forward, 22, 1, 100);

 drive.setPID(4);
 drive.turn(left, imuTarget(114), 1, 100);
 
 drive.setPID(2);
 drive.move(forward, 4.5, 1, 100); //get 2nd ring from ring stack 
 pros::delay(200);
 
 drive.move(backward, 16, 2, 100); //go get 1st 2 stack
 
 drive.setPID(4);
 drive.turn(left, imuTarget(60), 1, 100); //turn to 1st stack 

 drive.setPID(2);
 drive.move(forward, 22, 1, 100); //get 1st 2 stack 

 drive.turn(right, imuTarget(240), 1, 100);

 drive.move(forward, 27, 2, 100); //touch ele tow
//  drive.turn(left, imuTarget(293), 2, 90); //turn to 2nd 2 ring stack 
 
//  drive.addErrorFunc(20, LAMBDA(drive.setMaxVoltage(70)));
//  drive.move(forward, 45, 2, 100); //knock down stack  
 
//  pros::delay(700);
 
//  drive.move(forward, 26, 2, 100); //get 5th red 

//  pros::delay(150);

//  drive.turn(left, imuTarget(150), 1, 90);
 
//  drive.move(forward, 25, 1, 100); //touch bar  
 
 runOnError.remove();
 runIntakeControl.remove();
 drive.onErrorVector.clear();
}

void ringSideBlue(){
 pros::Task runOnError(onError_fn);
 pros::Task runIntakeControl(IntakeControlSystem_fn);
 Triangle tri;
 currentColor = red;
 
 drive.addErrorFunc(20, LAMBDA(drive.setMaxVoltage(20)));
 drive.addErrorFunc(2.5, LAMBDA(clampPis.set_value(true)));
 drive.move(backward, 34, 2, 100);
 pros::delay(100); //let clamp lock 

 setIntake(400, currentColor);
 startIntake();
 
 drive.setPID(2);
 drive.setSlew(mogoSlewProfile);
 drive.turn(left, imuTarget(220), 1, 90);

 drive.move(forward, 22, 1, 100);

 drive.setPID(4);
 drive.turn(right, imuTarget(252), 1, 100);
 
 drive.setPID(2);
 drive.move(forward, 4.7, 1, 100); //get 2nd ring from ring stack 
 pros::delay(700);
 
 drive.move(backward, 16, 2, 100); //go get 1st 2 stack
 
 drive.setPID(4);
 drive.turn(right, imuTarget(306), 1, 100); //turn to 1st stack 

 drive.setPID(2);
 drive.move(forward, 22, 1, 100); //get 1st 2 stack 

 drive.turn(left, imuTarget(126), 1, 100); //turn to 1st stack 

 drive.move(forward, 27, 2, 100); //touch ele tow 

//  drive.turn(right, imuTarget(73), 2, 90); //turn to 2nd 2 ring stack 
 
//  drive.addErrorFunc(20, LAMBDA(drive.setMaxVoltage(70)));
//  drive.move(forward, 45, 2, 100); //knock down stack  
 
//  pros::delay(700);
 
//  drive.move(forward, 26, 2, 100); //get 5th red 

//  pros::delay(150);

//  drive.turn(right, imuTarget(200), 1, 90);
 
//  drive.move(forward, 25, 1, 100); //touch bar  
 
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

 //drive.move(forward, 32, 2, 100); took out for elims 
 
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

 //drive.move(forward, 32, 2, 100); took out for elims
 
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
 Triangle tri;
 currentColor = blue;

 mogoArm.set_value(true);
 pros::delay(100);
 
 drive.setPID(5);
 findTri(&tri, 39, 360);
 drive.swerve(forwardLeft, tri.hyp, imuTarget(340), 3 , 80, 40);

 mogoArmClamp.set_value(true);
 pros::delay(200);

 pros::delay(2000);

 drive.setPID(6);
 drive.swerve(backwardRight, 52-tri.b, imuTarget(50), 5 , 50, 70);

 mogoArmClamp.set_value(false);
 pros::delay(200);

 pros::delay(2000);

 drive.setPID(1);

 findTri(&tri, 6, 25);
 drive.addErrorFunc(5, LAMBDA(mogoArm.set_value(false)));
 drive.move(backward, tri.hyp, 1, 100);
 
 drive.turn(right, imuTarget(215), 2, 70);
 
 drive.addErrorFunc(2.5, LAMBDA(clampPis.set_value(true)));
 drive.move(backward, 14, 1, 20);
 
 drive.turn(right, imuTarget(10), 1, 70);
 
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
 pros::delay(100);

 startIntake();

 pros::delay(450); //score on allince stake with preload 
 
 
 drive.setSlew(genSlewProfile);
 drive.move(forward, 14, 1, 100);

 drive.turn(left, imuTarget(270), 1, 70);
  
                 
                  /*{kP, kPa, kI, kIa, kD,  kDa,  kPd}*/
 drive.setCustomPID({50, 87,  0,  13,  160, 100,  150});
 drive.setSlew({0,35,70});

 drive.addErrorFunc(2, LAMBDA(clampPis.set_value(true)));
 drive.move(backward, 16, 3, 20);
 pros::delay(200);

 drive.setPID(1);
 drive.move(backward, 7, 1, 100);
 
 drive.setPID(2);
 drive.turn(right, imuTarget(0), 1, 90);

 drive.setSlew(mogoSlewProfile);
 findTri(&tri, 22, 360);
 drive.move(forward, tri.hyp, 1, 100);
 
 drive.turn(right, imuTarget(75), 1, 90);
 
 drive.setPID(2);
 drive.move(forward, 24-tri.b, 2, 100);

 drive.setPID(3);
 drive.turn(left, imuTarget(360), 1, 90);

 drive.setPID(2);
 findTri(&tri, 40, 360);
 drive.move(forward, tri.hyp, 3, 100); //get 3rd ring on goal 

 pros::delay(200);
 
 drive.setPID(2);
 drive.turn(right, imuTarget(153), 2, 90); //turn to ring in front of wall stake 

 armControl.setTarget(load); //prime the wall stake mech
 
 drive.move(forward, 23-tri.b, 1, 100); 

 pros::delay(900); //make sure ring loads

 drive.setPID(4);
 drive.turn(left, imuTarget(88), 1, 100);

 intake.move_voltage(-5000);
 pros::delay(100); //make sure the hooks won't stop the arm
 intake.move_voltage(0);
 
 drive.setPID(2);
 drive.addErrorFunc(3, LAMBDA(stopIntake()));
 drive.move(forward, 8, 1, 100);
 
 armControl.setTarget(score);  //score wall stake
 pros::delay(1000);

 drive.moveDriveTrain(9000, 1);

 armControl.setTarget(standby);
 pros::delay(1000);

 drive.moveDriveTrain(-8000, 0.2);
  
 setIntake(-400, std::nullopt); 
 startIntake();
 
 //turn to fill up rest of mogo 
 drive.setPID(4);
 drive.turn(right, imuTarget(180), 1, 100);
 
 stopIntake();
 setIntake(400, std::nullopt);

 pros::delay(100); //task are weird 

 startIntake();
 
 drive.setPID(4);
 findTri(&tri, 49, 180);
 drive.addErrorFunc(tri.hyp-30, LAMBDA(drive.setMaxVoltage(20)));
 drive.move(forward, tri.hyp, 3, 100);

 pros::delay(200); //let first ring get itself together 

 drive.move(forward, 14-tri.b, 1, 100);

 drive.setPID(2);
 drive.turn(left, imuTarget(67), 2, 90); //turn to out of line ring

 drive.move(forward, 11, 1, 100); //get out of line ring 
 
 drive.setPID(4);
 drive.turn(left, imuTarget(330), 2, 90); //turn to cornner  // deg turn 

 findTri(&tri, 9, 330);
 drive.addErrorFunc(3, LAMBDA(clampPis.set_value(false)));
 drive.move(backward, tri.hyp, 1, 100); //drop off 1st mogo

 pros::delay(600); //make sure lock isn't stuck 

 drive.setPID(1);
 drive.setSlew(genSlewProfile);
 
 findTri(&tri, 9, 330);
 drive.move(forward, tri.hyp, 1, 100);  //go get 2nd mogo

 drive.turn(right, imuTarget(90), 1, 70);  //turn to mogo 

 findTri(&tri, 70, 90);
 drive.setPID(4);
 drive.setSlew({0,0,0});
 drive.addErrorFunc(tri.hyp-48, LAMBDA(drive.setMaxVoltage(20)));
 
 drive.addErrorFunc(2.5, LAMBDA(clampPis.set_value(true))); //2nd mogo secured 
 drive.move(backward, tri.hyp, 15, 100); //go to 2nd mogo 
 
 pros::delay(300);

 drive.setPID(2);
 drive.setSlew(mogoSlewProfile);
 drive.move(backward, 8-tri.b, 3, 100);

 drive.turn(left, imuTarget(360), 1, 90);

 findTri(&tri, 20, 360);
 drive.move(forward, tri.hyp, 1, 100);
 
 drive.setPID(2);
 drive.turn(left, imuTarget(275), 1, 90);

 drive.move(forward, 20-tri.b, 2, 100); //get 2nd ring 

 drive.turn(right, imuTarget(360), 2, 90);
 
 drive.setPID(2);
 drive.move(forward, 40-tri.b, 3, 100); //get 3rd ring

 drive.turn(left, imuTarget(211), 1, 90); //turn to wall stake 

 armControl.setTarget(load);

 findTri(&tri, 22.5, 211);
 drive.move(forward, tri.hyp, 1, 100);

 pros::delay(800); //let ring load 

 drive.turn(right, imuTarget(275), 1, 90);

 stopIntake();
 intake.move_voltage(-5000);
 pros::delay(100); //make sure the hooks won't stop the arm
 intake.move_voltage(0);
 
 drive.setPID(2);
 drive.move(forward, 8, 1, 100);
 
 armControl.setTarget(score);  //score wall stake
 pros::delay(1000);

 drive.moveDriveTrain(8000, 1);

 armControl.setTarget(standby);
 pros::delay(1000);

 drive.moveDriveTrain(-8000, 0.2);

 startIntake();

 drive.setPID(4);
 drive.turn(left, imuTarget(0), 1, 100); //turn to in line rings 

 drive.setPID(4);
 findTri(&tri, 49, 0);
 drive.addErrorFunc(tri.hyp-35, LAMBDA(drive.setMaxVoltage(25)));
 drive.move(forward, tri.hyp, 3, 100);

 pros::delay(500); //let first rings get themselves together 

 drive.move(forward, 12-tri.b, 1, 100);

 drive.setPID(2);
 drive.turn(right, imuTarget(160), 2, 90); //turn to out of line ring 

 drive.move(forward, 12-tri.b, 1, 100); //get out of line ring 

 drive.turn(right, imuTarget(250), 2, 90);
 findTri(&tri, 11, 145);
 drive.addErrorFunc(3, LAMBDA(clampPis.set_value(false)));
 drive.move(backward, tri.hyp, 1, 100); //drop off 1st mogo

 pros::delay(600); //make sure clamp unlocks 

 drive.setPID(1);
 drive.setSlew(genSlewProfile);
 drive.move(forward, 25-tri.b, 1, 100);

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
