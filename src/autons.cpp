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
slewProfile RingSideMogoGrabSLewProfile{60, 0, 70};
IntakeControl conveyor;

void winPointRed(){
 pros::Task runOnError(onError_fn); 
 pros::Task runIntakeControl(IntakeControlSystem_fn);
 Triangle tri;
 setIntake(400, std::nullopt);

 currentColor = blue;

 drive.move(backward, 14.5, 1, 100);

 drive.turn(right, imuTarget(90), 1, 70);

 drive.move(backward, 5, 1, 100);

 startIntake();
 
 pros::delay(400);
 
 drive.move(forward, 4, 1, 100);
 
 drive.turn(right, imuTarget(180), 1, 70);

 drive.move(backward, 10, 1, 100);

 drive.turn(right, imuTarget(243), 1, 70); //turn to mogo

 drive.addErrorFunc(18, LAMBDA(drive.setMaxVoltage(20)));
 drive.addErrorFunc(4, LAMBDA(clampPis.set_value(true)));
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
 
 drive.turn(right, imuTarget(180), 2, 90);

 stopIntake();
 runIntakeControl.remove();
 pros::delay(1);
 intake.move_voltage(12000);

 drive.move(forward, 20, 1, 100); // touch elevation tower 
 
 runOnError.remove();
 drive.onErrorVector.clear();
}

void winPointBlue(){
 pros::Task runOnError(onError_fn); 
 pros::Task runIntakeControl(IntakeControlSystem_fn);
 Triangle tri;
 setIntake(400, currentColor);

 currentColor = red;

 drive.move(backward, 15, 1, 100);

 drive.turn(left, imuTarget(270), 1, 70);

 drive.move(backward, 5.5, 1, 100);

 startIntake();
 
 pros::delay(400);
 
 drive.move(forward, 5, 1, 100);
 
 drive.turn(left, imuTarget(180), 1, 70);

 drive.move(backward, 8, 1, 100);

 drive.turn(left, imuTarget(117), 1, 70); //turn to mogo

 drive.addErrorFunc(17, LAMBDA(drive.setMaxVoltage(25)));
 drive.addErrorFunc(2.5, LAMBDA(clampPis.set_value(true)));
 drive.move(backward, 33, 2, 100);
 pros::delay(200); //let clamp lock 
 
 drive.setPID(2);
 drive.setSlew(mogoSlewProfile);
 drive.turn(left, 310, 2, 90); //turn to ring stack

 startIntake();

 drive.move(forward, 23-tri.b, 1, 100); 

 pros::delay(100);

//  drive.setPID(2);
//  drive.turn(left, imuTarget(265), 1, 90);

//  setIntake(400, red);

//  drive.move(forward, 13, 1, 100); //get 2nd ring
//  pros::delay(150);
 
 
//  drive.move(backward, 6, 1, 100);
//  drive.turn(left, imuTarget(180), 2, 90);

//  stopIntake();
//  runIntakeControl.remove();
//  pros::delay(1);
//  intake.move_voltage(12000);

//  drive.move(forward, 28, 2, 100); // touch elevation tower  */
 
 runOnError.remove();
 drive.onErrorVector.clear();
}

void ringSideRed(){
 pros::Task runOnError(onError_fn);
 pros::Task runIntakeControl(IntakeControlSystem_fn);
 Triangle tri;
 currentColor = blue;
 setIntake(400, currentColor);

 pros::delay(1); //fix weird on error issue hoepfully?
 
 drive.setPID(7);
 drive.setSlew(RingSideMogoGrabSLewProfile);
 
 findTri(&tri, 25, 29);
 drive.addErrorFunc(20, LAMBDA(drive.setSlew({0,0,0})));
 drive.addErrorFunc(20, LAMBDA(drive.setMaxVoltage(30)));
 drive.addErrorFunc(4, LAMBDA(clampPis.set_value(true)));
 drive.move(backward, tri.hyp, 3, 100);


 drive.setPID(2);
 drive.setSlew(mogoSlewProfile);
 drive.turn(right, imuTarget(140), 1, 90); //turn to ring stack
 
 startIntake();

 drive.move(forward, 23-tri.b, 1, 100); 

 pros::delay(100);

 drive.setPID(4);
 drive.turn(left, imuTarget(100), 1, 100);
 
 findTri(&tri, 8, 92);
 drive.setPID(2);
 drive.move(forward, tri.hyp, 1, 100); //get 2nd ring from ring stack 
 pros::delay(200);
 
 drive.move(backward, 8-tri.b, 2, 100); //go get 1st 2 stack
 
 drive.setPID(4);
 drive.turn(left, imuTarget(40), 1, 100); //turn to 1st stack 
 
 findTri(&tri, 10, 40);
 drive.setPID(2);
 drive.move(forward, tri.hyp, 1, 100); //get 1st 2 stack 
 
 drive.turn(left, imuTarget(308), 2, 90); //turn to 2nd 2 ring stack 
 
 drive.addErrorFunc(30, LAMBDA(intakePis.set_value(true)));
 drive.addErrorFunc(20, LAMBDA(drive.setMaxVoltage(50)));
 drive.move(forward, 47-tri.b, 4, 100); //get 5th ring   

 intakePis.set_value(false);
 pros::delay(300);

 drive.move(backward, 5, 1, 100); //back UP TO take ring and touch ele 
 pros::delay(500);
 drive.move(backward, 5, 1, 100); //back UP TO take ring and touch ele 

 drive.turn(left, imuTarget(180), 2, 90);

 drive.move(forward, 18, 1, 100); //touch bar  

 stopIntake();
 runIntakeControl.remove();
 
 runOnError.remove();
 drive.onErrorVector.clear();
}

void ringSideBlue(){
 pros::Task runOnError(onError_fn);
 pros::Task runIntakeControl(IntakeControlSystem_fn);
 Triangle tri;
 currentColor = red;
 setIntake(400, std::nullopt); //currentColor

 pros::delay(1); //fix weird on error issue hoepfully?
 
 drive.setPID(7);
 drive.setSlew(RingSideMogoGrabSLewProfile);
 findTri(&tri, 25, 331);
 drive.addErrorFunc(20, LAMBDA(drive.setSlew({0,0,0})));
 drive.addErrorFunc(20, LAMBDA(drive.setMaxVoltage(30)));
 drive.addErrorFunc(4, LAMBDA(clampPis.set_value(true)));
 drive.move(backward, tri.hyp, 3, 100);

 drive.setPID(2);
 drive.setSlew(mogoSlewProfile);
 drive.turn(left, imuTarget(220), 1, 90); //turn to ring stack
 
 startIntake();

 drive.move(forward, 23-tri.b, 1, 100); 

 pros::delay(100);

 drive.setPID(4);
 drive.turn(right, imuTarget(260), 1, 100);
 
 findTri(&tri, 8, 260);
 drive.setPID(2);
 drive.move(forward, tri.hyp, 1, 100); //get 2nd ring from ring stack 
 pros::delay(200);
 
 drive.move(backward, 8-tri.b, 2, 100); //go get 1st 2 stack
 
 drive.setPID(4);
 drive.turn(right, imuTarget(320), 1, 100); //turn to 1st stack 
 
 findTri(&tri, 10, 320);
 drive.setPID(2);
 drive.move(forward, tri.hyp, 1, 100); //get 1st 2 stack 
 
 drive.turn(right, imuTarget(52), 2, 90); //turn to 2nd 2 ring stack 
 
 drive.addErrorFunc(30, LAMBDA(intakePis.set_value(true)));
 drive.addErrorFunc(20, LAMBDA(drive.setMaxVoltage(50)));
 drive.move(forward, 47-tri.b, 3, 100); //get 5th ring   

 intakePis.set_value(false);
 pros::delay(100);

 drive.move(backward, 5, 1, 100); //vCK UP TO take ring and touch ele 
 pros::delay(500);
 drive.move(backward, 5, 1, 100); //vCK UP TO take ring and touch ele 

 drive.turn(right, imuTarget(180), 2, 90);

 stopIntake();
 runIntakeControl.remove();

 pros::delay(20);

 intake.move_voltage(12000);
 
 drive.move(forward, 18, 1, 100); //touch bar  
 
 runOnError.remove();
 drive.onErrorVector.clear();
}

void goalSideRed(){
 pros::Task runOnError(onError_fn);
 pros::Task runIntakeControl(IntakeControlSystem_fn);
 Triangle tri;
 currentColor = blue;
 setIntake(400, currentColor);

 pros::delay(1); //possible weird task spawn issue 
 
 drive.setPID(7);
 drive.setSlew(RingSideMogoGrabSLewProfile);
 findTri(&tri, 28, 338);
 drive.addErrorFunc(20, LAMBDA(drive.setSlew({0,0,0})));
 drive.addErrorFunc(20, LAMBDA(drive.setMaxVoltage(30)));
 drive.addErrorFunc(4, LAMBDA(clampPis.set_value(true)));
 drive.move(backward, tri.hyp, 3, 50);

 pros::delay(300); //let goal clamp

 drive.setPID(2);
 drive.setSlew(mogoSlewProfile);

 startIntake();

 //go get 1st ring
 drive.turn(left, imuTarget(270), 2, 90);
 
 findTri(&tri, 20, 270);
 drive.move(forward, tri.hyp, 2, 70);
 
 drive.turn(right, imuTarget(0), 2, 90);

 setIntake(-400, std::nullopt);
 
 drive.move(forward, 24-tri.b, 3, 100);

 drive.turn(right, imuTarget(90), 2, 90);
 
 setIntake(400, currentColor);
 
 findTri(&tri, 42, 90);
 drive.addErrorFunc(24, LAMBDA(drive.setMaxVoltage(80)));
 drive.addErrorFunc(28, LAMBDA(intakePis.set_value(true)));
 drive.move(forward, tri.hyp, 3, 100);
 intakePis.set_value(false);
 pros::delay(150);

 drive.move(backward, 7, 1, 100);
 
 pros::delay(350);
  
 drive.turn(right, imuTarget(180), 2, 90); 

 drive.move(forward, 26, 2, 100);
 
 stopIntake();
 runIntakeControl.remove();
 runOnError.remove();
 drive.onErrorVector.clear();
}

void goalSideBlue(){
 pros::Task runOnError(onError_fn);
 pros::Task runIntakeControl(IntakeControlSystem_fn);
 Triangle tri;
 currentColor = red;
 setIntake(400, std::nullopt); //currentColor
 
 //get mogo
 drive.addErrorFunc(20, LAMBDA(drive.setMaxVoltage(25)));
 drive.addErrorFunc(4, LAMBDA(clampPis.set_value(true)));
 drive.move(backward, 32, 2, 100);

 pros::delay(300); //let goal clamp

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
 drive.addErrorFunc(28, LAMBDA(intakePis.set_value(true)));
 drive.move(forward, 45, 3, 100);
 intakePis.set_value(false);
 pros::delay(150);

 drive.move(backward, 7, 1, 100);
 
 pros::delay(350);
  
 drive.turn(left, imuTarget(180), 2, 90); 
 
 drive.move(forward, 26, 2, 100);
 
 stopIntake();
 runIntakeControl.remove();
 runOnError.remove();
 drive.onErrorVector.clear();
}

void ringElimRed(){
 pros::Task runOnError(onError_fn);
 pros::Task runIntakeControl(IntakeControlSystem_fn);
 currentColor = blue;

 Triangle tri;
 setIntake(400, currentColor);
 
 findTri(&tri, 15, 0);
 drive.move(backward, tri.hyp, 1, 100);

 drive.turn(right, imuTarget(90), 1, 70);
 
 drive.setSlew(RingSideMogoGrabSLewProfile);
 drive.move(backward, 4-tri.b, 1, 100);
 drive.setSlew({0,0,0});

 startIntake();
 
 pros::delay(400);

 findTri(&tri, 5, 90);
 drive.move(forward, tri.hyp, 1, 100);
 
 drive.turn(right, imuTarget(180), 1, 70);

 drive.move(backward, 8-tri.b, 1, 100);
 
 drive.setPID(3);
 drive.turn(right, imuTarget(240), 1, 70); //turn to mogo

 drive.setPID(7);
 drive.setSlew({85,0,45});
 findTri(&tri, 33, 240);
 drive.addErrorFunc(15, LAMBDA(drive.setSlew({0,0,0})));
 drive.addErrorFunc(15, LAMBDA(drive.setMaxVoltage(30)));
 drive.addErrorFunc(4, LAMBDA(clampPis.set_value(true)));
 drive.move(backward, tri.hyp, 3, 50);

 drive.setPID(2);
 drive.setSlew(mogoSlewProfile);
 drive.turn(right, imuTarget(35), 2, 90); //turn to ring stack

 startIntake();
 
 findTri(&tri, 25, 30);
 drive.move(forward, tri.hyp, 2, 100); 

 pros::delay(100);

 drive.setPID(4);
 drive.turn(left, imuTarget(7), 1, 100);
 
 drive.setPID(2);
 drive.move(forward,7-tri.b, 1, 100); //get 2nd ring from ring stack 
 pros::delay(200);
 
 drive.move(backward, 8-tri.b, 2, 100); //go get 1st 2 stack
 
 drive.setPID(2); //4
 drive.turn(left, imuTarget(297), 1, 100); //turn to 1st stack 

 stopIntake();
 runIntakeControl.remove();
 pros::delay(1);
 intake.move_voltage(12000);
 
 findTri(&tri, 10, 297);
 drive.setPID(2);
 drive.move(forward, tri.hyp, 1, 100); //get 1st 2 stack 

 runOnError.remove();
 drive.onErrorVector.clear();
}

void ringElimBlue(){
 pros::Task runOnError(onError_fn);
 pros::Task runIntakeControl(IntakeControlSystem_fn);
 currentColor = red;

 Triangle tri;
 setIntake(400, currentColor);
 
 findTri(&tri, 15, 0);
 drive.move(backward, tri.hyp, 1, 100);

 drive.turn(left, imuTarget(270), 1, 70);

 drive.setSlew(RingSideMogoGrabSLewProfile);
 drive.move(backward, 4-tri.b, 1, 100);
 drive.setSlew({0,0,0});

 startIntake();
 
 pros::delay(400);

 findTri(&tri, 5, 270);
 drive.move(forward, tri.hyp, 1, 100);
 
 drive.turn(left, imuTarget(180), 1, 70);

 drive.move(backward, 8-tri.b, 1, 100);

 drive.turn(left, imuTarget(120), 1, 70); //turn to mogo

 drive.setPID(7);
 drive.setSlew({90,0,45});
 findTri(&tri, 33, 117);
 drive.addErrorFunc(15, LAMBDA(drive.setSlew({0,0,0})));
 drive.addErrorFunc(15, LAMBDA(drive.setMaxVoltage(30)));
 drive.addErrorFunc(4, LAMBDA(clampPis.set_value(true)));
 drive.move(backward, tri.hyp, 3, 50);

 drive.setPID(2);
 drive.setSlew(mogoSlewProfile);
 drive.turn(left, imuTarget(315), 2, 90); //turn to ring stack

 startIntake();
 
 findTri(&tri, 25, 315);
 drive.move(forward, tri.hyp, 2, 100); 

 pros::delay(100);

 drive.setPID(4);
 drive.turn(right, imuTarget(355), 1, 100);
 
 drive.setPID(2);
 drive.move(forward,8-tri.b, 1, 100); //get 2nd ring from ring stack 
 pros::delay(200);
 
 drive.move(backward, 8-tri.b, 2, 100); //go get 1st 2 stack
 
 drive.setPID(2); //4
 drive.turn(right, imuTarget(63), 1, 100); //turn to 1st stack 

 stopIntake();
 runIntakeControl.remove();
 pros::delay(1);
 intake.move_voltage(12000);
 
 findTri(&tri, 10, 30);
 drive.setPID(2);
 drive.move(forward, tri.hyp, 1, 100); //get 1st 2 stack 

 runOnError.remove();
 drive.onErrorVector.clear();
}

void goalElimRed(){
 pros::Task runOnError(onError_fn);
 pros::Task runIntakeControl(IntakeControlSystem_fn);
 Triangle tri;
 currentColor = blue;
 setIntake(400, currentColor);

 mogoArm.set_value(true);
 pros::delay(100);
 
 findTri(&tri, 41, 360);
 drive.setPID(5);
 drive.addErrorFunc(15, LAMBDA(drive.setMaxVoltage(60))); 
 drive.addErrorFunc(1, LAMBDA(mogoArmClamp.set_value(true))); 
 drive.swerve(forwardLeft, tri.hyp, imuTarget(341), 3 , 90, 55);

 drive.setPID(6);
 drive.swerve(backwardRight, 52-tri.b, imuTarget(70), 4 , 70, 100); //imutarget(50) 50V 70VA

 mogoArmClamp.set_value(false);
 pros::delay(150);

 drive.setPID(1);
 findTri(&tri, 4, 70);
 drive.addErrorFunc(3, LAMBDA(mogoArm.set_value(false)));
 drive.move(backward, tri.hyp, 1, 100);
  
 drive.turn(right, imuTarget(220), 2, 70);
 
 drive.addErrorFunc(2.5, LAMBDA(clampPis.set_value(true)));
 drive.addErrorFunc(12, LAMBDA(drive.setMaxVoltage(25))); 
 drive.move(backward, 19-tri.b, 1, 100); //clamo on to middle goal reg

 findTri(&tri, 4, 215);
 
 drive.setPID(2);
 drive.addErrorFunc(3, LAMBDA(startIntake()));
 drive.move(backward, tri.hyp, 1, 100);
 
 drive.turn(right, imuTarget(360), 1, 90);

 drive.setPID(2);
 drive.move(forward, 12-tri.b, 1, 100); //score ring from 2 stack 

 pros::delay(200); 

 drive.turn(right, imuTarget(148), 2, 90);

 mogoArm.set_value(true);
 drive.move(forward, 42, 2, 100);
 
 drive.turn(right, imuTarget(328), 2, 90);
 
 
//  findTri(&tri, 8, 360);
//  drive.move(backward, tri.hyp, 1, 100);

//  clampPis.set_value(false); //drop off mid goal 
//  pros::delay(150);

//  stopIntake();

//  drive.setPID(1);

//  drive.move(forward, 8-tri.b, 1, 100); //go to allince side goal 
//  drive.turn(right, imuTarget(90), 1, 70); //turn to allince side goal 

//  findTri(&tri, 20, 90);
//  drive.setCustomPID({50, 87,  0,  13,  160, 100,  150});
//  drive.addErrorFunc(2, LAMBDA(clampPis.set_value(true)));
//  drive.addErrorFunc(12, LAMBDA(drive.setMaxVoltage(25))); 
//  drive.move(backward, tri.hyp, 1, 100); //get allince side goal 

//  drive.setPID(2);
//  drive.turn(right, imuTarget(230), 1.5, 90);

//  startIntake();
//  intakePis.set_value(true);
 
//  findTri(&tri, 31, 230);
//  drive.addErrorFunc(12, LAMBDA(intakePis.set_value(false))); 
//  drive.move(forward, tri.hyp, 1.5, 100);
//  pros::delay(100);

//  drive.move(backward, 5, 0.5, 100);
 
 stopIntake();
 runIntakeControl.remove();
 runOnError.remove();
 drive.onErrorVector.clear();
}

void goalElimBlue(){
 pros::Task runOnError(onError_fn);
 pros::Task runIntakeControl(IntakeControlSystem_fn);
 Triangle tri;
 currentColor = red;

 setIntake(400, currentColor);

 mogoArm.set_value(true);
 pros::delay(100);
 
 findTri(&tri, 38, 360);
 drive.setPID(5);
 drive.addErrorFunc(15, LAMBDA(drive.setMaxVoltage(60))); 
 drive.swerve(forwardLeft, tri.hyp, imuTarget(340), 2, 100, 50);

 mogoArmClamp.set_value(true);
 
 drive.swerve(backwardRight, 38-tri.b, imuTarget(10), 2, 100, 30);

 mogoArmClamp.set_value(false);
 pros::delay(100);

 drive.move(backward, 4, 1, 100);

//  drive.setPID(2);
//  drive.turn(shortest, 165, 2, 90); 

//  drive.setPID(1);
//  findTri(&tri, 4, 70);
//  drive.addErrorFunc(3, LAMBDA(mogoArm.set_value(false)));
//  drive.move(backward, tri.hyp, 1, 100);
  
//  drive.turn(right, imuTarget(220), 2, 70);
 
//  drive.addErrorFunc(2.5, LAMBDA(clampPis.set_value(true)));
//  drive.addErrorFunc(12, LAMBDA(drive.setMaxVoltage(25))); 
//  drive.move(backward, 16-tri.b, 1, 100); //clamo on to middle goal reg

//  findTri(&tri, 4, 215);
 
//  drive.setPID(2);
//  drive.addErrorFunc(3, LAMBDA(startIntake()));
//  drive.move(backward, tri.hyp, 1, 100);
 
//  drive.turn(right, imuTarget(360), 1, 70);

//  drive.setPID(2);
//  drive.move(forward, 12-tri.b, 1, 100); //score ring from 2 stack 

//  pros::delay(200); 
 
//  findTri(&tri, 6, 360);
//  drive.move(backward, tri.hyp, 1, 100);

//  clampPis.set_value(false); //drop off mid goal 
//  pros::delay(150);

//  stopIntake();

//  drive.setPID(1);

//  drive.move(forward, 9.5-tri.b, 1, 100); //go to allince side goal 
//  drive.turn(right, imuTarget(270), 1, 70); //turn to allince side goal 

//  findTri(&tri, 16, 90);
//  drive.setCustomPID({50, 87,  0,  13,  160, 100,  150});
//  drive.addErrorFunc(2.5, LAMBDA(clampPis.set_value(true)));
//  drive.addErrorFunc(12, LAMBDA(drive.setMaxVoltage(25))); 
//  drive.move(backward, tri.hyp, 1, 100); //get allince side goal 


 
 runOnError.remove();
 runIntakeControl.remove();
 drive.onErrorVector.clear();
}
           
void skills(){
 pros::Task runOnError(onError_fn); 
 pros::Task intakeControlTask(IntakeControlSystem_fn);
 pros::Task armControlTask(armControl_fn);
 Triangle tri;

 pros::delay(1);
 
 setIntake(400, std::nullopt);

 intake.move_voltage(12000);
 pros::delay(450); //score on allince stake with preload 
 intake.move_voltage(0);
 
 drive.setSlew(genSlewProfile);
 drive.move(forward, 14, 1, 100);
  
 startIntake();
 
 drive.turn(left, imuTarget(270), 1, 70);
                 
                  /*{kP, kPa, kI, kIa, kD,  kDa,  kPd}*/
 drive.setCustomPID({50, 87,  0,  13,  160, 100,  150});
 drive.setSlew({0,0,0});
 drive.addErrorFunc(12, LAMBDA(drive.setMaxVoltage(25)));
 drive.addErrorFunc(2, LAMBDA(clampPis.set_value(true)));
 drive.move(backward, 17, 3, 100);
 pros::delay(100);

 drive.setPID(1);
 drive.move(backward, 5, 1, 100);
 
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
 findTri(&tri, 37, 360);
 drive.move(forward, tri.hyp, 3, 100); //get 3rd ring on goal 

 pros::delay(200); //intake before big turn 
 
 drive.setPID(2);
 drive.turn(shortest, 178, 2, 90); //turn to inline rings 

//  armControl.setTarget(load); //prime the wall stake mech
//  stopIntake(); //no anti jam 
 
//  drive.addErrorFunc(20, LAMBDA(intake.move_voltage(12000)));
//  drive.addErrorFunc(8, LAMBDA(drive.setMaxVoltage(80)));
//  drive.move(forward, 19.4, 1, 100); 

//  pros::delay(100); //make sure ring loads

//  drive.setPID(4);
//  drive.turn(left, imuTarget(90), 1, 100);
 
//  //intake.move_voltage(-5000);
//  //pros::delay(100); //make sure the hooks won't stop the arm
//  //intake.move_voltage(0);
 
//  drive.setPID(2);
//  //drive.addErrorFunc(3, LAMBDA(stopIntake()));
//  drive.move(forward, 2.4, 1, 100); //get closer to stake 
 
//  intake.move_voltage(0);
//  setIntake(-400, std::nullopt); 
//  armControl.setTarget(score);  //score wall stake
//  pros::delay(700); //1000

//  drive.moveDriveTrain(9000, .45);
//  armControl.setTarget(standby);
//  startIntake();

//  drive.move(backward, 12, 1, 100);
  
//  setIntake(-400, std::nullopt); 

//  //turn to fill up rest of mogo 
//  drive.setPID(4);
//  drive.turn(right, imuTarget(180), 1, 100);

//  pros::delay(500); //make up for missing ring @ stem comp 

//  drive.setPID(4);
//  drive.addErrorFunc(10, LAMBDA(drive.setMaxVoltage(20))); 
//  drive.addErrorFunc(39, LAMBDA(setIntake(400, std::nullopt)));
//  drive.move(forward, 49, 3, 100);

                  /*{kP, kPa, kI, kIa, kD,  kDa,  kPd}*/
 drive.setCustomPID({20, 143, 0,  8,   50,  350,  600});   
 drive.addErrorFunc(10, LAMBDA(drive.setMaxVoltage(20))); 
 drive.addErrorFunc(39, LAMBDA(setIntake(400, std::nullopt)));
 drive.move(forward, 66, 3, 100);

 pros::delay(100); //let first ring get itself together 
 
 drive.setPID(2);
 findTri(&tri, 9, 178);
 drive.move(forward, tri.hyp, 1, 100);

 drive.turn(left, imuTarget(67), 2, 90); //turn to out of line ring

 drive.move(forward, 9-tri.b, 1, 100); //get out of line ring 
 
 drive.setPID(4);
 drive.turn(left, imuTarget(317), 2, 90); //turn to cornner  // deg turn 

 findTri(&tri, 9, 317);
 drive.addErrorFunc(3, LAMBDA(clampPis.set_value(false)));
 drive.move(backward, tri.hyp, 1, 100); //drop off 1st mogo

 pros::delay(250); //make sure lock isn't stuck 

 drive.setPID(1);
 drive.setSlew(genSlewProfile);
 
 findTri(&tri, 12.5, 317);
 drive.move(forward, tri.hyp, 1, 100);  //go get 2nd mogo

 drive.turn(right, imuTarget(90), 1, 70);  //turn to mogo 

 findTri(&tri, 71, 90);
 drive.setPID(4);
 drive.setSlew({0,0,0});
 drive.addErrorFunc(tri.hyp-48, LAMBDA(drive.setMaxVoltage(20)));
 drive.addErrorFunc(2.5, LAMBDA(clampPis.set_value(true))); //2nd mogo secured 
 drive.move(backward, tri.hyp, 15, 100); //go to 2nd mogo 
 
 pros::delay(100);

 drive.setPID(2);
 drive.setSlew(mogoSlewProfile);
 drive.move(backward, 5-tri.b, 3, 100);

 drive.turn(left, imuTarget(360), 1, 90);

 findTri(&tri, 22, 360);
 drive.move(forward, tri.hyp, 1, 100);
 
 drive.setPID(2);
 drive.turn(left, imuTarget(285), 1, 90);

 drive.move(forward, 24-tri.b, 2, 100); //get 2nd ring 

 drive.turn(right, imuTarget(360), 2, 90);
 
 drive.setPID(2);
 drive.move(forward, 44-tri.b, 5, 100); //get 3rd ring

 pros::delay(900);

//  drive.turn(left, imuTarget(211), 1, 90); //turn to wall stake ring 

//  armControl.setTarget(load);
//  stopIntake();

//  findTri(&tri, 26.6, 211);
//  drive.addErrorFunc(20, LAMBDA(intake.move_voltage(12000)));
//  drive.move(forward, tri.hyp, 1, 100);

//  pros::delay(100); //let ring load 

//  drive.turn(right, imuTarget(270), 1, 90);

//  //intake.move_voltage(-5000);
//  //pros::delay(100); //make sure the hooks won't stop the arm
//  //intake.move_voltage(0);
 
//  //drive.setPID(2);
 
//  drive.move(forward, 2.4, 1, 100); //get closer to stake 
//  intake.move_voltage(0); //tuen off intak e
//  armControl.setTarget(score);  //score wall stake
//  pros::delay(700);

//  drive.moveDriveTrain(9000, .45);

//  armControl.setTarget(standby);
//  //pros::delay(1000);

//  setIntake(-400, std::nullopt); 
 
//  drive.move(backward, 11, 1, 100);

//  startIntake();

 drive.setPID(2); //4
 drive.turn(shortest, imuTarget(178), 2, 100); //turn to in line rings  
 
                  /*{kP, kPa, kI, kIa, kD,  kDa,  kPd}*/
 drive.setCustomPID({20, 143, 0,  8,   50,  350,  600});   
 drive.addErrorFunc(10, LAMBDA(drive.setMaxVoltage(20))); 
 drive.addErrorFunc(39, LAMBDA(setIntake(400, std::nullopt)));
 drive.move(forward, 74, 4, 100);
 
 pros::delay(100); //let first rings get themselves together 
 
 findTri(&tri, 10, 178);
 drive.move(forward, tri.hyp, 1, 100);

 drive.setPID(2);
 drive.turn(right, imuTarget(300), 2, 90); //turn to out of line ring 

 drive.move(forward, 9-tri.b, 1, 100); //get out of line ring 
 
 drive.turn(right, imuTarget(43), 2, 90);

 findTri(&tri, 10, 43);
 drive.addErrorFunc(3, LAMBDA(clampPis.set_value(false)));
 drive.move(backward, tri.hyp, 1, 100); //drop off 2nd mogo

 pros::delay(500); //make sure clamp unlocks 

 drive.setPID(1);
 drive.setSlew(genSlewProfile);
 
 drive.move(forward, 25, 1, 100);

 drive.setPID(2);
 drive.turn(left, imuTarget(0), 1, 70);
 
 drive.setPID(3);
 drive.move(forward, 63, 3, 100);

 drive.setPID(1);
 drive.turn(right, imuTarget(90), 1, 70); //turn to pick up ring 
 
 drive.addErrorFunc(3, LAMBDA(stopIntake()));
 drive.move(forward, 32.5, 2, 100); //pick up ring 

 drive.setPID(1);
 drive.turn(right, imuTarget(180), 2, 70); //turn to go push 1st blue mogo 
 
 drive.setSlew({0,0,0});
 drive.move(backward, 37, 2, 100); //slow for some reason 
 
 drive.turn(right, imuTarget(270), 1, 70); //turn front to to mogo 
 
 drive.move(forward, 44, 3, 100); //push mogo in corner 

 drive.move(backward, 50, 3, 100); // go to allince stake 

 drive.turn(left, imuTarget(180), 2, 70); //turn to allince stake 

 drive.move(backward, 3, 1, 100); //back up and score 

 startIntake();
 pros::delay(500);
 
 drive.move(forward, 3, 1, 100); //leave to go score mogo 

 drive.turn(right, imuTarget(270), 1, 70);
 drive.setSlew({0,0,0});
 drive.move(backward, 62, 3, 100); //push 4th mogo in corner 
 
 runOnError.remove();
 intakeControlTask.remove();
 armControlTask.remove();
 drive.onErrorVector.clear();
}

void tune(){
 pros::Task runOnError(onError_fn);

                   /*{kP, kPa, kI, kIa, kD,  kDa,  kPd}*/
 //drive.setCustomPID({50, 87,  0,  13,  160, 100,  150});
 //drive.setSlew({0,0,0});
  drive.setPID(7);
 //drive.setSlew({90,0,45});
 drive.addErrorFunc(15, LAMBDA(drive.setSlew({0,0,0})));
 drive.addErrorFunc(15, LAMBDA(drive.setMaxVoltage(30)));
 //drive.addErrorFunc(4, LAMBDA(clampPis.set_value(true)));
 drive.move(backward, 33, 3, 50);
 pros::delay(1000);





                   /*{kP, kPa, kI, kIa, kD,  kDa,  kPd}*/
//  drive.setCustomPID({29, 87,  0,  13,  120, 100,  150});
 //drive.setSlew({90,30,80});
//  drive.addErrorFunc(20, LAMBDA(drive.setSlew({0,0,0})));
 //drive.addErrorFunc(20, LAMBDA(drive.setCustomPID({31, 82,  0, 14.5, 100,  227,  0})));
 //drive.addErrorFunc(20, LAMBDA(drive.setMaxVoltage(30)));
 //drive.move(backward, 25, 3, 100);
 
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
    // drive.setPID(1);
    // drive.setSlew(genSlewProfile);
    // drive.move(forward, 5, 2, 50);
    // pros::delay(1000);
    // drive.move(forward, 27, 2, 50);
    // pros::delay(1000);
    // drive.move(forward, 48, 2, 50);
    // pros::delay(1000);
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
