#include "drive.hpp"
#include "include.hpp"
#include "pros/rtos.hpp"
#include "autons.hpp"
#include "intake.hpp"
#include "util.hpp"
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
slewProfile mogoProfile{90, 30, 70};
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
 tiltPis.set_value(true);
 pros::delay(100);
 
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
 tiltPis.set_value(false);
 pros::delay(150);
 
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
 tiltPis.set_value(true);
 pros::delay(100);
 
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
 tiltPis.set_value(false);
 pros::delay(150);

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
 tiltPis.set_value(true);
 pros::delay(100);

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

 drive.move(forward, 35, 3, 50);
 

 clampPis.set_value(true);
 pros::delay(200);
 tiltPis.set_value(false);
 pros::delay(150);
 
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
 clampPis.set_value(false);
 tiltPis.set_value(true);
 pros::delay(100);

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

 drive.move(forward, 35, 3, 50);
 
 clampPis.set_value(true);
 pros::delay(200);
 tiltPis.set_value(false);
 pros::delay(150);

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
 tiltPis.set_value(true);
 pros::delay(100);

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
 tiltPis.set_value(false);
 pros::delay(150);
 
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
 tiltPis.set_value(true);
 pros::delay(100);

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
 tiltPis.set_value(false);
 pros::delay(150);

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

 pros::delay(200);

 setIntake(400, std::nullopt);
 startIntake();
 
 drive.moveDriveVoltage(-6500); 
 pros::delay(800);
 drive.moveDriveVoltage(0);

                  /*{kP, kPa, kI, kIa, kD,  kDa,  kPd}*/
 drive.setCustomPID({18, 87,  0,  10,  30,   90,  0});
 drive.move(forward, 14.5, 1, 100);
 
 drive.setPID(1);
 drive.turn(right, imuTarget(90), 1, 70);

 drive.move(backward, 16, 1, 100);

 clampPis.set_value(false);
 pros::delay(250);
 clampPis.set_value(false);
 tiltPis.set_value(true);
 pros::delay(100);
 
 // Get rings
 drive.setPID(2);
 drive.move(backward, 8, 1, 100);
 
 drive.setPID(2);
 drive.turn(left, imuTarget(0), 1, 90);
 
 drive.move(forward, 24, 1, 100);

 drive.turn(left, imuTarget(270), 1, 90);

 pros::delay(400);
 
 drive.move(forward, 24.5, 1, 100);

 pros::delay(400);
 
 //go to ring in front of wall stake 
 drive.setPID(2);
 drive.turn(right, imuTarget(331.5), 1, 90);

 findTri(&tri, 15, 331);
 
 drive.addErrorFunc(tri.hyp-10, LAMBDA(drive.setMaxVoltage(70)));
 drive.move(forward, tri.hyp, 3, 100);

 pros::delay(400);

 drive.move(backward, 17-tri.b,  1, 100);

 drive.turn(left, imuTarget(209), 1, 100);

 pros::delay(400);
 
 //out of line ring
 findTri(&tri, 18.5, 209);

 drive.move(forward, tri.hyp, 1, 100);

 pros::delay(500);

 drive.move(backward, 18-tri.b, 1, 100); 
 
 //get last two rings onto 1st mogo
 drive.setPID(5);  
 drive.turn(left, imuTarget(180), 2, 70);

 drive.move(forward, 18, 1, 100);
 pros::delay(500);

 drive.move(forward, 12, 1, 100);
 pros::delay(700);
  
 //drop off 1st mogo
 drive.setPID(2);
 drive.turn(left, imuTarget(65), 1, 90);

 findTri(&tri, 10, 65);
 drive.move(backward, tri.hyp, 1, 100);

 clampPis.set_value(true);
 pros::delay(300);
 tiltPis.set_value(false);
 pros::delay(250);

 drive.setPID(1); 
 
 //Go to 2nd mogo 
 drive.move(forward, 35-tri.b, 3, 100);

 drive.turn(left, imuTarget(270), 2, 70);

 drive.setPID(1); 
 drive.addErrorFunc(28, LAMBDA(drive.setMaxVoltage(50)));
 drive.move(backward, 46, 10, 100);

 clampPis.set_value(false);
 pros::delay(300);
 tiltPis.set_value(true);
 pros::delay(100);
 
 drive.setPID(2);
 findTri(&tri, 7, 270);
 drive.move(backward, tri.hyp, 1, 100);
 
 //Get offset ring
 drive.turn(left, imuTarget(116), 1, 90);

 drive.move(forward, 18-tri.b, 1, 100);
 pros::delay(300);

 //in line rings
 drive.move(backward, 23, 1, 100);

                  /*{kP, kPa, kI, kIa, kD,  kDa,  kPd}*/
 drive.setCustomPID({20, 230,  0, 0,   36,  490,  0});
 drive.turn(left, imuTarget(90), 1, 70); // 25 deg turn 

 drive.move(forward, 18, 1, 100);
 pros::delay(600);

 drive.move(forward, 12, 1, 100);
 pros::delay(300);
 
 drive.move(backward, 4, 1, 100);
 pros::delay(300);
 
 drive.setPID(2);
 drive.turn(left, imuTarget(0), 1, 90);
 
 drive.addErrorFunc(12, LAMBDA(drive.setMaxVoltage(70)));
 drive.move(forward, 21, 2, 100);
 
 drive.turn(left, imuTarget(270), 1, 90);
 
 drive.move(forward, 28, 2, 100);
 
 //get ring under climbing tower 
 drive.setPID(3);
 drive.turn(right, imuTarget(313), 1, 90);

 drive.setPID(2);
 drive.move(forward, 24, 1, 100);
 pros::delay(200);

 drive.setPID(3);
 drive.turn(left, imuTarget(300), 1, 90);
 
 //drop off 2nd mogo mech
 findTri(&tri, 71, 313);
                  /*{kP, kPa, kI, kIa, kD,  kDa,  kPd}*/
 drive.setCustomPID({16, 442,  0,   0, 50,  1000, 300});
 //drive.swerve(backwardShortest, tri.hyp, 315, 3, 100, 50);
 drive.move(backward, tri.hyp, 3, 100);

 pros::delay(1000);
 
 clampPis.set_value(true);
 pros::delay(300);
 tiltPis.set_value(false);
 pros::delay(250);
 
//  if(imu.get_heading()<314){
//     drive.move(forward, 4-tri.b, 1, 100);
//     drive.setPID(8);
//     drive.turn(right, imuTarget(320), 1, 100); //~12-30 error ?
//   } else if (imu.get_heading()>340) {
//     drive.move(forward, 4-tri.b, 1, 100);
//     drive.setPID(8);
//     drive.turn(left, imuTarget(320), 1, 100);
//   }


 //Go to 3rd mogo
 findTri(&tri, 148, 313); 
                    /*{kP, kPa, kI, kIa, kD,  kDa,  kPd}*/
 drive.setCustomPID({14, 442,  0,   0, 30,  1000, 300});
 drive.move(forward, tri.hyp, 7, 100); 

//  //drive.move(forward, tri.hyp-76, 7, 70);  //72 in with 0 error

//                   /*{kP, kPa, kI, kIa, kD,  kDa,  kPd}*/
//  drive.setCustomPID({14, 97,  0,  10,  30,   90,  0}),
//  drive.addErrorFunc(25, LAMBDA(stopIntake())); //try and hold 1-2 rings in the intake 
//  drive.turn(left, imuTarget(268), 1, 70); //47 deg turn

//  //get 3rd mogo
//  drive.addErrorFunc(28-tri.b, LAMBDA(drive.setMaxVoltage(50)));
//  drive.move(backward, 35.5-tri.b, 4, 100);

//  clampPis.set_value(false);
//  pros::delay(250);
//  clampPis.set_value(false);
//  tiltPis.set_value(true);
//  pros::delay(100);

//  startIntake();
 
//  // Get 3 right side rings
//  drive.setPID(2);
//  drive.move(backward, 28.5, 2, 100);

//  drive.turn(left, imuTarget(180), 1, 90);

//  drive.move(forward, 24, 1, 100);
//  pros::delay(150);

//  drive.turn(left, imuTarget(90), 1, 90);

//  drive.move(forward, 26, 1, 100); 
//  pros::delay(150);
 
//  drive.setPID(3);
//  drive.turn(right, imuTarget(140), 1, 90);

//  drive.move(forward, 21, 1, 100);

//  drive.turn(right, imuTarget(180), 1, 90);

//  findTri(&tri, 54, 180);
//  drive.move(backward, tri.hyp, 5, 100);
 
//  clampPis.set_value(true);
//  pros::delay(250);
//  tiltPis.set_value(false);
//  pros::delay(100);
 
//  //go to push 4th mogo 
//  drive.setPID(1);
//  drive.move(forward, 20-tri.b, 1, 100);

//  drive.turn(left,imuTarget(90), 1, 100);

//  drive.move(backward, 50, 1, 100);

//  drive.turn(right, imuTarget(180), 1, 70);

//  drive.move(backward, 16, 1, 100);

//  drive.turn(left, imuTarget(90), 1, 70);

//  drive.moveDriveTrain(-12000, 1.3);

//  drive.moveDriveTrain(12000, .5);

 runOnError.remove();
 runIntakeControl.remove();
 drive.onErrorVector.clear();

}

void tune(){
 pros::Task runOnError(onError_fn);
                     
                  /*{kP, kPa, kI, kIa, kD,  kDa,  kPd}*/
 drive.setCustomPID({20, 220,  0,  10,  30, 400,  0}), /*10-30 deg turn (100%) / gen lat*/
 drive.turn(right, 10, 1, 100); //~12-30 error ?
 pros::delay(1000);
 drive.turn(right, 20, 1, 100);
 pros::delay(1000);
 drive.turn(right, 30, 1, 100);
 pros::delay(1000);

 runOnError.remove();
 drive.onErrorVector.clear();
}
