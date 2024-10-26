#include "drive.hpp"
#include "include.hpp"
#include "pros/motors.h"
#include "pros/rtos.hpp"
#include "autons.hpp"
#include "intake.hpp"
#include "util.hpp"
#include <optional>
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
 
 optical.set_led_pwm(100);
 setIntake(400, std::nullopt);
 startIntake();
 clampPis.set_value(true); // Pull the clamp up 

 pros::delay(100);

 drive.moveDriveVoltage(-6000);
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
 drive.move(backward, 10, 1, 100);
 
 drive.setPID(2);
 drive.turn(left, imuTarget(0), 1, 90);
 
 drive.move(forward, 24, 1, 100);

 drive.turn(left, imuTarget(270), 1, 90);

 drive.move(forward, 24, 1, 100);
 
 //go to ring in front of wall stake 
 drive.setPID(2);
 drive.turn(right, imuTarget(331), 1, 90);

 findTri(&tri, 15, 331);
 
 drive.addErrorFunc(tri.hyp-10, LAMBDA(drive.setMaxVoltage(70)));
 drive.move(forward, tri.hyp, 3, 100);

 pros::delay(400);

 drive.move(backward, 17-tri.b,  1, 100);

 drive.turn(left, imuTarget(210), 1, 100);
 
 //out of line ring
 findTri(&tri, 18, 210);

 drive.move(forward, tri.hyp, 1, 100);

 drive.move(backward, 19-tri.b, 1, 100); 
 
 //get last two rings onto 1st mogo
 drive.setPID(5);  
 drive.turn(left, imuTarget(180), 2, 70);

 drive.move(forward, 22, 1, 100);
 pros::delay(300);

 drive.move(forward, 12, 1, 100);
 pros::delay(200);
  
 //drop off 1st mogo
 drive.setPID(2);
 drive.turn(left, imuTarget(65), 1, 90);

 findTri(&tri, 9, 65);
 drive.move(backward, tri.hyp, 1, 100);

 clampPis.set_value(true);
 pros::delay(200);
 tiltPis.set_value(false);
 pros::delay(150);

 drive.setPID(1); 
 
 //Go to 2nd mogo 
 drive.move(forward, 32.5-tri.b, 1, 100);

 drive.turn(left, imuTarget(272), 2, 70);

 drive.setPID(1); 
 drive.addErrorFunc(28, LAMBDA(drive.setMaxVoltage(50)));
 drive.move(backward, 47.5, 10, 100);

 clampPis.set_value(false);
 pros::delay(250);
 tiltPis.set_value(true);
 pros::delay(100);
 
 drive.setPID(2);
 findTri(&tri, 7, 270);
 drive.move(backward, tri.hyp, 1, 100);
 
 //Get offset ring
 drive.turn(left, imuTarget(115), 1, 90);

 drive.move(forward, 18-tri.b, 1, 100);
 pros::delay(100);

 //in line rings
 drive.move(backward, 24, 1, 100);

                  /*{kP, kPa, kI, kIa, kD,  kDa,  kPd}*/
 drive.setCustomPID({20, 230,  0, 0,   36,  490,  0});
 drive.turn(left, imuTarget(90), 1, 70); // 25 deg turn 

 drive.move(forward, 18, 1, 100);
 pros::delay(200);

 drive.move(forward, 12, 1, 100);
 pros::delay(100);
 
 drive.setPID(2);
 drive.turn(left, imuTarget(0), 1, 90);
 
 drive.addErrorFunc(12, LAMBDA(drive.setMaxVoltage(70)));
 drive.move(forward, 21, 2, 100);
 
 drive.turn(left, imuTarget(270), 1, 90);
 
 drive.move(forward, 24, 2, 100);
 
 //get ring under climbing tower 
 drive.setPID(3);
 drive.turn(right, imuTarget(315), 1, 90);

 drive.setPID(2);
 drive.move(forward, 25, 1, 100);
 pros::delay(200);

 //drop off 2nd mogo mech
 findTri(&tri, 71, 315);
                  /*{kP, kPa, kI, kIa, kD,  kDa,  kPd}*/
 drive.setCustomPID({16, 442,  0,   0, 50,  1000, 300});
//  drive.swerve(backwardShortest, tri.hyp, 315, 3, 100, 50);
 drive.move(backward, tri.hyp, 3, 100);

 clampPis.set_value(true);
 pros::delay(250);
 tiltPis.set_value(false);
 pros::delay(100);

 //Go to 3rd mogo
 findTri(&tri, 146, 315);
 
                    /*{kP, kPa, kI, kIa, kD,  kDa,  kPd}*/
 drive.setCustomPID({16, 442,  0,   0, 50,  1000, 300});
 drive.addErrorFunc(48, LAMBDA(drive.setMaxVoltage(60)));
 drive.move(forward, tri.hyp, 15, 100); 

                  /*{kP, kPa, kI, kIa, kD,  kDa,  kPd}*/
 drive.setCustomPID({14, 97,  0,  10,  30,   90,  0}),
 drive.addErrorFunc(25, LAMBDA(stopIntake()));
 drive.turn(left, imuTarget(268), 1, 70);

 //get 3rd mogo
 drive.addErrorFunc(28-tri.b, LAMBDA(drive.setMaxVoltage(50)));
 drive.move(backward, 38-tri.b, 4, 100);

 clampPis.set_value(false);
 pros::delay(250);
 clampPis.set_value(false);
 tiltPis.set_value(true);
 pros::delay(100);

 startIntake();
 
 // Get 3 right side rings
 drive.setPID(2);
 drive.move(backward, 27, 1, 100);

 drive.turn(left, imuTarget(180), 1, 90);

 drive.move(forward, 24, 1, 100);
 pros::delay(150);

 drive.turn(left, imuTarget(90), 1, 90);

 drive.move(forward, 24, 1, 100); 
 pros::delay(150);
 
 drive.setPID(3);
 drive.turn(right, imuTarget(140), 1, 90);

 drive.move(forward, 18, 1, 100);
 pros::delay(150);

 drive.turn(right, imuTarget(180), 1, 90);

 findTri(&tri, 48, 180);
 drive.move(backward, tri.hyp, 5, 100);
 
 clampPis.set_value(true);
 pros::delay(250);
 tiltPis.set_value(false);
 pros::delay(100);
 
 //go to push 4th mogo 
 drive.setPID(1);
 drive.move(forward, 20-tri.b, 1, 100);

 drive.turn(left,imuTarget(90), 1, 100);

 drive.move(backward, 50, 1, 100);

 drive.turn(right, imuTarget(180), 1, 70);

 drive.move(backward, 16, 1, 100);

 drive.turn(left, imuTarget(90), 1, 70);

 drive.moveDriveTrain(-12000, 1.3);

 drive.moveDriveTrain(12000, .5);

 runOnError.remove();
 runIntakeControl.remove();
 drive.onErrorVector.clear();
}

void tune(){
 pros::Task runOnError(onError_fn);
 
 drive.turn(right,  45, 10, 70);
 pros::delay(1000);

 runOnError.remove();
 drive.onErrorVector.clear();
}
