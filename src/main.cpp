#include "drive.hpp"
#include "include.hpp"
#include "autons.hpp"
#include "pros/llemu.hpp"
#include "pros/misc.h"
#include "pros/motors.h"
#include "pros/rotation.hpp"
#include "pros/rtos.hpp"
#include "util.hpp"
#include "intake.hpp"
#include "arm.hpp"
uint auton = AUTO_COUNT; 

void initialize(){
	//initBarGraph();
	//pros::Task brainDisplayTask(updateBarGraph_fn);
  pros::lcd::initialize();
  drive.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
  arm.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  intake.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
  optical.set_led_pwm(100);
  imu.reset();
  
} 

void disabled(){
	while(true){
		//Change auton value
		if(controller.get_digital_new_press(DIGITAL_LEFT)){auton--;}
		if(controller.get_digital_new_press(DIGITAL_RIGHT)){auton++;}
		pros::delay(20);
	}
}

void competition_initialize(){
	while(true){
		//Change auton value
		if(controller.get_digital_new_press(DIGITAL_LEFT)){auton--;}
		if(controller.get_digital_new_press(DIGITAL_RIGHT)){auton++;}
		pros::delay(20);
	}
}

void autonomous(){
  /* Run the auton currently selected and displayed */
	autos[auton%AUTO_COUNT].autonomous();
}

void set_tank(int l_stick, int r_stick){
  leftMotors.move_voltage(l_stick * (12000.0 / 127.0));
  rightMotors.move_voltage(r_stick * (12000.0 / 127.0));
}

double left_curve_function(double x, double left_curve_scale){
  if (left_curve_scale != 0) {
    return (powf(2.718, -(left_curve_scale / 10)) + powf(2.718, (fabs(x) - 127) / 10) * (1 - powf(2.718, -(left_curve_scale / 10)))) * x;
  }
  return x;
}

void arcade_standard(double curve) {

 int fwd_stick, turn_stick;

 // Put the joysticks through the curve function
 fwd_stick = left_curve_function(controller.get_analog(ANALOG_LEFT_Y), curve);
 turn_stick = left_curve_function(controller.get_analog(ANALOG_RIGHT_X), curve);

 // Set robot to l_stick and r_stick, check joystick threshold, set active brake
 set_tank(fwd_stick + turn_stick, fwd_stick - turn_stick);
}


void opcontrol() {
 /*Toggles*/
 bool backClampTog = false;
 bool sortTog = false;
 bool mogoArmTog = false;
 bool intakePisTog = false;
 int armTog = 0;

 optical.set_led_pwm(100);

 arm.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
 pros::Task runIntakeControl(IntakeControlSystem_fn);
 pros::Task armControlTask(armControl_fn);
 runIntakeControl.suspend();

 while (true) {
   //pros::lcd::print(0, "Hue Val (w/ brightness mult): %.2f", optical.get_hue()*optical.get_brightness());
   pros::lcd::print(0, "Hue Val: %.2f", optical.get_hue());
   
   pros::lcd::print(1, "Led Val: %.2f", optical.get_led_pwm());

   pros::lcd::print(2, "Arm Pot (deg): %i", armPot.get_angle()/100);

   pros::lcd::print(4, "Arm Tog Val: %i", armTog);
   
   /*Display current autonomous on the controller*/
   controllerPrintAuto();

   /*Change auton value*/
   if(controller.get_digital_new_press(DIGITAL_LEFT)){auton--;}
   if(controller.get_digital_new_press(DIGITAL_RIGHT)){auton++;}
     
   /*Run the currently selected autonomous when UP is pressed*/
   if(controller.get_digital_new_press(DIGITAL_DOWN)){autonomous();}
      
   /*Reset all sensors*/
   if(controller.get_digital_new_press(DIGITAL_UP)){
    pauseAndCalibrateIMU();
   } 
   
   setIntake(400, currentColor); //color sort whatever color we arenn't bases on auto by default sorts out blue

   /*DRIVER CONTROL */
   arcade_standard(5); 

   if(controller.get_digital_new_press(DIGITAL_A)){
    mogoArmTog = !mogoArmTog;
   }
   if(mogoArmTog){
    mogoArm.set_value(true);
   } else{mogoArm.set_value(false);}
   
   if(controller.get_digital_new_press(DIGITAL_B)){
     sortTog = !sortTog;
   }
   if(sortTog){
    runIntakeControl.resume();
    startIntake();
   }
   else{ 
     runIntakeControl.suspend();
     stopIntake(); 
    }
   if(!sortTog){
     if (controller.get_digital(DIGITAL_L1)){
       intake.move_voltage(12000);
     }
     else if (controller.get_digital(DIGITAL_L2)){
       intake.move_voltage(-12000);
     } else {
      intake.move_voltage(0);
     }
   }

    if(controller.get_digital_new_press(DIGITAL_R2)){
      ++armTog;
      armTog = armTog>2 ? 0 : armTog;
    }
    if(armTog == 0){
      armControl.setTarget(standby);
    }
    else if (armTog == 1) {
      armControl.setTarget(load);
    }
    else if (armTog == 2){
      armControl.setTarget(score);
    }
   
   if(controller.get_digital_new_press(DIGITAL_R1)){ backClampTog = !backClampTog;}
   if (!backClampTog){
      clampPis.set_value(false);
    } else {
      clampPis.set_value(true);
    }

   pros::delay(20);
 }
 runIntakeControl.remove();
 armControlTask.remove();
}