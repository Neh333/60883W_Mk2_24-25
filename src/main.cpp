#include "drive.hpp"
#include "include.hpp"
#include "autons.hpp"
#include "pros/llemu.hpp"
#include "pros/misc.h"
#include "pros/motors.h"
#include "util.hpp"
#include "intake.hpp"
uint8_t auton = AUTO_COUNT; 

void initialize(){
	//initBarGraph();
	//pros::Task brainDisplayTask(updateBarGraph_fn);
  pros::lcd::initialize();
  drive.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
  arm.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  imu.reset();
} 

void disabled(){
	while(true){
    //AUTO_SWITCH();
    
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
 bool backClampTog = true;
 bool redirectTog = false;
 
 optical.set_led_pwm(100);

 pros::Task runIntakeControl(IntakeControlSystem_fn);
 runIntakeControl.suspend();
 setIntake(390, any);

 while (true) {
   pros::lcd::print(0, "Hue Val: %.2f", optical.get_hue());
   
   pros::lcd::print(1, "Led Val: %.2f", optical.get_led_pwm());

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

   /*DRIVER CONTROL */
   arcade_standard(5);
   
   if(controller.get_digital_new_press(DIGITAL_B)){
     redirectTog = !redirectTog;
   }
   if(redirectTog){
    runIntakeControl.resume();
    startIntake();
   }
   else{ 
     runIntakeControl.suspend();
     stopIntake(); 
    }
   if(!redirectTog){
     if (controller.get_digital(DIGITAL_L1)){
       intake.move_voltage(12000);
     }
     else if (controller.get_digital(DIGITAL_L2)){
       intake.move_voltage(-12000);
     } else {
      intake.move_voltage(0);
     }
   }

    if (controller.get_digital(DIGITAL_R1)){
     arm.move_voltage(12000);
   } else if (controller.get_digital(DIGITAL_R2)){
     arm.move_voltage(-12000);
   } else {
    arm.move_voltage(0);
   }
   
   if(controller.get_digital_new_press(DIGITAL_A)){ backClampTog = !backClampTog;}
   if (!backClampTog){
      //intakePis.set_value(true);
      clampPis.set_value(false);
      pros::delay(100);
      tiltPis.set_value(true);
    } else {
      //intakePis.set_value(true);
      clampPis.set_value(true);
      tiltPis.set_value(false);
    }
   pros::delay(20);
 }
 runIntakeControl.remove();
}