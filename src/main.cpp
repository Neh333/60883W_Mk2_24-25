#include "drive.hpp"
#include "include.hpp"
#include "autons.hpp"
#include "util.hpp"
uint8_t auton = AUTO_COUNT; 

// void AUTO_SWITCH(){
// 	controller.print(2, 0, "%2.f", autos[auton%AUTO_COUNT].autoName + "%2.f", imu.get_heading());
// }

// #define AUTO_SWITCH(){ \
// 	switch(auton%AUTO_COUNT){\
//     case 0:   controller.print(2, 0, "WinP Red %.2f                ",imu.get_heading()); break;\
//     case 1:   controller.print(2, 0, "WinP Blue %.2f               ",imu.get_heading()); break;\
// 		case 2:   controller.print(2, 0, "GoalS Red %.2f               ",imu.get_heading()); break;\
// 		case 3:   controller.print(2, 0, "GoalS Blue %.2f              ",imu.get_heading()); break;\
// 		case 4:   controller.print(2, 0, "RingS Red %.2f               ",imu.get_heading()); break;\
// 		case 5:   controller.print(2, 0, "RingS Blue %.2f              ",imu.get_heading()); break;\
// 		case 6:   controller.print(2, 0, "GoalE Blue %.2f              ",imu.get_heading()); break;\
// 		case 7:   controller.print(2, 0, "GoalE Red %.2f               ",imu.get_heading()); break;\
// 		case 8:   controller.print(2, 0, "RingE Blue %.2f              ",imu.get_heading()); break;\
// 		case 9:   controller.print(2, 0, "RingE Red %.2f               ",imu.get_heading()); break;\
//     case 10:  controller.print(2, 0, "Skills      %.2f             ",imu.get_heading()); break;\
// 		case 11:  controller.print(2, 0, "Tune %.2f                    ",imu.get_heading()); break;\
// 	}\
// }\

void initialize(){
	//initBarGraph();
	//pros::Task brainDisplayTask(updateBarGraph_fn);
  pros::lcd::initialize();
  drive.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
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
 //bool tiltTog = true;

 while (true) {
   /*Display current autonomous on the controller*/
   controllerPrintAuto();
   //AUTO_SWITCH();

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
   if (controller.get_digital(DIGITAL_L1)){
     intake.move_voltage(12000);
   }
   else if (controller.get_digital(DIGITAL_L2)){
     intake.move_voltage(-12000);
   } else {
    intake.move_voltage(0);
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
      clampPis.set_value(false);
      pros::delay(100);
      tiltPis.set_value(true);
    } else {
      clampPis.set_value(true);
      tiltPis.set_value(false);
    }
   pros::delay(20);
 }
}