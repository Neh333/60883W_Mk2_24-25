#include "intake.hpp"
#include "include.hpp"
#include "pros/llemu.hpp"
#include "pros/rtos.hpp"
#include <optional>

bool JAM_PROTECTION_ACTIVE = true;
bool JAM_PROTECTION_INACTIVE = false;

pros::Mutex ringControlMutex;

void IntakeControl::startIntake(){
    isRunning = true;
}

void IntakeControl::stopIntake(){
    notifyOff = true;
}

void IntakeControl::setIntake(int16_t velocity, std::optional<autoColor> color){
    //Input will range from -400 to 400, voltage ranges from -12000 to 12000
    //12000 / 400 = 30
    intakeSpeed = velocity*30;
    intakeVel = velocity/2;
    
    optical.set_led_pwm(100);
    if (color == red) {
        lookingRed = true;
        lookingBlue = false;    
        lookingAny = false;
    }
    else if (color ==  blue) {
        lookingRed = false;
        lookingBlue = true;
        lookingAny = false;
    }
    else if(color == std::nullopt) { //explicit for eliminating error while testing 
        lookingRed = false; 
        lookingRed = false;
        lookingAny = false;
        optical.set_led_pwm(0);
    }
}

void IntakeControl::setJamThresh(int16_t velocity){
    //Input will range from -400 to 400, getVelocity returns between -200 and 200
    jamThresh = velocity/2;
}

void IntakeControl::setJamSpeed(int16_t velocity){
    //Input will range from -400 to 400, voltage ranges from -12000 to 12000
    //12000 / 400 = 30
    jamSpeed = velocity*30;
}

void IntakeControl::setJamProtection(bool state){
    jamProtection = state;
}


void IntakeControl::run(){
    ringControlMutex.take();
    if(notifyOff){
        notifyOff = false;
        isRunning = false;
        intake.move_voltage(0);
    }
    if(isRunning){
        //controller.print(2,0,"%i, %i, %i", jamCycles, deadCycles, reverseCycles) 
     if(lookingBlue){ 
        auto lookingBlueVal = lookingBlue ? "True" : "False";
        pros::lcd::print(5, "Looking Blue: %s", lookingBlueVal);
        switch (intakeFlag) {
            //Rings are intaking as normal 
            case 0:
            intake.move_voltage(intakeSpeed);
            pros::lcd::print(4, "Intake flag: %i", intakeFlag);
            if(optical.get_hue()>120)
            {
                detectCycles++;
            }
            if(detectCycles >= blueDetectThreshold){++intakeFlag;}
            break;
            //ring is blue
            case 1:
            intake.move_voltage(-12000); 
            pros::delay(300);
            pros::lcd::print(4, "Intake flag: %i", intakeFlag);
            noDetectCycles = 0; detectCycles = 0; intakeFlag = 0;
        }
     }
     else if (lookingRed) {    
        auto lookingRedVal = lookingRed ? "True" : "False";
        pros::lcd::print(5, "Looking Red: %s", lookingRedVal);
        switch (intakeFlag) {
            //No rings are past redirect 
            case 0:
            intake.move_voltage(intakeSpeed);
            pros::lcd::print(4, "Intake flag: %i", intakeFlag);
            if(optical.get_hue()<34)
            {
                detectCycles++;
            }
            if(detectCycles >= redDetectThreshold){++intakeFlag;}
            break;
            //ring is red
            case 1:
            intake.move_voltage(-12000); 
            pros::delay(300);
            pros::lcd::print(4, "Intake flag: %i", intakeFlag);
            noDetectCycles = 0; detectCycles = 0; intakeFlag = 0;

        }
     } else {intake.move_voltage(intakeSpeed);}
    }
   ringControlMutex.give();
}

//Set the Intakes speed to run at when operating
void setIntake(int16_t velocity, std::optional<autoColor> color){conveyor.setIntake(velocity, color);}
//Begin running the Intake at the previously specified speed
void startIntake(){conveyor.startIntake();}
//Stop running the Intake (Does not continously run at zero, rather set speed to zero then quits)
void stopIntake(){conveyor.stopIntake();}

void IntakeControlSystem_fn(void *param){
    uint32_t startTime = pros::millis();
    while(true){
        conveyor.run();
        pros::Task::delay_until(&startTime, 20);
    }
}