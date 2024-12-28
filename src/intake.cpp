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
    }
    else if (color ==  blue) {
        lookingRed = false;
        lookingBlue = true;
    }
    else if(color == std::nullopt) { //explicit for eliminating error while testing 
        lookingRed = false; 
        lookingBlue = false;
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
            if(optical.get_hue()>90 && optical.get_proximity()>120)
            {
                detectCycles++;
            }
            if(detectCycles >= blueDetectThreshold){++intakeFlag;}
            break;
            //ring is blue
            case 1:
            pros::delay(5);
            intake.move_voltage(0); 
            pros::delay(500);
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
            if(optical.get_hue()<80 && optical.get_proximity()>120)
            {
                detectCycles++;
            }
            if(detectCycles >= redDetectThreshold){++intakeFlag;}
            break;
            //ring is red
            case 1:
            pros::delay(5);
            intake.move_voltage(0); 
            pros::delay(500);

            pros::lcd::print(4, "Intake flag: %i", intakeFlag);
            noDetectCycles = 0; detectCycles = 0; intakeFlag = 0;

        }
     }  else {
            switch(intakeFlag){
                //Intake is not jammed or in the process of unjamming
                case 0:
                deadCycles = 0;
                reverseCycles = 0;
                intake.move_voltage(intakeSpeed);
                if((intakeVel - intake.get_actual_velocity()) > jamThresh){jamCycles++;}
                else{lastJamDead = 0; jamCycles = 0;}

                if(jamCycles >= jamCycleThreshold){intakeFlag = 1 + lastJamDead;}
                break;

                //Stop running the Intake to see if rings fall naturally and unjam
                case 1:
                jamCycles = 0;
                intake.move_voltage(0);
                deadCycles++;
                if(deadCycles >= dejamThreshold){deadCycles = 0; intakeFlag = 0; lastJamDead = 1;}
                break;

                //Stopping the Intake didn't fix the jam and rings will be run backward
                case 2:
                jamCycles = 0;
                intake.move_voltage(jamSpeed);
                reverseCycles++;
                if(reverseCycles >= dejamThreshold){reverseCycles = 0; intakeFlag = 0; lastJamDead = 0;}
                break;
           }
       }
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