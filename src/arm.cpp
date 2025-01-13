#include "arm.hpp"
#include "include.hpp"
#include <algorithm>
#include <cmath>

Arm armControl; //arm object for main.cpp and autons.cpp for changing the arm target

void Arm::setTarget(armState state){
    if(state == standby){
        this->target = standByTarget;
    } else if(state == load){
        this->target = loadingTarget;
    } else if (state == inter) {
        this->target = loadingTarget;
    } else {
        this->target = scoringTarget;
    }
}

void Arm::move(){
    this->error = target - (float)armPot.get_angle()/100;
    
    pros::lcd::print(3, "Arm PID Error : %.2d", this->error);
    
    float proportion = error;
    if(target == scoringTarget){
        this->kP = 150; //74
    }
    else if (target == loadingTarget || target == interTarget) {
        this->kP = 280;
    }

    if(fabs(error) <= intergralActive){intergral += error;}
    else{intergral = 0;}
    intergral = std::clamp(intergral, -intergralLimit, intergralLimit);

    float derivative = error - lastError;
    lastError = error;
    
    pros::lcd::print(3, "Arm PID :ast Error : %.2d", this->lastError);
    
    if(error == 0){derivative = 0;}

    int finalVolt = kP*proportion + kI*intergral + kD*derivative;

    //Set finalVolt to range
    finalVolt = std::clamp(finalVolt, -12000, 12000);

    //Set final lift speeds
    arm.move_voltage(-finalVolt);
}

void armControl_fn(void *param){
    uint32_t startTime = pros::millis();
    while(true){
        armControl.move();
        pros::Task::delay_until(&startTime, 20);
    }
}