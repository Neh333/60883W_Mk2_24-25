#include "arm.hpp"
#include "include.hpp"
#include <algorithm>
#include <cmath>

Arm armControl;

void Arm::setTarget(armState state){
    if(state == load){
        target = loading;
    } else {
        target = scoring;
    }
}

void Arm::waitUntilTargetReached(float timeOut){
    const uint32_t endTime = pros::millis() + timeOut*1000;
    // while(pros::millis() < endTime && fabs(error) < 3){
    //     pros::delay(10);
    // }
}

void Arm::move(){
    this->error = target - (float)armPot.get_angle()/100;
    
    pros::lcd::print(3, "Arm PID Error : %.2d", this->error);
    
    float proportion = error;

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

    //master.print(2,0,"%.2f, %.0f                ", error, target);

    //Set final lift speeds
    arm.move_voltage(-finalVolt);


//  if(target == scoring){
//   if(armPot.get_angle()/100 < scoring){
//     arm.move(-12000);
//   }
//   else {
//     arm.move_voltage(0);
//   }
//  } 
//  else if (target == loading){
//    if(armPot.get_angle()/100 > loading) {
//      arm.move(4000);
//    }
//    else {
//     arm.move_voltage(0);
//   }
//  }


}

void armControl_fn(void *param){
    uint32_t startTime = pros::millis();
    while(true){
        armControl.move();
        pros::Task::delay_until(&startTime, 20);
    }
}