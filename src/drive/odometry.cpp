#include "drive.hpp"
#include "pros/rtos.hpp"
#include <utility>

void Drive::startOdom(){
    if(odomTask == nullptr){
        odomTask = new pros::Task{[this]{
            odom.updatePose(); 
            pros::delay(20);
        }};
    }
}
double Drive::moveTo(Direction dir, std::pair<double, double> coord, double timeOut, double maxVelocity){
    std::pair<double, double> initCoord = odom.getCoord();

    while (true) {
    
    }
    return 0;
}

double Drive::turnTo(Direction dir, std::pair<double, double> coord, double timeOut, double maxVelocity){
    return 0;
}

double Drive::swerveTo(Direction dir, std::tuple<double, double, double> pose, double maxVel, double maxVel_a){
    return 0;
}

std::pair<double, double> Odometry::getCoord(){
    return this->currentCoord;
}

double Odometry::getTheta(){
    return this->theta;
}
