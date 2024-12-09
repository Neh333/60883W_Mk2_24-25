#include <algorithm>
#include "include.hpp"

enum armState{
    standby,
    load,
    score
};

const int scoringTarget = 232, loadingTarget = 127, standByTarget = 98; //130, 29, 0 

class Arm{
    private:
    //Initialize PID Values
    double kP = 74;
    double kI = 0;
    double kD = 0;
    const double intergralActive = 10;
    const double intergralLimit = 1000;

    //Variables that need to be read after each PID scope is destroyed
    double error = 0;
    double lastError = 0;
    double intergral = 0;

    public: 
    float target = loadingTarget;

    void move();
    void setTarget(armState state);
};

extern Arm armControl;


extern float armTimeOut;
void armControl_fn(void *param);