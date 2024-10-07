#pragma once
#include "pros/colors.hpp"
#include "util.hpp"
#include <cstdint>
#define LAMBDA(func) [](){func;}

int8_t sgn(double x);

double inchToTick(double inch);
double tickToInch(double tick);

double percentToVoltage(double percent);

double voltageToVelocity(double voltage);
double velocityToVoltage(double velocity);

double imuTarget(double target);

double radToDeg(double rad);
double degToRad(double deg);

struct Triangle
{
    double a;
    double b;
    double hyp;
    double alpha;
    double beta;
};

void findTri(struct Triangle *obj, double a, double reference); 

void controllerPrintAuto();

void pauseAndCalibrateIMU();

enum autoColor{
    red,
    blue
};

// void intakeToRedirect(void* param);
void intakeToRedirect(autoColor color);