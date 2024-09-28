#pragma once
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

float wrapAngle(float rad);

void controllerPrintAuto();

void pauseAndCalibrateIMU();