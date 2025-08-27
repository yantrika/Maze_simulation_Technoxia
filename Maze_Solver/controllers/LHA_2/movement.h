#ifndef MOVEMENT_H
#define MOVEMENT_H

#include "config.h"

// Sensor read helper
double getDistance(WbDeviceTag sensor);

// PID control
void PID_control(double leftSensor, double rightSensor, int followLeft);

#endif
