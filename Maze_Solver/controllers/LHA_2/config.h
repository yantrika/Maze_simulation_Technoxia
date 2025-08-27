#ifndef CONFIG_H
#define CONFIG_H

#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/distance_sensor.h>
#include <webots/position_sensor.h>
#include <math.h>
#include <stdio.h>

// ================== CONFIG ==================
#define TIME_STEP 64
#define BASE_SPEED 2.0   
#define WALL_THRESHOLD 0.100   // meters
#define FRONT_THRESHOLD 0.110  // meters

// Robot dimensions
#define WHEEL_RADIUS 0.03      // 3 cm
#define AXLE_LENGTH 0.18       // 16 cm

// PID coefficients
extern float P, I, D;
extern float oldErrorP, errorI, totalError;
extern int offset;

extern double prev_left_distance; 
extern const double Kp;

// ================== DEVICES ==================
extern WbDeviceTag left_motor, right_motor;
extern WbDeviceTag ds_left, ds_right, ds_front;
extern WbDeviceTag left_sensor, right_sensor;

#endif
