#include "movement.h"

// Globals defined here
double prev_left_distance = 0.0;  
const double Kp = 0.5;            
float P = 0.7, I = 0.4, D = 0.5;
float oldErrorP = 0.0;
float errorI = 0.0;
float totalError = 0.0;
int offset = 1;   // wall-following offset (0.1 but int type was weird, better use 1 → 0.1f)

// Devices
WbDeviceTag left_motor, right_motor;
WbDeviceTag ds_left, ds_right, ds_front;
WbDeviceTag left_sensor, right_sensor;

// ================== SENSOR READ ==================
double getDistance(WbDeviceTag sensor) {
  double val = wb_distance_sensor_get_value(sensor);
  return (val == 0) ? 0.2 : val * 0.001;
}

// ================== PID ==================
void PID_control(double leftSensor, double rightSensor, int followLeft) {
  float errorP;

  if (followLeft)
    errorP = (leftSensor - rightSensor) - offset;
  else
    errorP = (leftSensor - rightSensor) + offset;

  float errorD = errorP - oldErrorP;
  errorI = (2.0f / 3.0f) * errorI + errorP;
  totalError = P * errorP + D * errorD + I * errorI;

  oldErrorP = errorP;

  // Motor speeds
  double RMS = BASE_SPEED + totalError;
  double LMS = BASE_SPEED - totalError;

  // Clamp
  if (RMS > 6.0) RMS = 6.0;
  if (RMS < -6.0) RMS = -6.0;
  if (LMS > 6.0) LMS = 6.0;
  if (LMS < -6.0) LMS = -6.0;

  wb_motor_set_velocity(left_motor, LMS);
  wb_motor_set_velocity(right_motor, RMS);
}
