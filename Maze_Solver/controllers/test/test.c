#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/distance_sensor.h>
#include <webots/position_sensor.h>
#include <stdio.h>
#include <math.h>

// ================== CONFIG ==================
#define TIME_STEP 64
#define BASE_SPEED 2.0   // Base forward speed
#define WALL_THRESHOLD 0.09   // meters (13 cm)
#define FRONT_THRESHOLD 0.1  // meters (7 cm)

// Robot dimensions
#define WHEEL_RADIUS 0.03   // 3 cm
#define AXLE_LENGTH 0.16   // 10 cm

// PID coefficients
float P = 0.7, I = 0.4, D = 0.5;
float oldErrorP = 0.0;
float errorI = 0.0;
float totalError = 0.0;
int offset = 0;   // wall-following offset

// ================== DEVICES ==================
WbDeviceTag left_motor, right_motor;
WbDeviceTag ds_left, ds_right, ds_front;
WbDeviceTag left_sensor,right_sensor;

// ================== SENSOR READ ==================
double getDistance(WbDeviceTag sensor) {
  double val = wb_distance_sensor_get_value(sensor);
  // Convert to meters (assuming maxRange ≈ 0.2 m for IR/ultrasonic in Webots)
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
  errorI = (2.0 / 3.0) * errorI + errorP;
  totalError = P * errorP + D * errorD + I * errorI;

  oldErrorP = errorP;

  // Motor speeds
  double RMS = BASE_SPEED + totalError;
  double LMS = BASE_SPEED - totalError;

  // Clamp speeds
  if (RMS > 6.0) RMS = 6.0;
  if (RMS < -6.0) RMS = -6.0;
  if (LMS > 6.0) LMS = 6.0;
  if (LMS < -6.0) LMS = -6.0;

  wb_motor_set_velocity(left_motor, LMS);
  wb_motor_set_velocity(right_motor, RMS);
}

// ================== MAIN ==================
int main() {
  wb_robot_init();

  // Motors
  left_motor = wb_robot_get_device("m2");
  right_motor = wb_robot_get_device("m1");
  wb_motor_set_position(left_motor, INFINITY);
  wb_motor_set_position(right_motor, INFINITY);
  wb_motor_set_velocity(left_motor, 0.0);
  wb_motor_set_velocity(right_motor, 0.0);

  // Sensors
  ds_left = wb_robot_get_device("ds_left");
  ds_right = wb_robot_get_device("ds_right");
  ds_front = wb_robot_get_device("ds_front");
  wb_distance_sensor_enable(ds_left, TIME_STEP);
  wb_distance_sensor_enable(ds_right, TIME_STEP);
  wb_distance_sensor_enable(ds_front, TIME_STEP);
  
  // Encoders
  left_sensor = wb_robot_get_device("left_sensor");
  right_sensor = wb_robot_get_device("right_sensor");
  wb_position_sensor_enable(left_sensor, TIME_STEP);
  wb_position_sensor_enable(right_sensor, TIME_STEP);

  // Let encoders stabilize
  wb_robot_step(TIME_STEP);

  // Reset encoders (initial position)
  double left_start = wb_position_sensor_get_value(left_sensor);
  double right_start = wb_position_sensor_get_value(right_sensor);


  int followLeft = 1; // Start with left-wall-follow

  while (wb_robot_step(TIME_STEP) != -1) {
    double leftSensor = getDistance(ds_left);
    double rightSensor = getDistance(ds_right);
    double frontSensor = getDistance(ds_front);

    // Wall detection
    int leftWall = (leftSensor < WALL_THRESHOLD);
    int rightWall = (rightSensor < WALL_THRESHOLD);
    int frontWall = (frontSensor < FRONT_THRESHOLD);

    // Decision
    if (frontWall) {
      // If wall in front, turn away after moving a bit ahead for turning
      // ---------- STEP 1: Move forward ----------
      wb_motor_set_velocity(left_motor, 0.0);
      wb_motor_set_velocity(right_motor, 0.0);
    
      // Move forward until 0.2 m distance
      double target_distance = 0.0; // 20 cm
      double target_rotation = target_distance / WHEEL_RADIUS;
    
      while (wb_robot_step(TIME_STEP) != -1) {
        double left_pos = wb_position_sensor_get_value(left_sensor) - left_start;
        double right_pos = wb_position_sensor_get_value(right_sensor) - right_start;
        double avg_distance = WHEEL_RADIUS * (left_pos + right_pos) / 2.0;
    
        if (avg_distance >= target_distance) {
          wb_motor_set_velocity(left_motor, 0.0);
          wb_motor_set_velocity(right_motor, 0.0);
          printf("Reached forward distance: %.3f m\n", avg_distance);
          break;
        }
      }
      
      wb_robot_step(200); // small pause
      
      // Reset  encoders
      double right_start_turn = wb_position_sensor_get_value(right_sensor);
      double left_start_turn = wb_position_sensor_get_value(left_sensor);
    
      // Required arc length for 90° = (π/2 * axle_length / 2)
      double turn_arc = (M_PI / 2.0) * (AXLE_LENGTH / 2.0);
      double turn_rotation = turn_arc / WHEEL_RADIUS;
      
      printf("%d\t%d",leftWall,rightWall);
      if (leftWall && !rightWall){ //followLeft = 0; // switch to right
          printf("\nsmall pause\n\n");
          wb_motor_set_velocity(left_motor, 5.0);  // left wheel moves
          wb_motor_set_velocity(right_motor, 0.0); // right wheel stopped
          
          while (wb_robot_step(TIME_STEP) != -1) {
            double left_pos = wb_position_sensor_get_value(left_sensor) - left_start_turn;
            if (fabs(left_pos) >= turn_rotation) {
              wb_motor_set_velocity(left_motor, 0.0);
              wb_motor_set_velocity(right_motor, 0.0);
              printf("Completed 90 degree left turn!\n");
              break;
            }
          }
         }

         
      else if (rightWall && !leftWall){ //followLeft = 1; // switch to left
          wb_motor_set_velocity(left_motor, 0.0);  // left wheel stopped
          wb_motor_set_velocity(right_motor, 5.0); // right wheel moves
        
          while (wb_robot_step(TIME_STEP) != -1) {
            double right_pos = wb_position_sensor_get_value(right_sensor) - right_start_turn;
            if (fabs(right_pos) >= turn_rotation) {
              wb_motor_set_velocity(left_motor, 0.0);
              wb_motor_set_velocity(right_motor, 0.0);
              printf("Completed 90 degree left turn!\n");
              break;
            }
          }
         }
       wb_robot_step(500); // pause after turn
    }

    // Apply PID
    PID_control(leftSensor, rightSensor, followLeft);

    // Debug
    printf("L: %.3f  R: %.3f  F: %.3f  Error: %.3f\n",
           leftSensor, rightSensor, frontSensor, totalError);
  }

  wb_robot_cleanup();
  return 0;
}
