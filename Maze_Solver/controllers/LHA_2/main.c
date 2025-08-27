#include "config.h"
#include "movement.h"

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

  wb_robot_step(TIME_STEP); // stabilize encoders

  int followLeft = 1; 
  bool left_Wall1;

  while (wb_robot_step(TIME_STEP) != -1) {
    double leftSensor = getDistance(ds_left);
    double rightSensor = getDistance(ds_right);
    double frontSensor = getDistance(ds_front);

    int leftWall = (leftSensor < WALL_THRESHOLD);
    int rightWall = (rightSensor < WALL_THRESHOLD);
    int frontWall = (frontSensor < FRONT_THRESHOLD);

    printf("L: %.3f  R: %.3f  F: %.3f  Error: %.3f\n",
           leftSensor, rightSensor, frontSensor, totalError);

    // ========== Decision logic ==========
    // (copy-paste your IF/ELSE movement logic here from original main)

    // Example: PID fallback
    if (!(leftWall && !frontWall && !rightWall)) {
      PID_control(leftSensor, rightSensor, followLeft);
    }
  }

  wb_robot_cleanup();
  return 0;
}
