#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/distance_sensor.h>

#define TIME_STEP 64

#define MAX_SPEED 6.28

int main() {
  wb_robot_init();

  WbDeviceTag left_motor = wb_robot_get_device("left wheel motor");
  WbDeviceTag right_motor = wb_robot_get_device("right wheel motor");

  wb_motor_set_position(left_motor, INFINITY);
  wb_motor_set_position(right_motor, INFINITY);

  wb_motor_set_velocity(left_motor, 0.0);
  wb_motor_set_velocity(right_motor, 0.0);

  // Initialize distance sensors
  WbDeviceTag ir_left = wb_robot_get_device("ps5");   // Left
  WbDeviceTag ir_front = wb_robot_get_device("ps7");  // Front
  WbDeviceTag ir_right = wb_robot_get_device("ps0");  // Right

  wb_distance_sensor_enable(ir_left, TIME_STEP);
  wb_distance_sensor_enable(ir_front, TIME_STEP);
  wb_distance_sensor_enable(ir_right, TIME_STEP);

  while (wb_robot_step(TIME_STEP) != -1) {
    double left = wb_distance_sensor_get_value(ir_left);
    double front = wb_distance_sensor_get_value(ir_front);
    double right = wb_distance_sensor_get_value(ir_right);

    // Thresholds may need tuning based on environment
    int wall_left = left > 80.0;
    int wall_front = front > 80.0;
    int wall_right = right > 80.0;

    double left_speed = MAX_SPEED;
    double right_speed = MAX_SPEED;

    // Left-hand rule logic
    if (!wall_left) {
      // Turn left
      left_speed = 0.2 * MAX_SPEED;
      right_speed = MAX_SPEED;
    } else if (wall_front) {
      // Turn right
      left_speed = MAX_SPEED;
      right_speed = -MAX_SPEED;
    } else {
      // Go straight
      left_speed = MAX_SPEED;
      right_speed = MAX_SPEED;
    }

    wb_motor_set_velocity(left_motor, left_speed);
    wb_motor_set_velocity(right_motor, right_speed);
  }

  wb_robot_cleanup();
  return 0;
}
