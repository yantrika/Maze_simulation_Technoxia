#include <webots/robot.h>
#include <webots/distance_sensor.h>
#include <webots/motor.h>

#define TIME_STEP 32  // in ms
#define MAX_SPEED 6.28  // adjust as needed

int main() {
  wb_robot_init();

  WbDeviceTag ds_left = wb_robot_get_device("ds_left");   // sensor names must match your Webots model
  WbDeviceTag ds_right = wb_robot_get_device("ds_right");
  wb_distance_sensor_enable(ds_left, TIME_STEP);
  wb_distance_sensor_enable(ds_right, TIME_STEP);

  WbDeviceTag left_motor = wb_robot_get_device("left wheel motor");
  WbDeviceTag right_motor = wb_robot_get_device("right wheel motor");
  wb_motor_set_position(left_motor, INFINITY);
  wb_motor_set_position(right_motor, INFINITY);
  wb_motor_set_velocity(left_motor, 0.0);
  wb_motor_set_velocity(right_motor, 0.0);
  while (wb_robot_step(TIME_STEP) != -1) {
    double val_left = wb_distance_sensor_get_value(ds_left);
    double val_right = wb_distance_sensor_get_value(ds_right);

    // Convert from meters to cm (if needed)
    double dist_left = val_left * 100.0;
    double dist_right = val_right * 100.0;

    double v_left = MAX_SPEED;
    double v_right = MAX_SPEED;

    // Example logic: if left front detects wall (within, say, 10 cm), turn right
    if (dist_left < 10.0) {
      v_left = 0.5 * MAX_SPEED;
      v_right = MAX_SPEED;
    }
    // If right front detects wall, turn left
    else if (dist_right < 10.0) {
      v_left = MAX_SPEED;
      v_right = 0.5 * MAX_SPEED;
    }
    // If both clear, go straight
    else {
      v_left = MAX_SPEED;
      v_right = MAX_SPEED;
    }
    wb_motor_set_velocity(left_motor, v_left);
    wb_motor_set_velocity(right_motor, v_right);
  }

  wb_robot_cleanup();
  return 0;
}
