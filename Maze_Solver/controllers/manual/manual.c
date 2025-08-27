#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/keyboard.h>
#include <stdio.h>

#define TIME_STEP 64
#define MAX_SPEED 6.0

// Devices
WbDeviceTag left_motor, right_motor;

int main() {
  wb_robot_init();

  // Motors
  left_motor = wb_robot_get_device("m2");
  right_motor = wb_robot_get_device("m1");
  wb_motor_set_position(left_motor, INFINITY);
  wb_motor_set_position(right_motor, INFINITY);
  wb_motor_set_velocity(left_motor, 0.0);
  wb_motor_set_velocity(right_motor, 0.0);

  // Enable keyboard
  wb_keyboard_enable(TIME_STEP);

  while (wb_robot_step(TIME_STEP) != -1) {
    int key = wb_keyboard_get_key();
    double left_speed = 0.0;
    double right_speed = 0.0;

    // WASD control
    switch (key) {
      case 'W': // forward
        left_speed = MAX_SPEED;
        right_speed = MAX_SPEED;
        break;
      case 'S': // backward
        left_speed = -MAX_SPEED;
        right_speed = -MAX_SPEED;
        break;
      case 'A': // turn left
        left_speed = -MAX_SPEED * 0.5;
        right_speed = MAX_SPEED * 0.5;
        break;
      case 'D': // turn right
        left_speed = MAX_SPEED * 0.5;
        right_speed = -MAX_SPEED * 0.5;
        break;
      default: // stop if no key pressed
        left_speed = 0.0;
        right_speed = 0.0;
        break;
    }

    wb_motor_set_velocity(left_motor, left_speed);
    wb_motor_set_velocity(right_motor, right_speed);
  }

  wb_robot_cleanup();
  return 0;
}
