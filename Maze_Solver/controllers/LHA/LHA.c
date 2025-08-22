#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/distance_sensor.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>

#define MAX_SPEED 47.6
#define MAZE_SIZE 16 // standard micromouse maze (16x16)
#define CELL_SIZE 0.25 // 25 cm per cell

// sensors
#define NUM_SENSORS 3
static const char *sensor_names[NUM_SENSORS] = {
    "left ultrasonic sensor", "front ultrasonic sensor", "right ultrasonic sensor"};

WbDeviceTag sensors[NUM_SENSORS];
WbDeviceTag left_motor, right_motor;

// Maze representation
int flood[MAZE_SIZE][MAZE_SIZE]; // flood values (distance to goal)
bool walls[MAZE_SIZE][MAZE_SIZE][4]; // 0=N,1=E,2=S,3=W

// Robot state
int rob_x = 0, rob_y = 0; // starting cell
int rob_dir = 0;           // 0=N,1=E,2=S,3=W

//------------------------------------------------------
// Flood Fill Initialization
//------------------------------------------------------
void init_flood() {
  for (int y = 0; y < MAZE_SIZE; y++) {
    for (int x = 0; x < MAZE_SIZE; x++) {
      flood[y][x] = abs(x - (MAZE_SIZE/2)) + abs(y - (MAZE_SIZE/2)); // Manhattan distance to goal (center)
      for (int d = 0; d < 4; d++) walls[y][x][d] = false;
    }
  }
}

//------------------------------------------------------
// Sensor -> Wall detection (approx)
//------------------------------------------------------
void update_walls_from_sensors() {
  double left = wb_distance_sensor_get_value(sensors[0])*100;
  double front = wb_distance_sensor_get_value(sensors[1])*100;
  double right = wb_distance_sensor_get_value(sensors[2])*100;

  // thresholding (tune experimentally)
  bool wall_left = left < 10.0;
  bool wall_front = front < 10.0;
  bool wall_right = right < 10.0;

  int cx = rob_x, cy = rob_y;

  if (rob_dir == 0) { // facing north
    if (wall_front) walls[cy][cx][0] = true;
    if (wall_left)  walls[cy][cx][3] = true;
    if (wall_right) walls[cy][cx][1] = true;
  } else if (rob_dir == 1) { // east
    if (wall_front) walls[cy][cx][1] = true;
    if (wall_left)  walls[cy][cx][0] = true;
    if (wall_right) walls[cy][cx][2] = true;
  } else if (rob_dir == 2) { // south
    if (wall_front) walls[cy][cx][2] = true;
    if (wall_left)  walls[cy][cx][1] = true;
    if (wall_right) walls[cy][cx][3] = true;
  } else if (rob_dir == 3) { // west
    if (wall_front) walls[cy][cx][3] = true;
    if (wall_left)  walls[cy][cx][2] = true;
    if (wall_right) walls[cy][cx][0] = true;
  }
}

//------------------------------------------------------
// Flood Fill Update
//------------------------------------------------------
void update_flood() {
  bool changed = true;
  while (changed) {
    changed = false;
    for (int y = 0; y < MAZE_SIZE; y++) {
      for (int x = 0; x < MAZE_SIZE; x++) {
        int minval = 9999;
        for (int d = 0; d < 4; d++) {
          int nx = x + (d==1) - (d==3);
          int ny = y + (d==0) - (d==2);
          if (nx < 0 || nx >= MAZE_SIZE || ny < 0 || ny >= MAZE_SIZE) continue;
          if (!walls[y][x][d]) { // if no wall that direction
            if (flood[ny][nx] < minval)
              minval = flood[ny][nx];
          }
        }
        if (flood[y][x] != minval + 1) {
          flood[y][x] = minval + 1;
          changed = true;
        }
      }
    }
  }
}

//------------------------------------------------------
// Movement primitives
//------------------------------------------------------
void move_forward() {
  double duration = 2500; // ms to move ~1 cell (tune experimentally)
  double elapsed = 0;
  while (elapsed < duration) {
    wb_motor_set_velocity(left_motor, MAX_SPEED*0.5);
    wb_motor_set_velocity(right_motor, MAX_SPEED*0.5);
    wb_robot_step(32);
    elapsed += 32;
  }
  wb_motor_set_velocity(left_motor, 0);
  wb_motor_set_velocity(right_motor, 0);

  // update logical position
  if (rob_dir==0) rob_y++;
  else if (rob_dir==1) rob_x++;
  else if (rob_dir==2) rob_y--;
  else if (rob_dir==3) rob_x--;
}

void turn_left() {
  wb_motor_set_velocity(left_motor, -MAX_SPEED*0.3);
  wb_motor_set_velocity(right_motor, MAX_SPEED*0.3);
  wb_robot_step(700);
  wb_motor_set_velocity(left_motor, 0);
  wb_motor_set_velocity(right_motor, 0);
  rob_dir = (rob_dir+3)%4;
}

void turn_right() {
  wb_motor_set_velocity(left_motor, MAX_SPEED*0.3);
  wb_motor_set_velocity(right_motor, -MAX_SPEED*0.3);
  wb_robot_step(700);
  wb_motor_set_velocity(left_motor, 0);
  wb_motor_set_velocity(right_motor, 0);
  rob_dir = (rob_dir+1)%4;
}

//------------------------------------------------------
// Main control
//------------------------------------------------------
int main(int argc, char **argv) {
  wb_robot_init();
  int time_step = (int)wb_robot_get_basic_time_step();

  // init sensors
  for (int i=0; i<NUM_SENSORS; i++) {
    sensors[i] = wb_robot_get_device(sensor_names[i]);
    wb_distance_sensor_enable(sensors[i], time_step);
  }

  // init motors
  left_motor = wb_robot_get_device("left wheel motor");
  right_motor = wb_robot_get_device("right wheel motor");
  wb_motor_set_position(left_motor, INFINITY);
  wb_motor_set_position(right_motor, INFINITY);
  wb_motor_set_velocity(left_motor, 0.0);
  wb_motor_set_velocity(right_motor, 0.0);

  init_flood();
  while (wb_robot_step(time_step) != -1) {
    update_walls_from_sensors();
    update_flood();

    // choose neighbor with lowest flood value
    int best_dir = -1, best_val = 9999;
   
    for (int d=0; d<4; d++) {
      int nx = rob_x + (d==1) - (d==3);
      int ny = rob_y + (d==0) - (d==2);
      if (nx < 0 || nx >= MAZE_SIZE || ny < 0 || ny >= MAZE_SIZE) continue;
      if (!walls[rob_y][rob_x][d]) {
        if (flood[ny][nx] < best_val) {
          best_val = flood[ny][nx];
          best_dir = d;
        }
      }
    }
     printf("%d\n",best_dir);
    if (best_dir == -1) {
      // no moves → stay idle but keep running
      wb_motor_set_velocity(left_motor, 0);
      wb_motor_set_velocity(right_motor, 0);
      continue;
    }
    
    // rotate gradually until aligned
    while (rob_dir != best_dir) {
      int diff = (best_dir - rob_dir + 4) % 4;
      if (diff == 1) turn_right();
      else if (diff == 3) turn_left();
      else { turn_left(); turn_left(); }
      wb_robot_step(time_step);
    }

    // move one cell forward
    move_forward();
}}
