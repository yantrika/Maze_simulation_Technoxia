#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/distance_sensor.h>
#include <webots/position_sensor.h>   // <<< ADDED
#include <stdio.h>
#include <math.h>
#include <webots/inertial_unit.h>

#define MAX_SPEED 6.28 // Adjust to your bot's wheel speed
#define MAZE_SIZE 16
#define CELL_SIZE 0.25      // meters
#define WALL_THRESHOLD 0.07 // 7cm from sensor to wall
#define WHEEL_RADIUS 0.021  // <<< ADDED (meters, Khepera IV approx)

typedef struct {
    int dist;
    unsigned char walls; // bitmask: N=1, E=2, S=4, W=8
    unsigned char visited;
} Cell;

Cell maze[MAZE_SIZE][MAZE_SIZE];

static const char *ultrasonic_names[3] = {
    "left ultrasonic sensor",
    "front ultrasonic sensor",
    "right ultrasonic sensor"};

WbDeviceTag imu;
WbDeviceTag left_motor, right_motor, usensors[3];
WbDeviceTag left_ps, right_ps; // <<< ADDED

int botX = 0, botY = 0, heading = 0; // start bottom-left facing North
int goalX = MAZE_SIZE / 2, goalY = MAZE_SIZE / 2;

// ---------- Flood fill ----------
void flood_fill(int gx, int gy) {
    for (int y = 0; y < MAZE_SIZE; y++)
        for (int x = 0; x < MAZE_SIZE; x++)
            maze[y][x].dist = 255;
    maze[gy][gx].dist = 0;

    int changed = 1;
    while (changed) {
        changed = 0;
        for (int y = 0; y < MAZE_SIZE; y++) {
            for (int x = 0; x < MAZE_SIZE; x++) {
                if (x == gx && y == gy) continue;
                int minN = 255;
                if (!(maze[y][x].walls & 1) && y > 0)
                    if (maze[y - 1][x].dist < minN) minN = maze[y - 1][x].dist;
                if (!(maze[y][x].walls & 2) && x < MAZE_SIZE - 1)
                    if (maze[y][x + 1].dist < minN) minN = maze[y][x + 1].dist;
                if (!(maze[y][x].walls & 4) && y < MAZE_SIZE - 1)
                    if (maze[y + 1][x].dist < minN) minN = maze[y + 1][x].dist;
                if (!(maze[y][x].walls & 8) && x > 0)
                    if (maze[y][x - 1].dist < minN) minN = maze[y][x - 1].dist;

                int newDist = minN + 1;
                if (newDist < maze[y][x].dist) {
                    maze[y][x].dist = newDist;
                    changed = 1;
                }
            }
        }
    }
}

// ---------- Wall updating ----------
void update_walls(double front, double left, double right) {
    if (front < WALL_THRESHOLD)
        maze[botY][botX].walls |= (1 << heading);
    if (left < WALL_THRESHOLD)
        maze[botY][botX].walls |= (1 << ((heading + 3) % 4));
    if (right < WALL_THRESHOLD)
        maze[botY][botX].walls |= (1 << ((heading + 1) % 4));
}

// ---------- Movement ----------
void turn_left(int time_step) {
    const double *rpy = wb_inertial_unit_get_roll_pitch_yaw(imu);
    double target = rpy[2] + M_PI_2;
    if (target > M_PI) target -= 2 * M_PI;

    wb_motor_set_velocity(left_motor, -MAX_SPEED/2);
    wb_motor_set_velocity(right_motor, MAX_SPEED/2);

    while (wb_robot_step(time_step) != -1) {
        const double *ang = wb_inertial_unit_get_roll_pitch_yaw(imu);
        double yaw = ang[2];
        if (fabs(yaw - target) < 0.05) break;
    }

    wb_motor_set_velocity(left_motor, 0);
    wb_motor_set_velocity(right_motor, 0);
    heading = (heading + 3) % 4;
}

void turn_right(int time_step) {
    const double *rpy = wb_inertial_unit_get_roll_pitch_yaw(imu);
    double target = rpy[2] - M_PI_2;
    if (target < -M_PI) target += 2 * M_PI;

    wb_motor_set_velocity(left_motor, MAX_SPEED/2);
    wb_motor_set_velocity(right_motor, -MAX_SPEED/2);

    while (wb_robot_step(time_step) != -1) {
        const double *ang = wb_inertial_unit_get_roll_pitch_yaw(imu);
        double yaw = ang[2];
        if (fabs(yaw - target) < 0.05) break;
    }

    wb_motor_set_velocity(left_motor, 0);
    wb_motor_set_velocity(right_motor, 0);
    heading = (heading + 1) % 4;
}

// <<< REPLACED move_forward with encoder-based
void move_forward_cell(int time_step) {
    double distance = CELL_SIZE;
    double target_rot = distance / WHEEL_RADIUS;

    double left_start = wb_position_sensor_get_value(left_ps);
    double right_start = wb_position_sensor_get_value(right_ps);

    wb_motor_set_velocity(left_motor, MAX_SPEED * 0.5);
    wb_motor_set_velocity(right_motor, MAX_SPEED * 0.5);

    while (wb_robot_step(time_step) != -1) {
        double left_pos = wb_position_sensor_get_value(left_ps);
        double right_pos = wb_position_sensor_get_value(right_ps);

        double left_delta = fabs(left_pos - left_start);
        double right_delta = fabs(right_pos - right_start);

        if (left_delta >= target_rot && right_delta >= target_rot)
            break;
    }

    wb_motor_set_velocity(left_motor, 0);
    wb_motor_set_velocity(right_motor, 0);

    if (heading == 0) botY--;
    else if (heading == 1) botX++;
    else if (heading == 2) botY++;
    else if (heading == 3) botX--;
}

// ---------- Main ----------
int main(int argc, char **argv) {
    wb_robot_init();
    int time_step = (int)wb_robot_get_basic_time_step();

    // Motors + IMU
    imu = wb_robot_get_device("inertial unit");
    wb_inertial_unit_enable(imu, time_step);
    left_motor = wb_robot_get_device("left wheel motor");
    right_motor = wb_robot_get_device("right wheel motor");
    wb_motor_set_position(left_motor, INFINITY);
    wb_motor_set_position(right_motor, INFINITY);
    wb_motor_set_velocity(left_motor, 0);
    wb_motor_set_velocity(right_motor, 0);

    // Position sensors <<< ADDED
    left_ps = wb_robot_get_device("left wheel sensor");
    right_ps = wb_robot_get_device("right wheel sensor");
    wb_position_sensor_enable(left_ps, time_step);
    wb_position_sensor_enable(right_ps, time_step);

    // Ultrasonic sensors
    for (int i = 0; i < 3; i++) {
        usensors[i] = wb_robot_get_device(ultrasonic_names[i]);
        wb_distance_sensor_enable(usensors[i], time_step);
    }

    // Maze init
    for (int y = 0; y < MAZE_SIZE; y++)
        for (int x = 0; x < MAZE_SIZE; x++) {
            maze[y][x].walls = 0;
            maze[y][x].visited = 0;
            maze[y][x].dist = 255;
        }

    while (wb_robot_step(time_step) != -1) {
        double left_d = wb_distance_sensor_get_value(usensors[0]);
        double front_d = wb_distance_sensor_get_value(usensors[1]);
        double right_d = wb_distance_sensor_get_value(usensors[2]);
        

        update_walls(front_d, left_d, right_d);
        maze[botY][botX].visited = 1;

        flood_fill(goalX, goalY);

        int bestDir = heading;
        int minDist = maze[botY][botX].dist;
        for (int dir = 0; dir < 4; dir++) {
            int nx = botX, ny = botY;
            if (dir == 0 && !(maze[botY][botX].walls & 1)) ny--;
            else if (dir == 1 && !(maze[botY][botX].walls & 2)) nx++;
            else if (dir == 2 && !(maze[botY][botX].walls & 4)) ny++;
            else if (dir == 3 && !(maze[botY][botX].walls & 8)) nx--;

            if (nx >= 0 && nx < MAZE_SIZE && ny >= 0 && ny < MAZE_SIZE) {
                if (maze[ny][nx].dist < minDist) {
                    minDist = maze[ny][nx].dist;
                    bestDir = dir;
                }
            }
        }
        printf("| L: %.3f m | F: %.3f m | R: %.3f m | bD=%d |\n",
               left_d, front_d, right_d,bestDir);

        if (bestDir != heading) {
            int diff = (bestDir - heading + 4) % 4;
            if (diff == 1) turn_right(time_step);
            else if (diff == 3) turn_left(time_step);
            else { turn_right(time_step); turn_right(time_step); }
        }

        move_forward_cell(time_step);

        if (botX == goalX && botY == goalY) {
            printf("Goal reached!\n");
            break;
        }
    }

    wb_robot_cleanup();
    return 0;
}
