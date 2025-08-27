#ifndef ROBOT_CONTROLLER_H
#define ROBOT_CONTROLLER_H

#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/PositionSensor.hpp>
#include <iostream>
#include <cmath>

using namespace webots;

class RobotController {
public:
  RobotController();
  void run();

private:
  // Core Webots instance
  Robot *robot;

  // Devices
  Motor *leftMotor;
  Motor *rightMotor;
  DistanceSensor *dsLeft;
  DistanceSensor *dsRight;
  DistanceSensor *dsFront;
  PositionSensor *leftEncoder;
  PositionSensor *rightEncoder;

  // Config constants
  const int TIME_STEP = 64;
  const double BASE_SPEED = 2.0;
  const double WALL_THRESHOLD = 0.100;
  const double FRONT_THRESHOLD = 0.110;
  const double WHEEL_RADIUS = 0.03;
  const double AXLE_LENGTH = 0.18;

  // PID params
  double P = 0.7, I = 0.4, D = 0.5;
  double oldErrorP = 0.0, errorI = 0.0, totalError = 0.0;
  double offset = 0.1;

  // State
  double prevLeftDistance = 0.0;
  const double Kp = 0.5;

  // Helpers
  double getDistance(DistanceSensor *sensor);
  void PIDControl(double leftSensor, double rightSensor, bool followLeft);

  // Movement primitives
  void moveForward(double distance);
  void turnLeft90();
  void turnRight90();
};

#endif
