#include "RobotController.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

RobotController::RobotController() {
  robot = new Robot();

  // Motors
  leftMotor = robot->getMotor("m2");
  rightMotor = robot->getMotor("m1");
  leftMotor->setPosition(INFINITY);
  rightMotor->setPosition(INFINITY);
  leftMotor->setVelocity(0.0);
  rightMotor->setVelocity(0.0);

  // Distance sensors
  dsLeft = robot->getDistanceSensor("ds_left");
  dsRight = robot->getDistanceSensor("ds_right");
  dsFront = robot->getDistanceSensor("ds_front");
  dsLeft->enable(TIME_STEP);
  dsRight->enable(TIME_STEP);
  dsFront->enable(TIME_STEP);

  // Encoders
  leftEncoder = robot->getPositionSensor("left_sensor");
  rightEncoder = robot->getPositionSensor("right_sensor");
  leftEncoder->enable(TIME_STEP);
  rightEncoder->enable(TIME_STEP);

  robot->step(TIME_STEP); // stabilize
}

double RobotController::getDistance(DistanceSensor *sensor) {
  double val = sensor->getValue();
  return (val == 0) ? 0.2 : val * 0.001;
}

void RobotController::PIDControl(double leftSensor, double rightSensor, bool followLeft) {
  double errorP = followLeft ? (leftSensor - rightSensor) - offset
                             : (leftSensor - rightSensor) + offset;

  double errorD = errorP - oldErrorP;
  errorI = (2.0 / 3.0) * errorI + errorP;
  totalError = P * errorP + D * errorD + I * errorI;

  oldErrorP = errorP;

  double RMS = BASE_SPEED + totalError;
  double LMS = BASE_SPEED - totalError;

  RMS = std::max(-6.0, std::min(6.0, RMS));
  LMS = std::max(-6.0, std::min(6.0, LMS));

  leftMotor->setVelocity(LMS);
  rightMotor->setVelocity(RMS);
}

void RobotController::moveForward(double distance) {
  double leftStart = leftEncoder->getValue();
  double rightStart = rightEncoder->getValue();

  leftMotor->setVelocity(2.0);
  rightMotor->setVelocity(2.0);

  while (robot->step(TIME_STEP) != -1) {
    double leftPos = leftEncoder->getValue() - leftStart;
    double rightPos = rightEncoder->getValue() - rightStart;
    double avgDist = WHEEL_RADIUS * (leftPos + rightPos) / 2.0;
    if (avgDist >= distance) break;
  }

  leftMotor->setVelocity(0.0);
  rightMotor->setVelocity(0.0);
}

void RobotController::turnLeft90() {
  double rightStart = rightEncoder->getValue();
  double turnArc = (M_PI / 2.0) * (AXLE_LENGTH / 2.0);
  double turnRot = turnArc / WHEEL_RADIUS;

  leftMotor->setVelocity(0.0);
  rightMotor->setVelocity(5.0);

  while (robot->step(TIME_STEP) != -1) {
    double rightPos = rightEncoder->getValue() - rightStart;
    if (fabs(rightPos) >= turnRot) break;
  }

  leftMotor->setVelocity(0.0);
  rightMotor->setVelocity(0.0);
}

void RobotController::turnRight90() {
  double leftStart = leftEncoder->getValue();
  double turnArc = (M_PI / 2.0) * (AXLE_LENGTH / 2.0);
  double turnRot = turnArc / WHEEL_RADIUS;

  leftMotor->setVelocity(5.0);
  rightMotor->setVelocity(0.0);

  while (robot->step(TIME_STEP) != -1) {
    double leftPos = leftEncoder->getValue() - leftStart;
    if (fabs(leftPos) >= turnRot) break;
  }

  leftMotor->setVelocity(0.0);
  rightMotor->setVelocity(0.0);
}

void RobotController::run() {
  bool followLeft = true;

  while (robot->step(TIME_STEP) != -1) {
    double leftSensor = getDistance(dsLeft);
    double rightSensor = getDistance(dsRight);
    double frontSensor = getDistance(dsFront);

    bool leftWall = (leftSensor < WALL_THRESHOLD);
    bool rightWall = (rightSensor < WALL_THRESHOLD);
    bool frontWall = (frontSensor < FRONT_THRESHOLD);

    std::cout << "L:" << leftSensor
              << " R:" << rightSensor
              << " F:" << frontSensor
              << " Err:" << totalError << std::endl;

    if (!leftWall && !frontWall) {
      moveForward(0.1);
      turnLeft90();
    } else if (frontWall && leftWall) {
      turnRight90();
    } else {
      PIDControl(leftSensor, rightSensor, followLeft);
    }
  }
}
