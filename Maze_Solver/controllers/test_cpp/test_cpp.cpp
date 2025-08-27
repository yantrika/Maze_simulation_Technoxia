#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/PositionSensor.hpp>
#include <iostream>
#include <cmath>
#include <string>

// ================== CONFIG ==================
constexpr int TIME_STEP = 64;
constexpr double BASE_SPEED = 2.0;
constexpr double WALL_THRESHOLD = 0.09;   // meters (9 cm)
constexpr double FRONT_THRESHOLD = 0.1;   // meters (10 cm)
constexpr double WHEEL_RADIUS = 0.03;     // 3 cm
constexpr double AXLE_LENGTH = 0.16;      // 16 cm

// ================== PID Controller ==================
class PIDController {
private:
  double P, I, D;
  double oldErrorP = 0.0;
  double errorI = 0.0;

public:
  PIDController(double p, double i, double d) : P(p), I(i), D(d) {}

  double compute(double error) {
    double errorD = error - oldErrorP;
    errorI = (2.0 / 3.0) * errorI + error;
    double output = P * error + D * errorD + I * errorI;
    oldErrorP = error;
    return output;
  }
};

// ================== Robot Driver ==================
class RobotDriver {
private:
  webots::Motor *leftMotor;
  webots::Motor *rightMotor;
  webots::PositionSensor *leftEncoder;
  webots::PositionSensor *rightEncoder;

public:
  RobotDriver(webots::Robot &robot) {
    leftMotor = robot.getMotor("m2");
    rightMotor = robot.getMotor("m1");
    leftMotor->setPosition(INFINITY);
    rightMotor->setPosition(INFINITY);
    leftMotor->setVelocity(0.0);
    rightMotor->setVelocity(0.0);

    leftEncoder = robot.getPositionSensor("left_sensor");
    rightEncoder = robot.getPositionSensor("right_sensor");
    leftEncoder->enable(TIME_STEP);
    rightEncoder->enable(TIME_STEP);
  }

  void setVelocity(double left, double right) {
    leftMotor->setVelocity(left);
    rightMotor->setVelocity(right);
  }

  void stop() { setVelocity(0.0, 0.0); }

  double getLeftPos() const { return leftEncoder->getValue(); }
  double getRightPos() const { return rightEncoder->getValue(); }

  void moveForward(webots::Robot &robot, double distance) {
    double leftStart = getLeftPos();
    double rightStart = getRightPos();
    double targetRotation = distance / WHEEL_RADIUS;

    setVelocity(BASE_SPEED, BASE_SPEED);
    while (robot.step(TIME_STEP) != -1) {
      double leftPos = getLeftPos() - leftStart;
      double rightPos = getRightPos() - rightStart;
      double avgDistance = WHEEL_RADIUS * (leftPos + rightPos) / 2.0;
      if (avgDistance >= distance) {
        stop();
        break;
      }
    }
  }

  void turn(webots::Robot &robot, bool leftTurn) {
    double leftStart = getLeftPos();
    double rightStart = getRightPos();

    double turnArc = (M_PI / 2.0) * (AXLE_LENGTH / 2.0);
    double turnRotation = turnArc / WHEEL_RADIUS;

    if (leftTurn) {
      setVelocity(5.0, 0.0);
      while (robot.step(TIME_STEP) != -1) {
        if (fabs(getLeftPos() - leftStart) >= turnRotation) {
          stop();
          break;
        }
      }
    } else {
      setVelocity(0.0, 5.0);
      while (robot.step(TIME_STEP) != -1) {
        if (fabs(getRightPos() - rightStart) >= turnRotation) {
          stop();
          break;
        }
      }
    }
  }
};

// ================== Wall-Follower Robot ==================
class WallFollowerRobot {
private:
  webots::Robot robot;
  RobotDriver driver;
  PIDController pid;

  webots::DistanceSensor *dsLeft, *dsRight, *dsFront;
  bool followLeft = true;
  int offset = 0;

public:
  WallFollowerRobot() : driver(robot), pid(0.7, 0.4, 0.5) {
    dsLeft = robot.getDistanceSensor("ds_left");
    dsRight = robot.getDistanceSensor("ds_right");
    dsFront = robot.getDistanceSensor("ds_front");
    dsLeft->enable(TIME_STEP);
    dsRight->enable(TIME_STEP);
    dsFront->enable(TIME_STEP);

    robot.step(TIME_STEP); // stabilize sensors
  }

  double getDistance(webots::DistanceSensor *sensor) {
    double val = sensor->getValue();
    return (val == 0) ? 0.2 : val * 0.001; // assume max ~0.2 m
  }

  void followWall(double leftDist, double rightDist) {
    double error = followLeft
                       ? (leftDist - rightDist) - offset
                       : (leftDist - rightDist) + offset;

    double correction = pid.compute(error);

    double RMS = BASE_SPEED + correction;
    double LMS = BASE_SPEED - correction;

    RMS = std::clamp(RMS, -6.0, 6.0);
    LMS = std::clamp(LMS, -6.0, 6.0);

    driver.setVelocity(LMS, RMS);

    std::cout << "L: " << leftDist
              << "  R: " << rightDist
              << "  Error: " << correction
              << std::endl;
  }

  void run() {
    while (robot.step(TIME_STEP) != -1) {
      double leftDist = getDistance(dsLeft);
      double rightDist = getDistance(dsRight);
      double frontDist = getDistance(dsFront);

      bool leftWall = (leftDist < WALL_THRESHOLD);
      bool rightWall = (rightDist < WALL_THRESHOLD);
      bool frontWall = (frontDist < FRONT_THRESHOLD);

      if (frontWall) {
        driver.stop();

        if (leftWall && !rightWall) {
          driver.turn(robot, true);
        } else if (rightWall && !leftWall) {
          driver.turn(robot, false);
        }
      }

      followWall(leftDist, rightDist);
    }
  }
};

// ================== MAIN ==================
int main() {
  WallFollowerRobot robot;
  robot.run();
  return 0;
}
