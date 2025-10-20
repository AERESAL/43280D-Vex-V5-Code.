#include "vex.h"
#include <cmath>

#ifndef M_PI
#define M_PI 3.141592653589793238462643383279502884197169399375105820974944592307816406286
#endif

using namespace vex;

// External declarations (defined in main.cpp)
extern brain Brain;
extern motor FL, FR, BL, BR, ML, MR;
extern motor_group LeftMotors, RightMotors;
extern motor I, I2, A;
extern rotation ArmRotation;
extern pneumatics DoubleSolenoid;
extern inertial Inertial;
extern drivetrain Drivetrain;
extern controller Controller;

// Autonomous mode selection
extern int autoMode;
extern const char* autoModeNames[];

// PID drive function
void drivePID(double targetDistance, double kP, double kI, double kD) {
  double error = 0, prevError = 0, integral = 0, derivative = 0;
  double leftStart = FL.position(degrees);
  double rightStart = FR.position(degrees);
  double avgPosition = 0;
  double targetDegrees = (targetDistance / (4.0 * M_PI)) * 360.0;
  int maxTime = 3000;
  int timer = 0;
  while (timer < maxTime) {
    double leftPos = FL.position(degrees) - leftStart;
    double rightPos = FR.position(degrees) - rightStart;
    avgPosition = (leftPos + rightPos) / 2.0;
    error = targetDegrees - avgPosition;
    integral += error;
    derivative = error - prevError;
    double output = kP * error + kI * integral + kD * derivative;
    if (output > 100.0) output = 100.0;
    if (output < -100.0) output = -100.0;
    LeftMotors.spin(forward, output, percent);
    RightMotors.spin(forward, output, percent);
    prevError = error;
    if (fabs(error) < 5) break;
    wait(20, msec);
    timer += 20;
  }
  LeftMotors.stop(hold);
  RightMotors.stop(hold);
}

// TUNE THESE VALUES:
// - Distance (inches): How far the robot should move
// - kP: Proportional gain (increase for faster response, decrease if oscillating)
// - kI: Integral gain (helps eliminate steady-state error)
// - kD: Derivative gain (reduces overshoot and oscillation)
// Example: drivePID(distance, kP, kI, kD)

// Autonomous routines for different starting positions

void autonLeft() {
  I.spin(forward, 100, percentUnits::pct);
  I2.spin(forward, 100, percentUnits::pct);
  drivePID(18, 0, 0, 0);
  LeftMotors.spin(forward, 50, percentUnits::pct);
  drivePID(10, 0, 0, 0);
  I.spin(forward, 100, percentUnits::pct);
  I2.spin(forward, 100, percentUnits::pct);
}

void autonRight() {
  I.spin(forward, 100, percentUnits::pct);
  I2.spin(forward, 100, percentUnits::pct);
  drivePID(18, 0, 0, 0);
  RightMotors.spin(forward, 50, percentUnits::pct);
  A.spin(forward, 100, percentUnits::pct);
  A.stop(hold);
  drivePID(10, 0, 0, 0);
  I.spin(forward, 100, percentUnits::pct);
  I2.spin(forward, 100, percentUnits::pct);
}

void autonLeftCenter() {
  // Add your left center autonomous code here
}

void autonRightCenter() {
  // Add your right center autonomous code here
}

/**
 * @brief Main autonomous routine
 * 
 * This function is called during the autonomous period.
 * Selects and runs the appropriate autonomous routine based on autoMode.
 */
void autonomous() {
  if (autoMode == 0) {  // LEFT
    autonLeft();
  } else if (autoMode == 1) {  // LEFT_CENTER
    autonLeftCenter();
  } else if (autoMode == 2) {  // RIGHT_CENTER
    autonRightCenter();
  } else if (autoMode == 3) {  // RIGHT
    autonRight();
  }
}
