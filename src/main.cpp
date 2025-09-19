/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       saise                                                     */
/*    Created:      6/18/2025, 6:13:28 PM                                     */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "vex.h"
#include <cmath>

#ifndef M_PI
#define M_PI 3.141592653589793238462643383279502884197169399375105820974944592307816406286
#endif
using namespace vex;
using signature = vision::signature;
using code = vision::code;

competition Competition;
brain Brain;

motor FL = motor(PORT13, ratio36_1, false);
motor FR = motor(PORT18, ratio36_1, false);
motor BL = motor(PORT11, ratio36_1, true);
motor BR = motor(PORT20, ratio36_1, true);
motor ML = motor(PORT12, ratio36_1, true);
motor MR = motor(PORT19, ratio36_1, true);

motor_group LeftMotors = motor_group(FL, BL, ML);
motor_group RightMotors = motor_group(FR, BR, MR);

motor I = motor(PORT1, ratio18_1, true);
motor I2 = motor(PORT2, ratio18_1, true);

motor A = motor(PORT9, ratio36_1, false);

rotation ArmRotation = rotation(PORT10, false);
pneumatics DoubleSolenoid = pneumatics(Brain.ThreeWirePort.A);

inertial Inertial = inertial(PORT15, turnType::right);
drivetrain Drivetrain = drivetrain(LeftMotors, RightMotors, 12.0, 12.0, 0.0, mm, 1);

controller Controller = controller(primary);










void SetDrive() {
  int axis2 = Controller.Axis2.position();
  int axis3 = Controller.Axis3.position();
  int leftSpeed = -axis3;
  int rightSpeed = axis2;
  if (abs(axis2) < 5) rightSpeed = 0;
  if (abs(axis3) < 5) leftSpeed = 0;
  LeftMotors.spin(forward, leftSpeed, percentUnits::pct);
  RightMotors.spin(forward, rightSpeed, percentUnits::pct);
}

void SetIntakeAndArm() {
  if (Controller.ButtonA.pressing()) {
    I.spin(forward, 100, percentUnits::pct);
    I2.spin(forward, 100, percentUnits::pct);
  } else if (Controller.ButtonB.pressing()) {
    I.spin(reverse, 100, percentUnits::pct);
    I2.spin(reverse, 100, percentUnits::pct);
  } else {
    I.stop();
    I2.stop();
  }
  if (Controller.ButtonX.pressing()) {
    A.spin(forward, 100, percentUnits::pct);
  } else if (Controller.ButtonY.pressing()) {
    A.spin(reverse, 100, percentUnits::pct);
  } else {
    A.stop(hold);
  }

  if (Controller.ButtonL1.pressing()) {
    DoubleSolenoid.open();
  } else if (Controller.ButtonL2.pressing()) {
    DoubleSolenoid.close();
  }
}








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

void autonomous() {
  drivePID(24.0, 0.25, 0.0001, 0.2);
  wait(500, msec);
  drivePID(-24.0, 0.25, 0.0001, 0.2);
  wait(500, msec);
  I.spin(forward, 100, percent);
  I2.spin(forward, 100, percent);
  wait(1000, msec);
  I.stop();
  I2.stop();
  A.spin(forward, 100, percent);
  wait(1000, msec);
  A.stop(hold);
  DoubleSolenoid.open();
  wait(500, msec);
  DoubleSolenoid.close();
}










int main() {
  Competition.autonomous(autonomous);
  while (true) {
    SetDrive();
    SetIntakeAndArm();
    wait(10, msec);
  }
}



// Copyright © 2025 43280D. All rights reserved.
// This code is part of a competitive VEX V5 robotics project.
// Shared for inspiration only — not for reuse in other teams' competition code.
