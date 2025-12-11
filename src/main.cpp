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

#define M_PI 3.141592653589793238462643383279502884197169399375105820974944592307816406286
using namespace vex;
using signature = vision::signature;
using code = vision::code;

// Forward declaration of autonomous function
void autonomous();

competition Competition;
brain Brain;

// Autonomous mode selection
int autoMode = 0;
const char* autoModeNames[] = {"Left", "Left Center", "Right Center", "Right"};

void onScreenPressed() {
  autoMode = (autoMode + 1) % 4;
  Brain.Screen.clearScreen();
  Brain.Screen.setCursor(2, 2);
  Brain.Screen.print("Auton: %s", autoModeNames[autoMode]);
}

void showAutoMode() {
  Brain.Screen.clearScreen();
  Brain.Screen.setCursor(2, 2);
  Brain.Screen.print("Auton: %s", autoModeNames[autoMode]);
}

motor FL = motor(PORT13, ratio36_1, true);
motor FR = motor(PORT18, ratio36_1, true);
motor BL = motor(PORT11, ratio36_1, true);
motor BR = motor(PORT20, ratio36_1, true);
motor ML = motor(PORT12, ratio36_1, true);
motor MR = motor(PORT19, ratio36_1, true);

motor_group LeftMotors = motor_group(FL, BL, ML);
motor_group RightMotors = motor_group(FR, BR, MR);

motor I = motor(PORT1, ratio18_1, true);
motor convey = motor(PORT2, ratio18_1, true);


rotation ArmRotation = rotation(PORT10, false);
pneumatics descore = pneumatics(Brain.ThreeWirePort.A);
pneumatics tongue = pneumatics(Brain.ThreeWirePort.B);


inertial Inertial = inertial(PORT15, turnType::right);
drivetrain Drivetrain = drivetrain(LeftMotors, RightMotors, 12.0, 12.0, 0.0, mm, 1);

controller Controller = controller(primary);


void SetDrive() {
  int axis1 = Controller.Axis3.position();  // Right joystick X (turn)
  int axis3 = Controller.Axis1.position();  // Left joystick Y (forward/backward)

  // Arcade drive: left stick Y for forward/back, right stick X for turn
  int forward = -axis3;
  int turn = -axis1;
  
  // Apply deadzone
  if (abs(forward) < 5) forward = 0;
  if (abs(turn) < 5) turn = 0;
  
  // Calculate left and right motor speeds
  int leftSpeed = forward + turn;
  int rightSpeed = forward - turn;
  
  LeftMotors.spin(directionType::fwd, leftSpeed, percentUnits::pct);
  RightMotors.spin(directionType::fwd, rightSpeed, percentUnits::pct);
}


void SetConvey() {
    if (Controller.ButtonR1.pressing()) {
        I.spin(forward, 100, percentUnits::pct);
        convey.spin(forward, 100, percentUnits::pct);
    } else if (Controller.ButtonR2.pressing()) {
        I.spin(reverse, 100, percentUnits::pct);
        convey.spin(reverse, 100, percentUnits::pct);
    } else {
        I.stop(brakeType::hold);
        convey.stop(brakeType::hold);
    }


}

int main() {
  Brain.Screen.pressed(onScreenPressed);
  showAutoMode();
  Competition.autonomous(autonomous);
  while (true) {
    SetDrive();
    SetConvey();
    wait(10, msec);
  }
}



// Copyright © 2025 43280D. All rights reserved.
// This code is part of a competitive VEX V5 robotics project.
// saitrseelam.com
