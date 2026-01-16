/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       saise                                                     */
/*    Created:      6/18/2025                                                */
/*    Description:  V5 project                                                */
/*----------------------------------------------------------------------------*/

#include "vex.h"
using namespace vex;

// Forward declaration of autonomous function
void autonomous();

// Forward declaration of vexcode init
extern void vexcodeInit();

competition Competition;
brain Brain;

// Motors
motor FL = motor(PORT13, ratio6_1, true);
motor FR = motor(PORT18, ratio6_1, false);
motor BL = motor(PORT11, ratio6_1, true);
motor BR = motor(PORT20, ratio6_1, false);
motor ML = motor(PORT12, ratio6_1, true);
motor MR = motor(PORT19, ratio6_1, false);

motor_group LeftMotors = motor_group(FL, BL, ML);
motor_group RightMotors = motor_group(FR, BR, MR);

// Other mechanisms
motor I = motor(PORT1, ratio18_1, false);
motor convey = motor(PORT2, ratio18_1, true);
motor check = motor(PORT4, ratio36_1, false);

motor_group stor = motor_group(I, convey);

pneumatics descore = pneumatics(Brain.ThreeWirePort.C);
pneumatics tongue = pneumatics(Brain.ThreeWirePort.G);

// Drivetrain & Controller
drivetrain Drivetrain = drivetrain(LeftMotors, RightMotors, 12.0, 12.0, 0.0, mm, 1);
controller Controller = controller(primary);

// Inertial sensor for turning
inertial InertialSensor = inertial(PORT17);



// Driver control
void SetDrive() {
  int fwdInput = Controller.Axis3.position();
  int strafe = Controller.Axis1.position();

  if (abs(fwdInput) < 5) fwdInput = 0;
  if (abs(strafe) < 5) strafe = 0;

  int leftSpeed = fwdInput + strafe;
  int rightSpeed = fwdInput - strafe;

  if (leftSpeed == 0 && rightSpeed == 0) {
    LeftMotors.stop(brake);
    RightMotors.stop(brake);
  } else {
    if (leftSpeed > 0) LeftMotors.spin(directionType::fwd, leftSpeed, velocityUnits::pct);
    else if (leftSpeed < 0) LeftMotors.spin(directionType::rev, abs(leftSpeed), velocityUnits::pct);

    if (rightSpeed > 0) RightMotors.spin(directionType::fwd, rightSpeed, velocityUnits::pct);
    else if (rightSpeed < 0) RightMotors.spin(directionType::rev, abs(rightSpeed), velocityUnits::pct);
  }
}
// R1 = Convey + Intake F
// L1 = Convey + Intake B

// R2 = Check F
// L2 = Check B

void SetConvey() {
  if (Controller.ButtonR1.pressing()) {
    stor.spin(forward, 100, percentUnits::pct);
  } else if (Controller.ButtonR2.pressing()) {
    check.spin(forward, 100, percentUnits::pct);
  } else if (Controller.ButtonL1.pressing()) {
    stor.spin(reverse, 100, percentUnits::pct);
  } else if (Controller.ButtonL2.pressing()) {
    check.stop(brakeType::hold);
  } else {
    check.stop(brake);
    stor.stop(brake);
  }
}


void displayAllAngles() {
    Brain.Screen.clearScreen();
    Brain.Screen.setFont(mono20);
    Brain.Screen.setPenColor(white);

    while (true) {
        double yaw   = InertialSensor.yaw();
        double pitch = InertialSensor.pitch();
        double roll  = InertialSensor.roll();

        Brain.Screen.clearLine();

        Brain.Screen.setCursor(1,1);
        Brain.Screen.print("Yaw: %.2f", yaw);

        Brain.Screen.setCursor(2,1);
        Brain.Screen.print("Pitch: %.2f", pitch);

        Brain.Screen.setCursor(3,1);
        Brain.Screen.print("Roll:  %.2f", roll);

        task::sleep(50); // update ~20 times/sec
    }
}













struct PIDController {
    double Kp, Ki, Kd;
    double integral = 0;
    double lastError = 0;

    PIDController(double p, double i, double d) : Kp(p), Ki(i), Kd(d) {}

    double compute(double target, double current, double dt) {
        double error = target - current;
        integral += error * dt;
        double derivative = (error - lastError) / dt;
        lastError = error;
        return Kp * error + Ki * integral + Kd * derivative;
    }

    void reset() {
        integral = 0;
        lastError = 0;
    }
};

// ==================== HELPER FUNCTIONS ====================
double getDrivePosition() {
    return (LeftMotors.position(degrees) + RightMotors.position(degrees)) / 2.0;
}
// ==================== DRIVE FUNCTION ====================
void drivePID(double targetDegrees, double maxVoltage = 12.0) {
    PIDController pid(0.02, 0.0, 0.002); //ADJUST (Dont tocuh 1.)
    pid.reset();

    const int dt = 10;
    const int timeout = 1500; // ms
    int elapsed = 0;

    double error;

    LeftMotors.resetPosition();
    RightMotors.resetPosition();

    while (elapsed < timeout) {
        double current = getDrivePosition();
        error = targetDegrees - current;

        if (fabs(error) <= 1.0)
            break;

        double output = pid.compute(targetDegrees, current, dt);

        if (output > maxVoltage) output = maxVoltage;
        if (output < -maxVoltage) output = -maxVoltage;

        LeftMotors.spin(forward, output, voltageUnits::volt);
        RightMotors.spin(forward, output, voltageUnits::volt);

        task::sleep(dt);
        elapsed += dt;
    }

    LeftMotors.stop(brakeType::hold);
    RightMotors.stop(brakeType::hold);
}

void driveSimple(double inches, double maxVoltage = 12.0) {
  double degrees = inches * (5760 / (27.0 * 3.1415926535897932384626433832795028841971693993751058209749445)); 
  drivePID(degrees, maxVoltage);
}

// ==================== TURN FUNCTION ====================
void turnPID(double targetDeg, double maxVoltage = 12.0) {
    const double kP = 0.1565; // ADJUST
    const double kD = 0.1369; // ADJUST

    PIDController pid(kP, 0.0, kD);
    pid.reset();

    const int dt = 10;
    const int timeout = 3500;
    const double errorTol = 5.0; // STOP within ±5 degrees

    int elapsed = 0;

    InertialSensor.resetRotation();
    task::sleep(200);

    while (elapsed < timeout) {
        double current = InertialSensor.yaw();
        double error = targetDeg - current;

        // Wrap error to [-180, 180]
        while (error > 180)  error -= 360;
        while (error < -180) error += 360;

        // Exit when close enough
        if (fabs(error) <= errorTol) break;

        // Use ERROR directly (not target/current)
        double output = pid.compute(targetDeg, current, dt);

        // Clamp voltage
        if (output > maxVoltage) output = maxVoltage;
        if (output < -maxVoltage) output = -maxVoltage;

        LeftMotors.spin(forward, output, voltageUnits::volt);
        RightMotors.spin(reverse, output, voltageUnits::volt);

        task::sleep(dt);
        elapsed += dt;
    }

    LeftMotors.stop(brakeType::hold);
    RightMotors.stop(brakeType::hold);
}















// ==================== AUTONOMOUS ====================


//NUMBER 1 FUNCTION TO CHECK!!
void ClearLoader() {
    tongue.set(true);
    wait(0.5, seconds);
    stor.spin(reverse, 100, percentUnits::pct);
    driveSimple(15.0);
    wait(2.5, seconds);
    driveSimple(-30.0);
    check.spin(forward, 100, percentUnits::pct);
}

void autonomous() {
    driveSimple(30.0);
    turnPID(-90.0);
    ClearLoader();
    
}


void autonomousSkill() {
  // score center bottom
    driveSimple(26.0);
    turnPID(45.0);
    stor.spin(forward, 100, percentUnits::pct);
    driveSimple(12.0);
    stor.stop(brakeType::hold);
    turnPID(-55.0);
    driveSimple(12.0);
    stor.spin(reverse, 100, percentUnits::pct);
    driveSimple(12.0);


    driveSimple(-46.0);
    turnPID(135.0);

  // score right loader 
    ClearLoader();


    driveSimple(24.0);
    turnPID(90);
    driveSimple(72.0);
    turnPID(-90);

   // score left loader
    ClearLoader();

    driveSimple(24.0);
    turnPID(-90);
    driveSimple(12.0);
    turnPID(-90);
    driveSimple(72.0);
    turnPID(-90);
    driveSimple(12.0);
    turnPID(90);

    // score left back loader
    ClearLoader();

    driveSimple(24.0);
    turnPID(90);
    driveSimple(72.0);
    turnPID(-90);

   // score left loader
    ClearLoader();
}










void drivercontrol() {
  while (true) {
    SetDrive();
    SetConvey();
    
    if (Controller.ButtonA.pressing()) {
      tongue.set(true);
    } else if (Controller.ButtonB.pressing()) {
      tongue.set(false);
    }
        if (Controller.ButtonY.pressing()) {
      descore.set(true);
    } else if (Controller.ButtonX.pressing()) {
      descore.set(false);
    }
    
    task::sleep(10);
  }
}




int main() {
  vexcodeInit();
  
  // Calibrate inertial sensor
  InertialSensor.calibrate();
  while(InertialSensor.isCalibrating()) {
    task::sleep(100);
  }
  Competition.autonomous(autonomous);
  Competition.drivercontrol(drivercontrol);
  while (true) {
    task::sleep(10);
  }
}
