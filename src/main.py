# PYTHON VER. CUZ IK PYTHON NOT RLLY C++
M_PI = 3.141592653589793


if False:
    Competition = None
    Brain = None
    Motor = None
    MotorGroup = None
    Ports = None
    GearingRatio = None
    Rotation = None
    ThreeWirePort = None
    Pneumatics = None
    Inertial = None
    Controller = None
    PERCENT = None
    FORWARD = None
    REVERSE = None
    DEGREES = None
    MSEC = None
    PRIMARY = None
    import math
    def wait(duration, unit=None):
        pass

# Import wait if available, otherwise define a compatible wait function
try:
    wait
except NameError:
    import time
    def wait(duration, unit=None):
        # Only supports MSEC for simplicity
        if unit is not None and str(unit).lower().startswith("msec"):
            time.sleep(duration / 1000.0)
        else:
            time.sleep(duration)

class Ports:
    PORT1 = 1
    PORT2 = 2
    PORT9 = 9
    PORT10 = 10
    PORT11 = 11
    PORT12 = 12
    PORT13 = 13
    PORT15 = 15
    PORT18 = 18
    PORT19 = 19
    PORT20 = 20

try:
    GearingRatio
except NameError:
    class GearingRatio:
        RATIO_36_1 = "36:1"
        RATIO_18_1 = "18:1"
        RATIO_6_1 = "6:1"

try:
    Motor
except NameError:
    class Motor:
        def __init__(self, port, gearSetting=None, isReversed=False):
            self._pos = 0
        def position(self, units=None):
            return self._pos
        def spin(self, direction, speed, unit=None):
            pass
        def stop(self, mode=None):
            pass

    class MotorGroup:
        def __init__(self, *motors):
            pass
        def spin(self, direction, speed, unit=None):
            pass
        def stop(self, mode=None):
            pass

try:
    FORWARD
except NameError:
    FORWARD = 1
    REVERSE = -1

try:
    PERCENT
except NameError:
    PERCENT = 'percent'

try:
    DEGREES
except NameError:
    DEGREES = 'deg'

try:
    MSEC
except NameError:
    MSEC = 'msec'

try:
    PRIMARY
except NameError:
    PRIMARY = 1

# Motors - keep the same ports as your C++ project
FL = Motor(Ports.PORT13, gearSetting=GearingRatio.RATIO_36_1, isReversed=False)
FR = Motor(Ports.PORT18, gearSetting=GearingRatio.RATIO_36_1, isReversed=False)
BL = Motor(Ports.PORT11, gearSetting=GearingRatio.RATIO_36_1, isReversed=True)
BR = Motor(Ports.PORT20, gearSetting=GearingRatio.RATIO_36_1, isReversed=True)
ML = Motor(Ports.PORT12, gearSetting=GearingRatio.RATIO_36_1, isReversed=True)
MR = Motor(Ports.PORT19, gearSetting=GearingRatio.RATIO_36_1, isReversed=True)

LeftMotors = MotorGroup(FL, BL, ML)
RightMotors = MotorGroup(FR, BR, MR)

I = Motor(Ports.PORT1, gearSetting=GearingRatio.RATIO_18_1, isReversed=True)   # intake
I2 = Motor(Ports.PORT2, gearSetting=GearingRatio.RATIO_18_1, isReversed=True)  # intake
A = Motor(Ports.PORT9, gearSetting=GearingRatio.RATIO_36_1, isReversed=False)  # arm

# Rotation & pneumatics
try:
    Rotation
except NameError:
    class Rotation:
        def __init__(self, port):
            self.port = port
        def position(self, units=None):
            return 0
        def reset_position(self):
            pass
ArmRotation = Rotation(Ports.PORT10) 

try:
    Pneumatics
    ThreeWirePort
except NameError:
    class ThreeWirePort:
        A = 1
    class Pneumatics:
        def __init__(self, port):
            self.port = port
        def open(self):
            pass
        def close(self):
            pass

try:
    DoubleSolenoid = Pneumatics(ThreeWirePort.A)
except Exception:
    class PneumaticStub:
        def open(self): pass
        def close(self): pass
    DoubleSolenoid = PneumaticStub()

Inertial = Inertial(Ports.PORT15)

# Configuration constants (in inches)
WHEEL_DIAMETER_IN = 4.0
TRACK_WIDTH_IN = 12.0
WHEEL_CIRCUMFERENCE_IN = WHEEL_DIAMETER_IN * M_PI

# Controller
Controller = Controller(PRIMARY)

# Autonomous mode selection
class AutoMode:
    LEFT = 0
    LEFT_CENTER = 1
    RIGHT_CENTER = 2
    RIGHT = 3

autoMode = 0
autoModeNames = ["Left", "Left Center", "Right Center", "Right"]


def onScreenPressed():
    global autoMode
    autoMode = (autoMode + 1) % 4
    Brain.screen.clear_screen()
    Brain.screen.set_cursor(2, 2)
    Brain.screen.print("Auton: {}".format(autoModeNames[autoMode]))


def showAutoMode():
    Brain.screen.clear_screen()
    Brain.screen.set_cursor(2, 2)
    Brain.screen.print("Auton: {}".format(autoModeNames[autoMode]))

# Drive & helpers
def set_drive():
    axis2 = Controller.axis2.position()
    axis3 = Controller.axis3.position()
    left_speed = -axis3
    right_speed = axis2
    if abs(axis2) < 5:
        right_speed = 0
    if abs(axis3) < 5:
        left_speed = 0
    LeftMotors.spin(FORWARD, left_speed, PERCENT)
    RightMotors.spin(FORWARD, right_speed, PERCENT)


def set_intake_and_arm():
    if Controller.buttonA.pressing():
        I.spin(FORWARD, 100, PERCENT)
        I2.spin(FORWARD, 100, PERCENT)
    elif Controller.buttonB.pressing():
        I.spin(REVERSE, 100, PERCENT)
        I2.spin(REVERSE, 100, PERCENT)
    else:
        I.stop()
        I2.stop()

    if Controller.buttonX.pressing():
        A.spin(FORWARD, 100, PERCENT)
    elif Controller.buttonY.pressing():
        A.spin(REVERSE, 100, PERCENT)
    else:
        A.stop("hold")

    if Controller.buttonL1.pressing():
        DoubleSolenoid.open()
    elif Controller.buttonL2.pressing():
        DoubleSolenoid.close()

# drivePID - using motor rotations (degrees)
def drive_pid(target_distance_in, kP, kI, kD):
    error = prev_error = integral = derivative = 0.0
    left_start = FL.position(DEGREES)
    right_start = FR.position(DEGREES)
    target_degrees = (target_distance_in / WHEEL_CIRCUMFERENCE_IN) * 360.0
    max_time = 3000
    timer = 0
    while timer < max_time:
        left_pos = FL.position(DEGREES) - left_start
        right_pos = FR.position(DEGREES) - right_start
        avg_pos = (left_pos + right_pos) / 2.0
        error = target_degrees - avg_pos
        integral += error
        derivative = error - prev_error
        output = kP * error + kI * integral + kD * derivative
        if output > 100.0:
            output = 100.0
        if output < -100.0:
            output = -100.0
        LeftMotors.spin(FORWARD, output, PERCENT)
        RightMotors.spin(FORWARD, output, PERCENT)
        prev_error = error
        if abs(error) < 5:
            break
        wait(20, MSEC)
        timer += 20
    LeftMotors.stop("hold")
    RightMotors.stop("hold")

# turnPID - inertial based PD
def turn_pid(target_angle, kP, kD, timeout_ms=2000):
    start_heading = Inertial.heading()
    target_heading = (start_heading + target_angle) % 360.0
    error = prev_error = derivative = 0.0
    timer = 0
    dt = 20
    while timer < timeout_ms:
        current = Inertial.heading()
        diff = target_heading - current
        if diff > 180:
            diff -= 360
        if diff < -180:
            diff += 360
        error = diff
        derivative = error - prev_error
        output = kP * error + kD * derivative
        if output > 100:
            output = 100
        if output < -100:
            output = -100
        LeftMotors.spin(FORWARD, output, PERCENT)
        RightMotors.spin(REVERSE, output, PERCENT)
        prev_error = error
        if abs(error) < 2.0:
            break
        wait(dt, MSEC)
        timer += dt
    LeftMotors.stop("hold")
    RightMotors.stop("hold")

# Autonomous routines

def auton_left():
    drive_pid(24.0, 0.25, 0.0001, 0.2)
    wait(500, MSEC)
    turn_pid(90.0, 0.8, 0.08)
    drive_pid(-24.0, 0.25, 0.0001, 0.2)
    wait(500, MSEC)
    I.spin(FORWARD, 100, PERCENT)
    I2.spin(FORWARD, 100, PERCENT)
    wait(1000, MSEC)
    I.stop()
    I2.stop()
    A.spin(FORWARD, 100, PERCENT)
    wait(1000, MSEC)
    A.stop("hold")
    DoubleSolenoid.open()
    wait(500, MSEC)
    DoubleSolenoid.close()


def auton_right():
    drive_pid(36.0, 0.25, 0.0001, 0.2)
    wait(500, MSEC)
    turn_pid(-45.0, 0.8, 0.08)
    drive_pid(-12.0, 0.25, 0.0001, 0.2)
    wait(500, MSEC)
    I.spin(FORWARD, 100, PERCENT)
    I2.spin(FORWARD, 100, PERCENT)
    wait(800, MSEC)
    I.stop()
    I2.stop()
    A.spin(FORWARD, 100, PERCENT)
    wait(800, MSEC)
    A.stop("hold")
    DoubleSolenoid.open()
    wait(400, MSEC)
    DoubleSolenoid.close()


def auton_left_center():
    drive_pid(20.0, 0.25, 0.0001, 0.2)
    wait(450, MSEC)
    turn_pid(45.0, 0.8, 0.08)
    drive_pid(-20.0, 0.25, 0.0001, 0.2)
    wait(450, MSEC)
    I.spin(FORWARD, 100, PERCENT)
    I2.spin(FORWARD, 100, PERCENT)
    wait(900, MSEC)
    I.stop()
    I2.stop()
    A.spin(FORWARD, 100, PERCENT)
    wait(900, MSEC)
    A.stop("hold")
    DoubleSolenoid.open()
    wait(450, MSEC)
    DoubleSolenoid.close()


def auton_right_center():
    drive_pid(30.0, 0.25, 0.0001, 0.2)
    wait(450, MSEC)
    turn_pid(-30.0, 0.8, 0.08)
    drive_pid(-18.0, 0.25, 0.0001, 0.2)
    wait(450, MSEC)
    I.spin(FORWARD, 100, PERCENT)
    I2.spin(FORWARD, 100, PERCENT)
    wait(700, MSEC)
    I.stop()
    I2.stop()
    A.spin(FORWARD, 100, PERCENT)
    wait(700, MSEC)
    A.stop("hold")
    DoubleSolenoid.open()
    wait(350, MSEC)
    DoubleSolenoid.close()


def autonomous():
    if autoMode == AutoMode.LEFT:
        auton_left()
    elif autoMode == AutoMode.LEFT_CENTER:
        auton_left_center()
    elif autoMode == AutoMode.RIGHT_CENTER:
        auton_right_center()
    else:
        auton_right()


# Main
Brain.screen.pressed(onScreenPressed)
showAutoMode()

# Calibrate inertial at startup
Brain.screen.print_at(10, 40, "Calibrating Inertial...")
Inertial.calibrate()
while Inertial.is_calibrating():
    wait(50, MSEC)
Brain.screen.clear_line(2)
Brain.screen.print_at(10, 40, "Inertial Ready")

Competition.autonomous(autonomous)

while True:
    set_drive()
    set_intake_and_arm()
    wait(10, MSEC)
