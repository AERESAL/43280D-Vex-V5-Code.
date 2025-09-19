# VEX V5 Competition Robot Code

## Overview

This repository contains C++ code for a VEX V5 competition robot, designed for both autonomous and driver-controlled periods in VEX Robotics events. The codebase is structured for reliability, modularity, and ease of iteration, supporting a robust engineering design process and competitive performance.

---

## Project Purpose & Competition Goals

This robot is engineered to excel in VEX Robotics Competition (VRC) matches, with the following objectives:
- **Consistent autonomous scoring**: Reliable routines for match-opening points.
- **Responsive driver control**: Smooth, intuitive teleop for all drive team members.
- **Modular code**: Easy to adapt for new strategies, hardware changes, and iterative improvements.

---

## Setup Instructions

### Requirements
- **VEXcode V5** or **VEXcode Pro V5** (latest version recommended)
- VEX V5 Brain, motors, sensors, controller, and competition field elements

### Getting Started
1. **Clone or Download** this repository to your computer.
2. **Open VEXcode V5/Pro V5** and select `Open Project`, then choose this folder.
3. **Check Device Ports**: Ensure all motor and sensor ports in `main.cpp` match your robot's wiring.
4. **Build** the project (click the Build button or use `Ctrl+B`).
5. **Download** the program to the V5 Brain (click Download or use `Ctrl+U`).
6. **Run** the program from the Brain's menu.

---

## Hardware Overview

- **Drivetrain**: 6-motor tank drive (3 left, 3 right)
- **Intake**: Dual-motor intake for game object collection
- **Arm**: High-torque arm motor with rotation sensor
- **Pneumatics**: Double-acting solenoid for endgame or scoring mechanisms
- **Sensors**:
  - Inertial sensor (gyro) for precise turning
  - Rotation sensor for arm position
- **Controller Layout**:
  - Joysticks: Tank drive
  - Button A/B: Intake in/out
  - Button X/Y: Arm up/down
  - L1/L2: Solenoid expand/compress

---

## Code Structure

- `main.cpp`: Main entry point, device definitions, and core logic
- **Autonomous routines**: Functions for pre-programmed match actions
- **Driver control logic**: Reads controller input, manages drive, intake, arm, and pneumatics
- **Helper functions**: PID control for driving and turning, sensor calibration, etc.
- **Modular design**: Easy to split into multiple files (e.g., `pid.cpp`, `auton.cpp`) for larger projects

---

## Uploading & Running the Code

1. Connect the V5 Brain to your computer via USB.
2. In VEXcode, click **Download** to upload the code.
3. On the Brain, select the program and press **Run**.
4. Use the competition switch or field control system to test autonomous and driver modes.

---

## Troubleshooting Tips

- **Motors not moving?**
  - Double-check port numbers in code vs. wiring.
  - Ensure motors are not reversed unless intended.
- **Sensors not responding?**
  - Calibrate the inertial sensor at startup.
  - Check sensor ports and cable connections.
- **Controller not working?**
  - Make sure the controller is paired to the Brain.
  - Verify button mappings in code.
- **Build errors?**
  - Ensure all device types are defined (see `vex.h`).
  - Use the latest VEXcode version.

---

## Engineering Notebook Integration

This codebase is designed to support the engineering design process:
- **Version control**: Track code changes and iterations for notebook documentation.
- **Commented logic**: Clear explanations for autonomous and driver strategies.
- **Modular structure**: Facilitates rapid prototyping and testing of new ideas.
- **Strategy alignment**: Code can be easily updated to reflect new match strategies and design changes.

Include code snippets, flowcharts, and explanations from this repository in your engineering notebook to document your programming process and design decisions.

---

## License

This project is released under the MIT License. See [LICENSE](LICENSE) for details.

## Contributing

Contributions are welcome! Please fork the repository and submit a pull request. For major changes, open an issue first to discuss your ideas.

- Follow VEX coding standards and comment your code.
- Test thoroughly on real hardware before submitting changes.

---

For questions or support, open an issue or contact the repository maintainer.

