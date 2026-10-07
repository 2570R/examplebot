
## 🚀 Features

### ✅ Universal PID Class
- Modular, reusable PID controller for all motion and heading tasks.

### 📍 Odometry
- Tracks robot position using drivetrain encoders and/or tracking wheels for accurate field navigation.

### 🧭 IMU Support
- Integrates inertial sensor (IMU) for heading correction and absolute orientation.

### 🔁 Tracking Wheel Flexibility
- Works with or without tracking wheels.
- Supports all tracking wheel configurations.
- Fall-back support using only drivetrain encoders and IMU for basic pose estimation.

### 🎯 Turn to Face Point/Heading
- Algorithms to turn the robot to face any field point or specific heading.

### 🔄 Swing to Face Heading
- Swing turn routines for efficient, single-side turning to a point or heading.

### 🏹 Move to Point
- Navigate to a specific coordinate using position and heading correction.
- Works consistently with or without tracking wheels.

### 📐 Curve Path Movement (`curveCircle`)
- Executes smooth, circular/curved paths to a target point.
- Fully compatible with robots **not using tracking wheels or full odometry**.
- Uses heading + distance PID control to maintain curvature.

### 🏹 Move to Pose via Boomerang
- Advanced “boomerang” algorithm for smooth, curved motion to a target pose (position + heading).

### ⛓️ Motion Chaining
- Seamlessly links multiple motion commands for fluid, uninterrupted autonomous routines.

### 🧭📍 Distance Resets
- Seamlessly integrated position resets using distance sensors.
- Has support for sensors on all sides, or on just the specific sides you want.


---

## 📘 Usage Guide

**Note: Anything modified outside of the custom folder will not be preserved in updates.**

### Getting Started

1. Open `custom/src/robot-config.cpp` and `custom/include/robot-config.h`. Set the
   ports, motor directions, sensors, and motor groups to match your robot.
2. Tune the chassis PID in
   `custom/src/robot-config.cpp`. Then calibrate the lift conversion, limits, Lift PID,
   claw, and roller settings in `custom/src/subsystems.cpp` before operating the
   mechanisms.
3. Add and tune autonomous routines in `custom/src/autonomous.cpp`. Select which
   routine runs in `runAutonomous()` in `custom/src/user.cpp`, then test it on the
   field.

### 1. Project Structure

Your project is organized into the following folders and files:

**custom/include/**
- `autonomous.h` — Declarations for autonomous routines  
- `robot-config.h` — Declarations for robot devices
- `subsystems.h` — Lift, claw, and roller subsystem controls
- `user.h` — Declarations for user functions

**custom/src/**
- `autonomous.cpp` — Your autonomous routines  
- `robot-config.cpp` — Your robot's device configurations
- `subsystems.cpp` — Lift, claw, and roller control loops
- `user.cpp` — User functions

**include/**
- `motor-control.h` — Core drive and motion control function declarations  
- `pid.h` — Reusable PID controller class declaration  
- `utils.h` — Math and geometry utility functions  
- `vex.h` — Standard VEX libraries and device setup

**src/**
- `main.cpp` — Competition entry point and control flow  
- `motor-control.cpp` — Core drive and motion control function implementations  
- `pid.cpp` — PID controller logic  
- `utils.cpp` — Math and geometry utility function implementations

### 2. Robot Configuration

Edit `custom/src/robot-config.cpp` and `custom/include/robot-config.h` to match your robot’s hardware.

- Set the correct ports for motors, sensors, and other robot devices
- Group drive motors into `left_chassis` and `right_chassis` motor groups

### 3. Tuning Your Robot

In `custom/src/robot-config.cpp`, locate the section labeled **USER-CONFIGURABLE PARAMETERS** and adjust the following:

- `distance_between_wheels`: Distance between the left and right wheels (in inches)  
- `wheel_distance_in`: Wheel circumference (see comments in code for help)  
- PID constants: `distance_kp`, `distance_ki`, `distance_kd`, etc.

If using a horizontal and/or vertical tracking wheel:

- Set `using_horizontal_tracker = true` and/or `using_vertical_tracker = true`
- Configure tracker distances and diameters accordingly

If using distance resets:

- Configure your ports to the prebuilt sensor declarations (DO NOT CHANGE THE NAMES)
- Set your sensor offsets (in inches) for each sensor (All offset values are positive)

### 4. Autonomous Programming

Edit `custom/src/autonomous.cpp` to define your autonomous routines.

You can use the motion functions from `motor-control.h`:

- `driveChassis(left_power, right_power)` — directly set left/right drive power.
- `driveTo(distance_in, time_limit_msec, exit, max_output)` — drive a distance in
  inches; positive drives forward and negative drives backward.
- `turnToAngle(turn_angle, time_limit_msec, exit, max_output)` — turn to an
  absolute heading in degrees.
- `swing(swing_angle, drive_direction, time_limit_msec, exit, max_output)` —
  turn around one stationary side of the chassis.
- `curveCircle(result_angle_deg, center_radius, time_limit_msec, exit, max_output)`
  — follow an arc using its radius and desired angle.
- `turnToPoint(x, y, dir, time_limit_msec)` — turn to face a field coordinate.
- `moveToPoint(x, y, dir, time_limit_msec, exit, max_output, overturn)` — drive
  to a field coordinate, correcting heading and distance along the way.
- `boomerang(x, y, dir, angle, dlead, time_limit_msec, exit, max_output, overturn)`
  — follow a curved path to a position and final heading.

Coordinates and distances are in inches, headings and angles are in degrees, and
time limits are in milliseconds. `exit` controls whether a motion stops the
chassis at the end, which can be useful when chaining motions. `max_output`
limits the motion controller's motor output. Tracking wheels can improve field
position estimates for point-to-point movement; the template can also use
drivetrain encoders and the inertial sensor.

Select the routine to run inside the `runAutonomous()` function in `custom/src/user.cpp`.

### Subsystem Functionality

The lift, claw, and roller control loop starts during pre-autonomous and continues
to run in both autonomous and driver control:

- **Claw intake and detection:** At the bottom, the claw motor group intakes until
  `clawDetect` sees a game piece within `claw_detect_distance_mm`; it then holds
  the piece. The claw rotator retracts for intake.
- **Driver lift and scoring:** Hold **L2** to raise the lift. Encoder feedback
  measures the lift height in inches and PID control tracks the target while the
  button is held. Releasing **L2** starts the descent. Once the lift is within
  5 inches of its captured scoring height, the rotator opens and the claw slowly
  outtakes. After reaching the bottom and completing the release, it returns to
  intake.
- **Manual lift target:** Call `setLiftTargetHeight(height_inches)` to request a
  lift height asynchronously. It clamps the target from zero to
  `lift_max_height_inches` and holds that target. A target of zero returns the
  lift to the bottom without starting the automatic scoring release.
- **Autonomous scoring:** Call `scoreLiftAutonomous()` to run a complete raise,
  lower, and release cycle. Each completed call increases the target by 10 inches
  (10 inches on the first call, 20 on the second, etc.), up to
  `lift_max_height_inches`. The function waits for that cycle to complete.
  `getAutonomousScoreCount()` returns the number of completed autonomous cycles.
- **Toggle roller:** The optical sensor continuously checks the roller color. The
  toggle motor spins while an object is present and its color does not match the
  target; it holds when the target color is detected or no object is seen. Set the
  target with `setRollerTargetColor(RollerColor::Red)`, `RollerColor::Yellow`, or
  `RollerColor::Blue`; the default is red.
- **Tied in Functionality:** There is only one button needed to be pressed between intaking and scoring.

Subsystem tuning variables, including lift conversion and PID gains, claw
detection distance and rotator angles, and toggle motor voltage, are declared in
`custom/include/subsystems.h` and initialized in `custom/src/subsystems.cpp`.
`lift_inches_per_output_revolution` defaults to 28 inches and
`lift_gear_reduction` is 60:12 (5:1). The lift encoder is zeroed at startup, so
start with the lift fully lowered and tune the conversion to the actual mechanism.

### 5. Driver Control

Edit the `runDriver()` function in `custom/src/user.cpp`.

By default, driver control uses arcade-style drive: **Axis3** controls forward and
reverse, and **Axis1** controls turning. The current mix is:

`driveChassis(ch3 * 0.12 + ch1 * 0.123, ch3 * 0.12 - ch1 * 0.123);`

You can customize this for:

- Tank drive
- Different arcade or split-arcade mixes
- Adding button controls for mechanisms

### 6. Competition Setup

This template follows the VEX Competition structure:

- `pre_auton()` which calls `runPreAutonomous()` runs once at startup (ideal for sensor calibration)  
- `autonomous()` which calls `runAutonomous()` runs during the autonomous period  
- `usercontrol()` which calls `runDriver()` runs during the driver control period  
- The `main()` function connects all of these automatically

### 7. Tips for Success

- Read comments in each file — they clarify how each function and variable works  
- Tune PID values for optimal performance  
- Test motion functions like `driveTo`, `turnToAngle`, and `moveToPoint` individually  
- Use odometry for advanced navigation (with or without tracking wheels)  
- Don’t hesitate to reach out to me or the VEX community for help
- Join our discord (link is in the description)

### 8. Where to Start

- Set up ports and devices in `custom/src/robot-config.cpp` and `custom/include/robot-config.h`  
- Input chassis measurements and PID values in `custom/src/robot-config.cpp`  
- Confirm movement and controls via driver control testing  
- Create and test simple autonomous routines  
- Expand with more complex logic and paths as you grow confident

---

Each new branch added would be for the other robots.
