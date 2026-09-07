# Bebop PID Controller Tutorial

This package provides one PID exercise script. The ROS and simulation template is already given, and students implement the PID calculation in:

```text
scripts/pid_template.py
```

The provided code already handles:

- Bebop odometry;
- goals selected in RViz;
- automatic takeoff;
- stopping PID control when a landing message is received;
- conversion from world coordinates to the Bebop body frame;
- simple velocity limits and goal tolerance; and
- publishing velocity commands to the simulator.

The starter `PidAxis.update()` method returns zero. Therefore, the unmodified
template will receive RViz goals but will not move the Bebop toward them.

## PID implementation task

Complete the PID implementation inside `PidAxis.update()`:

Yaw is not controlled in this exercise. The template only reads the current
yaw to transform the global X/Y PID outputs into the Bebop's local body frame.
A yaw controller can therefore be added later without changing the X/Y/Z PID
implementation.

Do not run keyboard teleoperation while the PID controller is active. Both
programs publish commands to the Bebop.

## Build and test

First build and source the workspace as described in the
[main README](../README.md). Then run:

```bash
roslaunch bebop_pid_controller pid_demo.launch
```

This command starts Gazebo, RViz, the Bebop, and the student controller. The
Bebop automatically takes off and then waits for a goal.

Wait until the Bebop has reached its takeoff height before selecting a goal.

In RViz:

1. Select **2D Nav Goal** from the toolbar.
2. Click the desired position on the ground grid.
3. Release the mouse button to send the goal. The arrow direction is ignored.

The RViz tool supplies X and Y. The controller always uses a height of 2 m.
You can click another goal at any time.

To land, open another sourced terminal and run:

```bash
rostopic pub --once /bebop/land std_msgs/Empty "{}"
```

The landing message cancels the active goal before landing.

## Start only the controller

If the empty-world simulation is already running, start only the controller:

```bash
roslaunch bebop_pid_controller pid_controller.launch
```
