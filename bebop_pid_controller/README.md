# Bebop PID Controller Reference

This package shows how a PID position controller is implemented for the
simulated Bebop. You can use this code as a reference for the assignment.

The reference controller is in:

```text
scripts/pid_template.py
```

The provided code handles:

- Bebop odometry;
- goals selected in RViz;
- automatic takeoff;
- stopping PID control when a landing message is received;
- conversion from world coordinates to the Bebop body frame;
- simple velocity limits and goal tolerance;
- publishing velocity commands to the simulator; and
- the proportional, integral, and derivative calculation.

## What the reference code demonstrates

The `PidAxis.update()` method shows how to:

- calculate the proportional term from the current error;
- accumulate the integral term over time;
- limit the accumulated integral to reduce integral windup;
- calculate the derivative from the current and previous errors; and
- combine the three terms into one controller output.

The same `PidAxis` class is used independently for the global X, global Y,
and altitude errors. You can use this implementation as a reference for the
assignment.

## Run the reference controller

First build and source the workspace as described in the
[main README](../README.md). Then run:

```bash
roslaunch bebop_pid_controller pid_demo.launch
```

This command starts Gazebo, RViz, the Bebop, and the reference controller. The
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
