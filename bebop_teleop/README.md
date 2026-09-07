# Bebop keyboard teleoperation

Start a Bebop simulation, then run:

```bash
rosrun bebop_teleop bebop_teleop.py
```

Keys:

- `1`: take off
- `2`: land
- `i`, `,`: forward/backward
- `j`, `l`: yaw left/right
- `Shift+J`, `Shift+L`: strafe left/right
- `3`, `4`: ascend/descend
- `q`, `z`: increase/decrease all speeds
- `w`, `x`: increase/decrease linear speed
- `e`, `c`: increase/decrease yaw speed
- any other key: stop
- `Ctrl+C`: stop and exit

Do not run this at the same time as another Bebop controller. Only one node
should command the quadrotor at a time.
