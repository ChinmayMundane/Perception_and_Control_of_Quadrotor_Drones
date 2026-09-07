#!/usr/bin/env python3

from __future__ import print_function

import select
import sys
import termios
import tty

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty


HELP = """
Keyboard control for the simulated Bebop
-----------------------------------------
Moving:
   u    i    o
   j    k    l
   m    ,    .

Shift + movement key: strafe
1/2: take off/land
3/4: ascend/descend
q/z: increase/decrease all speeds
w/x: increase/decrease linear speed
e/c: increase/decrease yaw speed
Any other key: stop
Ctrl-C: exit
"""

MOVE_BINDINGS = {
    "i": (1, 0, 0, 0),
    "o": (1, 0, 0, -1),
    "j": (0, 0, 0, 1),
    "l": (0, 0, 0, -1),
    "u": (1, 0, 0, 1),
    ",": (-1, 0, 0, 0),
    ".": (-1, 0, 0, 1),
    "m": (-1, 0, 0, -1),
    "O": (1, -1, 0, 0),
    "I": (1, 0, 0, 0),
    "J": (0, 1, 0, 0),
    "L": (0, -1, 0, 0),
    "U": (1, 1, 0, 0),
    "<": (-1, 0, 0, 0),
    ">": (-1, -1, 0, 0),
    "M": (-1, 1, 0, 0),
    "3": (0, 0, 1, 0),
    "4": (0, 0, -1, 0),
}

SPEED_BINDINGS = {
    "q": (1.1, 1.1),
    "z": (0.9, 0.9),
    "w": (1.1, 1.0),
    "x": (0.9, 1.0),
    "e": (1.0, 1.1),
    "c": (1.0, 0.9),
}


def get_key(settings):
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def make_twist(x=0.0, y=0.0, z=0.0, yaw=0.0):
    command = Twist()
    command.linear.x = x
    command.linear.y = y
    command.linear.z = z
    command.angular.z = yaw
    return command


def main():
    settings = termios.tcgetattr(sys.stdin)
    rospy.init_node("bebop_keyboard_teleop")

    velocity_pub = rospy.Publisher("/bebop/cmd_vel", Twist, queue_size=1)
    takeoff_pub = rospy.Publisher("/bebop/takeoff", Empty, queue_size=1)
    land_pub = rospy.Publisher("/bebop/land", Empty, queue_size=1)

    speed = rospy.get_param("~speed", 0.5)
    turn = rospy.get_param("~turn", 1.0)

    print(HELP)
    print("speed: {:.2f}, turn: {:.2f}".format(speed, turn))

    try:
        while not rospy.is_shutdown():
            key = get_key(settings)

            if key in MOVE_BINDINGS:
                x, y, z, yaw = MOVE_BINDINGS[key]
                velocity_pub.publish(
                    make_twist(x * speed, y * speed, z * speed, yaw * turn)
                )
            elif key in SPEED_BINDINGS:
                speed_scale, turn_scale = SPEED_BINDINGS[key]
                speed *= speed_scale
                turn *= turn_scale
                print("speed: {:.2f}, turn: {:.2f}".format(speed, turn))
            elif key == "1":
                takeoff_pub.publish(Empty())
                print("Taking off")
            elif key == "2":
                land_pub.publish(Empty())
                print("Landing")
            elif key == "\x03":
                break
            else:
                velocity_pub.publish(make_twist())
    finally:
        velocity_pub.publish(make_twist())
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)


if __name__ == "__main__":
    main()
