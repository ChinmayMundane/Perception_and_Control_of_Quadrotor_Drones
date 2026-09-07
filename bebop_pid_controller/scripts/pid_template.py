#!/usr/bin/env python3

"""PID exercise for the simulated Bebop."""

import math

import rospy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Empty
from tf.transformations import euler_from_quaternion


RATE = 20.0
GOAL_ALTITUDE = 2.0
MAX_XY_SPEED = 0.8
MAX_Z_SPEED = 0.5
GOAL_TOLERANCE = 0.2


class PidAxis:
    """Implement the PID controller, complete the update method part."""

    def __init__(self, kp, ki, kd, integral_limit=1.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral_limit = integral_limit
        self.integral = 0.0
        self.previous_error = None

    def reset(self):
        self.integral = 0.0
        self.previous_error = None

    def update(self, error, dt):
        # TODO 1: Calculate the proportional term.

        # TODO 2: Add error * dt to self.integral and clamp it to
        #         [-self.integral_limit, self.integral_limit].

        # TODO 3: Calculate the derivative from the current and previous
        #         errors. Use zero on the first update.

        # TODO 4: Save the current error in self.previous_error.

        # TODO 5: Return the sum of the P, I, and D terms.
        return 0.0


# The ROS code below is supplied. You do not need to change it.
class TutorialPidController:
    def __init__(self):
        rospy.init_node("pid_template")

        self.pid_x = PidAxis(0.8, 0.02, 0.20)
        self.pid_y = PidAxis(0.8, 0.02, 0.20)
        self.pid_z = PidAxis(0.8, 0.03, 0.20)
        self.position = None
        self.yaw = 0.0
        self.goal = None

        # See what the publishers and subscribers are and which topics they use.
        self.command_pub = rospy.Publisher("/bebop/cmd_vel", Twist, queue_size=1)
        self.takeoff_pub = rospy.Publisher("/bebop/takeoff", Empty, queue_size=1)
        rospy.Subscriber("/bebop/odom", Odometry, self.odom_callback)
        rospy.Subscriber(
            "/move_base_simple/goal", PoseStamped, self.goal_callback
        )
        rospy.Subscriber("/bebop/land", Empty, self.land_callback)

        self.takeoff_timer = rospy.Timer(rospy.Duration(1.0), self.takeoff)
        rospy.Timer(rospy.Duration(1.0 / RATE), self.control)
        rospy.on_shutdown(self.stop)
        rospy.loginfo("PID tutorial ready; select a 2D Nav Goal in RViz")

    def reset_pid(self):
        self.pid_x.reset()
        self.pid_y.reset()
        self.pid_z.reset()

    def takeoff(self, _event):
        if self.position is not None and self.position[2] > 0.5:
            self.takeoff_timer.shutdown()
        else:
            self.takeoff_pub.publish(Empty())

    def odom_callback(self, message):
        pose = message.pose.pose
        self.position = (pose.position.x, pose.position.y, pose.position.z)
        orientation = pose.orientation
        quaternion = (
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w,
        )
        self.yaw = euler_from_quaternion(quaternion)[2]

    def goal_callback(self, message):
        self.goal = (
            message.pose.position.x,
            message.pose.position.y,
            GOAL_ALTITUDE,
        )
        self.reset_pid()
        rospy.loginfo(
            "New goal: x=%.2f, y=%.2f, z=%.2f",
            self.goal[0],
            self.goal[1],
            self.goal[2],
        )

    def land_callback(self, _message):
        self.goal = None
        self.reset_pid()
        self.stop()

    def control(self, _event):
        if self.position is None or self.goal is None:
            return

        error_x = self.goal[0] - self.position[0]
        error_y = self.goal[1] - self.position[1]
        error_z = self.goal[2] - self.position[2]

        distance = math.sqrt(error_x ** 2 + error_y ** 2 + error_z ** 2)
        if distance < GOAL_TOLERANCE:
            self.stop()
            return

        dt = 1.0 / RATE
        velocity_x_global = self.pid_x.update(error_x, dt)
        velocity_y_global = self.pid_y.update(error_y, dt)
        velocity_z = self.pid_z.update(error_z, dt)

        xy_speed = math.hypot(velocity_x_global, velocity_y_global)
        if xy_speed > MAX_XY_SPEED:
            scale = MAX_XY_SPEED / xy_speed
            velocity_x_global *= scale
            velocity_y_global *= scale

        # Transform global X/Y velocities into the drone's local frame.
        cos_yaw = math.cos(self.yaw)
        sin_yaw = math.sin(self.yaw)
        velocity_x_local = (
            cos_yaw * velocity_x_global + sin_yaw * velocity_y_global
        )
        velocity_y_local = (
            -sin_yaw * velocity_x_global + cos_yaw * velocity_y_global
        )

        command = Twist()
        command.linear.x = velocity_x_local
        command.linear.y = velocity_y_local
        command.linear.z = max(-MAX_Z_SPEED, min(velocity_z, MAX_Z_SPEED))
        self.command_pub.publish(command)

    def stop(self):
        self.command_pub.publish(Twist())


if __name__ == "__main__":
    TutorialPidController()
    rospy.spin()
