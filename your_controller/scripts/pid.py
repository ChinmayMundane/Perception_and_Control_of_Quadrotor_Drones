#!/usr/bin/env python3

import sys
import select
import termios
import tty

import numpy as np
import rospy
from geometry_msgs.msg import Point, PoseStamped, Twist
from std_msgs.msg import Empty
from visualization_msgs.msg import Marker
from tf import transformations

# -----------------------------
# Parameters (keep all user-tunable values here)
# -----------------------------
# Student task 1: choose your own goal in the horizontal plane.
# The drone is already flying at a safe altitude after takeoff, so we only need to
# control x and y motion here. The z-axis is not part of this PID task.
# TODO: Change the 2D goal below to your own test location (in meters in the odom/world frame).
# To inspect the current Bebop pose, run:
#   rostopic echo /vrpn_client_node/bebop/pose
# This prints the current position and orientation in ROS.
GOAL_POSITION = np.array([3.0, -2.0], dtype=float)

# DO NOT CHANGE below parameters unless you know what you are doing.
MAX_SPEED = 0.15
GOAL_THRESHOLD = 0.25
TAKEOFF_ALTITUDE = 0.75
START_KEY = 's'
LAND_KEY = 'l'
EXIT_KEY = '\x1b'  # ESC
TAKEOFF_WAIT_TIMEOUT = 15.0
POST_TAKEOFF_DELAY = 3.0
PUBLISH_RATE_HZ = 5.0

# PID gains for the x/y planar motion.
KP = np.array([0.8, 0.8], dtype=float)
KI = np.array([0.02, 0.02], dtype=float)
KD = np.array([0.15, 0.15], dtype=float)

# -----------------------------
# Global state
# -----------------------------
# These variables keep the controller state across callbacks.
# The PID update depends on the previous error and the elapsed time between pose updates.
last_error = np.zeros(2, dtype=float)
integral_error = np.zeros(2, dtype=float)
last_time = None
goal_reached = False
pid_active = False

velocity_pub = None
velocity_viz_pub = None
goal_viz_pub = None
drone_pose_viz_pub = None
current_pose = None

takeoff_pub = None
land_pub = None


def transform_to_bebop_frame(pose_msg, target_world_point):
    """TODO: Convert a target point from the world frame into the Bebop body frame.

    Hints for students:
      1. Read the drone position and yaw/orientation from pose_msg.
      2. Compute the relative vector from the drone to the target in the world frame.
      3. Use the drone rotation matrix to convert that vector into the drone body frame.
      4. Keep only the x and y components because the drone is already flying at altitude.

    The math is:
      relative_world = target_world - drone_world
      relative_body  = R_drone.T * relative_world
    """
    # Student task: implement the frame transform here.
    # Replace the placeholder with the formula above.
    # Hint: the final output should be a 2D vector [vx, vy] in the drone frame.
    return np.zeros(2, dtype=float)


def pid_controller(error, dt, kp, ki, kd):
    """TODO: Implement the 2D PID update law.

    Hints for students:
      1. Use the current error e.
      2. Accumulate the integral: integral += e * dt.
      3. Compute the derivative: (e - previous_error) / dt.
      4. Apply the classic PID law: u = kp * e + ki * integral + kd * derivative.
      5. Store the previous error into "last_error" before returning the control output.

    The controller should output a 2D velocity command in the drone frame.
    """
    global last_error, integral_error

    # Student task: write the discrete PID controller here.
    # Keep the error history in the global variables so the next control step can
    # compare against the previous error value.
    return np.zeros_like(error, dtype=float)


def transform_body_velocity_to_world(pose_msg, velocity_body):
    """TODO: Rotate a body-frame velocity into the world/odom frame for RViz.

    Hints for students:
      1. Build the drone rotation matrix from the quaternion in pose_msg.
      2. Treat the commanded planar velocity as [vx, vy, 0] in the body frame.
      3. Multiply by the rotation matrix to get the world-frame velocity.
      4. Keep only the x and y components for the RViz arrow visualization.
    """
    # Student task: convert the body-frame velocity to world coordinates here.
    # This only needs to return a 2D vector for plotting in RViz.
    return np.zeros(2, dtype=float)


def publish_empty_msg(pub):
    """Publish a standard Empty message to a ROS topic.

    ROS action topics such as takeoff and land use Empty messages, which means
    "no payload is needed"—the topic name itself tells the drone what to do.
    """
    if pub is not None:
        pub.publish(Empty())


def read_single_key(timeout=0.1):
    """Read one keyboard key without blocking the rest of the ROS loop.

    This is useful in a teaching/demo setup because the node can keep running,
    waiting for user input without freezing the ROS callbacks or RViz updates.
    """
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setcbreak(fd)
        rlist, _, _ = select.select([sys.stdin], [], [], timeout)
        if rlist:
            ch = sys.stdin.read(1)
            return ch
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return None


def wait_until_altitude(target_altitude, timeout_seconds):
    """Wait until the drone reaches the target altitude or exits on timeout.

    This is used after takeoff: we want to make sure the Bebop has actually climbed
    to a safe height before starting the horizontal PID controller.
    """
    start = rospy.Time.now().to_sec()
    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        if current_pose is not None:
            current_z = current_pose.pose.position.z
            if current_z >= target_altitude:
                return True
        if (rospy.Time.now().to_sec() - start) > timeout_seconds:
            rospy.logwarn("Timed out waiting for drone altitude %.2f m", target_altitude)
            return False
        rate.sleep()
    return False


def visualize_goal_marker(goal_pos, z=1.0):
    """Publish a green sphere in RViz to show the target point.

    This makes the goal location visible in the map frame so students can easily see
    where the controller is trying to drive the drone.
    """
    goal_marker = Marker()
    goal_marker.header.frame_id = "odom"
    goal_marker.header.stamp = rospy.Time.now()
    goal_marker.ns = "goal_position"
    goal_marker.id = 0
    goal_marker.type = Marker.SPHERE
    goal_marker.action = Marker.ADD
    goal_marker.pose.position.x = goal_pos[0]
    goal_marker.pose.position.y = goal_pos[1]
    goal_marker.pose.position.z = z
    goal_marker.pose.orientation.w = 1.0
    goal_marker.scale.x = 0.3
    goal_marker.scale.y = 0.3
    goal_marker.scale.z = 0.3
    goal_marker.color.r = 0.0
    goal_marker.color.g = 1.0
    goal_marker.color.b = 0.0
    goal_marker.color.a = 1.0
    goal_marker.lifetime = rospy.Duration(0.5)
    goal_viz_pub.publish(goal_marker)


def visualize_drone_pose(drone_pos):
    """Publish a blue sphere to visualize the current drone position in RViz.

    This is useful for debugging the robot state while the controller is running.
    Students can compare the drone position with the goal marker and the control arrow.
    """
    pose_marker = Marker()
    pose_marker.header.frame_id = "odom"
    pose_marker.header.stamp = rospy.Time.now()
    pose_marker.ns = "drone_pose"
    pose_marker.id = 0
    pose_marker.type = Marker.SPHERE
    pose_marker.action = Marker.ADD
    pose_marker.pose.position.x = drone_pos[0]
    pose_marker.pose.position.y = drone_pos[1]
    pose_marker.pose.position.z = drone_pos[2]
    pose_marker.pose.orientation.w = 1.0
    pose_marker.scale.x = 0.25
    pose_marker.scale.y = 0.25
    pose_marker.scale.z = 0.25
    pose_marker.color.r = 0.0
    pose_marker.color.g = 0.4
    pose_marker.color.b = 1.0
    pose_marker.color.a = 1.0
    # Keep the current pose marker alive long enough to remain visible between
    # successive pose updates, otherwise it can blink out at the publish rate.
    pose_marker.lifetime = rospy.Duration(1.0)
    drone_pose_viz_pub.publish(pose_marker)


def visualize_velocity_marker(drone_pos, velocity_cmd):
    """Draw the computed velocity as a red arrow in RViz.

    We draw an arrow from the current robot position to a point shifted by the
    commanded velocity. This gives an intuitive visual of the control action.
    """
    marker = Marker()
    marker.header.frame_id = "odom"
    marker.header.stamp = rospy.Time.now()
    marker.ns = "control_velocity"
    marker.id = 0
    marker.type = Marker.ARROW
    marker.action = Marker.ADD

    start = Point()
    start.x = drone_pos[0]
    start.y = drone_pos[1]
    start.z = drone_pos[2]

    end = Point()
    end.x = drone_pos[0] + velocity_cmd[0] * 5.0
    end.y = drone_pos[1] + velocity_cmd[1] * 5.0
    end.z = drone_pos[2]

    marker.points = [start, end]
    marker.scale.x = 0.05
    marker.scale.y = 0.10
    marker.scale.z = 0.15
    marker.color.r = 1.0
    marker.color.g = 0.0
    marker.color.b = 0.0
    marker.color.a = 1.0
    # Give the control arrow a longer lifetime so it stays visible while the
    # drone is being driven and does not disappear between update cycles.
    marker.lifetime = rospy.Duration(1.0)
    velocity_viz_pub.publish(marker)


def drone_pose_callback(msg):
    """Receive pose updates and compute the next corrective velocity command.

    This callback is the heart of the controller: it reads the current pose, computes
    the position error relative to the goal, and sends a velocity command back to the drone.
    The drone already has a safe altitude, so we only need to stabilize the horizontal plane.
    """
    global current_pose, last_error, integral_error, last_time, goal_reached, pid_active

    current_pose = msg

    if not pid_active:
        return

    # Current drone position in the world/odom frame.
    drone_pos = np.array([
        msg.pose.position.x,
        msg.pose.position.y,
        msg.pose.position.z,
    ], dtype=float)

    # Convert the goal into the drone body frame so the controller can react to the
    # target relative to the drone's current orientation.
    error_body = transform_to_bebop_frame(msg.pose, GOAL_POSITION)

    # Estimate the elapsed time between pose messages. The PID controller uses this to
    # update the integral and derivative terms correctly.
    now = rospy.Time.now().to_sec()
    if last_time is None:
        dt = 0.02
    else:
        dt = max(now - last_time, 1e-3)
    last_time = now

    # Compute the planar velocity command in the drone body frame.
    control_body = pid_controller(error_body, dt, KP, KI, KD)

    # Saturate the command so the drone does not exceed the safe maximum speed.
    norm = np.linalg.norm(control_body)
    if norm > MAX_SPEED:
        control_body = control_body / norm * MAX_SPEED

    twist_msg = Twist()
    twist_msg.linear.x = float(control_body[0])
    twist_msg.linear.y = float(control_body[1])
    twist_msg.linear.z = 0.0
    twist_msg.angular.x = 0.0
    twist_msg.angular.y = 0.0
    twist_msg.angular.z = 0.0
    velocity_pub.publish(twist_msg)

    # Plot the commanded velocity in the world frame for debugging in RViz.
    velocity_world = transform_body_velocity_to_world(msg.pose, control_body)
    visualize_drone_pose(drone_pos)
    visualize_velocity_marker(drone_pos, velocity_world)

    # Stop the controller once the drone reaches the target in the horizontal plane.
    distance = np.linalg.norm(drone_pos[:2] - GOAL_POSITION[:2])
    if distance < GOAL_THRESHOLD:
        goal_reached = True
        pid_active = False
        rospy.loginfo("Goal reached. PID controller stopped.")
        twist_msg.linear.x = 0.0
        twist_msg.linear.y = 0.0
        twist_msg.linear.z = 0.0
        velocity_pub.publish(twist_msg)
        rospy.loginfo("Press '%s' to land the drone, or '%s' to exit without landing.", LAND_KEY, EXIT_KEY)


def start_sequence():
    """Take off, wait for altitude, hold briefly, then enable PID control.

    This step-by-step sequence makes the mission more reliable:
      1. publish takeoff
      2. wait until altitude is high enough
      3. pause briefly to stabilize
      4. start the PID controller
    """
    global pid_active

    rospy.loginfo("Sending takeoff command...")
    publish_empty_msg(takeoff_pub)

    if not wait_until_altitude(TAKEOFF_ALTITUDE, TAKEOFF_WAIT_TIMEOUT):
        rospy.logwarn("Takeoff did not reach target height in time. Exiting.")
        land_and_exit()
        return False

    rospy.loginfo("Altitude %.2f m reached. Holding for %.1f s", TAKEOFF_ALTITUDE, POST_TAKEOFF_DELAY)
    rospy.sleep(POST_TAKEOFF_DELAY)

    pid_active = True
    rospy.loginfo("PID control enabled.")
    return True


def land_and_exit():
    """Stop any motion and send a landing command.

    This is used both for the user-triggered landing and for safety shutdowns. The
    drone is commanded to stop moving, and then the land action is sent.
    """
    global pid_active

    pid_active = False
    zero_twist = Twist()
    zero_twist.linear.x = 0.0
    zero_twist.linear.y = 0.0
    zero_twist.linear.z = 0.0
    velocity_pub.publish(zero_twist)

    rospy.loginfo("Sending landing command...")
    publish_empty_msg(land_pub)


def main():
    """Main ROS node loop.

    The node waits for keyboard input, starts the mission when 's' is pressed, and
    keeps publishing the goal marker to RViz while monitoring for user exit or landing.
    """
    global velocity_pub, velocity_viz_pub, goal_viz_pub, drone_pose_viz_pub, takeoff_pub, land_pub

    rospy.init_node('pid_node', anonymous=False)

    # ROS publishers used by the controller and visualization tools.
    # The drone command is sent to /bebop/velocity, while RViz markers show the goal,
    # current pose, and the commanded velocity arrow.
    velocity_pub = rospy.Publisher('/bebop/velocity', Twist, queue_size=2)
    velocity_viz_pub = rospy.Publisher('/control_velocity_viz', Marker, queue_size=2)
    goal_viz_pub = rospy.Publisher('/goal_viz', Marker, queue_size=2)
    drone_pose_viz_pub = rospy.Publisher('/drone_pose_viz', Marker, queue_size=2)
    takeoff_pub = rospy.Publisher('/bebop/takeoff', Empty, queue_size=2)
    land_pub = rospy.Publisher('/bebop/land', Empty, queue_size=2)

    # Subscribe to the drone pose estimate from the VRPN system.
    rospy.Subscriber('/vrpn_client_node/bebop/pose', PoseStamped, drone_pose_callback, queue_size=1)

    print("Keyboard control")
    print("  Press '%s' to start takeoff -> PID control" % START_KEY)
    print("  When goal is reached, press '%s' to land" % LAND_KEY)
    print("  Press ESC to exit without landing")

    rate = rospy.Rate(PUBLISH_RATE_HZ)

    while not rospy.is_shutdown():
        visualize_goal_marker(GOAL_POSITION, z=1.0)

        key = read_single_key(timeout=0.1)
        if key is None:
            rate.sleep()
            continue

        if key == START_KEY:
            if not pid_active and not goal_reached:
                if not start_sequence():
                    rospy.loginfo("Exiting node after takeoff timeout.")
                    break
        elif key == LAND_KEY:
            if goal_reached:
                land_and_exit()
                rospy.loginfo("User landed the drone.")
                break
            else:
                rospy.logwarn("Goal not reached yet; cannot land now.")
        elif key == EXIT_KEY:
            land_and_exit()
            rospy.loginfo("Exiting node without landing.")
            break

        if goal_reached:
            rospy.loginfo("Goal reached. Waiting for user command: press '%s' to land or '%s' to exit.", LAND_KEY, EXIT_KEY)

        rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
