#!/usr/bin/env python3

import numpy as np
import cvxpy as cp
import rospy
import sensor_msgs.point_cloud2 as pcl2
from geometry_msgs.msg import PoseStamped, Twist, Point
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Empty
from tf import transformations
from visualization_msgs.msg import Marker

# ------------------------------------------------------------
# CBF helper functions
# ------------------------------------------------------------
# This file follows the same teaching style as pid.py:
# - keep the controller logic in a function
# - keep ROS-message conversion in separate helper functions
# - make the main barrier logic easy to replace with student TODOs

# Student task 1: set your own goal for the horizontal navigation task.
# The drone is already climbing to a safe altitude after takeoff, so this CBF example
# only needs to control x and y motion. The z-axis is not part of the assignment.
# TODO: Replace the default goal with your own 2D target in the odom/world frame.
# Example: GOAL_POSITION = np.array([1.5, -1.0], dtype=float)
# Hint: choose a point that is reachable, within the range of mocap and easy to observe in RViz.
GOAL_POSITION = np.array([3.0, -2.0], dtype=float)

# DO NOT change the following parameters unless you are tuning your own controller.
MAX_SPEED = 0.15
GOAL_THRESHOLD = 0.25
TAKEOFF_ALTITUDE = 0.75
START_KEY = 's'
LAND_KEY = 'l'
EXIT_KEY = '\x1b'
TAKEOFF_WAIT_TIMEOUT = 15.0
POST_TAKEOFF_DELAY = 3.0
PUBLISH_RATE_HZ = 5.0

# Student task 2: tune the barrier parameters for your own obstacle setup.
# These values affect how strongly the drone is attracted to the goal and how far it
# stays away from nearby obstacles.
# TODO: Choose a safe desired separation distance, barrier strength, and attraction gain.
# Hints:
#   - D_OBS: desired minimum clearance from nearest obstacles (meters)
#   - ALPHA: control strength of the barrier constraint
#   - K_ATT: how strongly the controller moves toward the goal
#   - K_NEAR_OBSTACLES: how many closest obstacle points to keep for the QP to reduce solve time
D_OBS = 0.25
ALPHA = 0.1
K_ATT = 1.0
K_NEAR_OBSTACLES = 50

# DO NOT change the following global variables unless you are debugging the code.
latest_obstacle_cloud = None
current_pose = None
last_time = None
goal_reached = False
cbf_active = False

velocity_pub = None
velocity_viz_pub = None
drone_pose_viz_pub = None
goal_viz_pub = None
takeoff_pub = None
land_pub = None


def visualize_goal_marker(goal_pos, z=1.0):
    """Publish a green sphere in RViz to show the target goal."""
    goal_marker = Marker()
    goal_marker.header.frame_id = "odom"
    goal_marker.header.stamp = rospy.Time.now()
    goal_marker.ns = "goal_position"
    goal_marker.id = 0
    goal_marker.type = Marker.SPHERE
    goal_marker.action = Marker.ADD
    goal_marker.pose.position.x = float(goal_pos[0])
    goal_marker.pose.position.y = float(goal_pos[1])
    goal_marker.pose.position.z = float(z)
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
    """Publish a blue sphere to visualize the current drone position in RViz."""
    drone_pos = np.asarray(drone_pos, dtype=float)
    pose_marker = Marker()
    pose_marker.header.frame_id = "odom"
    pose_marker.header.stamp = rospy.Time.now()
    pose_marker.ns = "drone_pose"
    pose_marker.id = 0
    pose_marker.type = Marker.SPHERE
    pose_marker.action = Marker.ADD
    pose_marker.pose.position.x = float(drone_pos[0])
    pose_marker.pose.position.y = float(drone_pos[1])
    pose_marker.pose.position.z = float(drone_pos[2]) if drone_pos.size > 2 else 1.0
    pose_marker.pose.orientation.w = 1.0
    pose_marker.scale.x = 0.25
    pose_marker.scale.y = 0.25
    pose_marker.scale.z = 0.25
    pose_marker.color.r = 0.0
    pose_marker.color.g = 0.4
    pose_marker.color.b = 1.0
    pose_marker.color.a = 1.0
    pose_marker.lifetime = rospy.Duration(1.0)
    drone_pose_viz_pub.publish(pose_marker)


def visualize_velocity_marker(drone_pos, velocity_cmd):
    """Draw the computed velocity as a red arrow in RViz."""
    drone_pos = np.asarray(drone_pos, dtype=float)
    velocity_cmd = np.asarray(velocity_cmd, dtype=float)

    marker = Marker()
    marker.header.frame_id = "odom"
    marker.header.stamp = rospy.Time.now()
    marker.ns = "control_velocity"
    marker.id = 0
    marker.type = Marker.ARROW
    marker.action = Marker.ADD

    start = Point()
    start.x = float(drone_pos[0])
    start.y = float(drone_pos[1])
    start.z = float(drone_pos[2]) if drone_pos.size > 2 else 1.0

    end = Point()
    end.x = float(drone_pos[0]) + float(velocity_cmd[0]) * 5.0
    end.y = float(drone_pos[1]) + float(velocity_cmd[1]) * 5.0
    end.z = float(drone_pos[2]) if drone_pos.size > 2 else 1.0

    marker.points = [start, end]
    marker.scale.x = 0.05
    marker.scale.y = 0.10
    marker.scale.z = 0.15
    marker.color.r = 1.0
    marker.color.g = 0.0
    marker.color.b = 0.0
    marker.color.a = 1.0
    marker.lifetime = rospy.Duration(1.0)
    velocity_viz_pub.publish(marker)


def transform_to_bebop_frame(pose_msg, target_world_point):
    """TODO: Convert a planar world target into the Bebop body frame.

    Hints for students:
      1. Get the drone position and quaternion from pose_msg.
      2. Compute the target relative to the drone in the world frame.
      3. Rotate that vector with the drone's rotation matrix.
      4. Keep only x and y, since the drone already has altitude control.

    The usual relationship is:
      relative_world = target_world - drone_world
      relative_body  = R_drone.T * [relative_world_x, relative_world_y, 0]
    """
    # Student task: implement the frame conversion here.
    # The output should be a 2D vector in the drone's local frame.
    return np.zeros(2, dtype=float)


def transform_velocity_to_bebop_frame(pose_msg, velocity_world):
    """TODO: Rotate a world-frame planar velocity into the Bebop body frame.

    Hints for students:
      1. Build the drone rotation matrix from the quaternion in pose_msg.
      2. Treat the world-frame velocity as [vx, vy, 0] in the odom frame.
      3. Multiply by R_drone.T to move into the drone body frame.
      4. Keep only x and y so the final command matches the commanded planar motion.

    This is the same idea used in the PID task when commands are published to the drone.
    """
    # Student task: implement the transform from world velocity to body velocity.
    return np.zeros(2, dtype=float)


def filter_nearest_obstacles(current_pos, obstacles, k=K_NEAR_OBSTACLES):
    """Keep only the K closest obstacle samples to the drone to reduce CBF solve time."""
    if obstacles is None or obstacles.size == 0:
        return np.empty((0, 2), dtype=float)

    obstacles = np.asarray(obstacles, dtype=float)
    if obstacles.ndim == 1:
        obstacles = obstacles.reshape(1, -1)
    if obstacles.shape[0] <= k or k <= 0:
        return obstacles[:, :2]

    rel = obstacles[:, :2] - np.asarray(current_pos[:2], dtype=float)
    dists = np.linalg.norm(rel, axis=1)
    keep_idx = np.argsort(dists)[:k]
    return obstacles[keep_idx, :2]


def cbf_controller(current_pos, goal_pos, obstacles, k_att=K_ATT, d_obs=D_OBS, alpha=ALPHA, max_speed=MAX_SPEED):
    """TODO: Implement the CBF controller logic yourself.

    Student task:
      1. Prepare the obstacle list from the input data.
      2. Build the nominal goal-seeking velocity.
      3. Add safety constraints using the CBF idea.
      4. Solve the optimization problem and return the final safe command.

    Keep the implementation in the 2D plane only.
    """
    current_pos = np.asarray(current_pos, dtype=float)
    goal_pos = np.asarray(goal_pos, dtype=float)

    if current_pos.shape[0] < 2:
        raise ValueError('current_pos must contain at least [x, y].')
    if goal_pos.shape[0] < 2:
        raise ValueError('goal_pos must contain at least [x, y].')

    # Student task - prepare the obstacle data.
    # Example: obstacles = np.asarray(obstacles, dtype=float)
    # Handle empty data and keep only the nearby obstacles you want to consider
    # To avoid excessive computation, you can filter the obstacles to only 
    # the closest K points with filter_nearest_obstacles() function.
    obstacles = ...

    # Student task - define the goal-attraction term.
    v_des = np.zeros(2, dtype=float)

    # Student task - form the CBF optimization.
    # Create the decision variable, objective, and constraints for the barrier problem.
    v = ...  # Decision variable for the velocity command
    cost = ... # Objective function to minimize the difference from v_des
    constraints = [] # List of constraints to enforce safety with respect to obstacles
    
    # Add the CBF constraints for each nearby obstacle.
    for obs in obstacles:
        # Compute the relative position and distance to the obstacle
        # Formulate the CBF constraint and append it to the constraints list
        ...
    
    # Solve the optimization and recover the final safe velocity.
    if constraints:
        problem = cp.Problem(cp.Minimize(cost), constraints)
        try:
            problem.solve(solver=cp.ECOS, verbose=False)
        except Exception:
            problem.solve(solver=cp.SCS, verbose=False)
        if v.value is not None:
            u = np.asarray(v.value, dtype=float).reshape(2)
        else:
            u = v_des.copy()
    else:
        u = v_des.copy()

    # Enforce the speed limit.
    norm = np.linalg.norm(u)
    if norm > max_speed and norm > 1e-12:
        u = (u / norm) * max_speed

    return u


def pointcloud_to_obstacles(cloud_msg):
    """Convert a PointCloud2 obstacle cloud into a NumPy array of obstacle points."""
    if cloud_msg is None:
        return np.empty((0, 3), dtype=float)

    points = np.asarray(
        list(pcl2.read_points(cloud_msg, field_names=('x', 'y', 'z'), skip_nans=True))
    )
    if points.size == 0:
        return np.empty((0, 3), dtype=float)
    return points.astype(float)


def obstacle_cloud_callback(msg):
    """Store the latest obstacle point cloud received from setup_circular_obstacles.py."""
    global latest_obstacle_cloud
    latest_obstacle_cloud = msg


def cbf_controller_from_cloud(current_pos, goal_pos, cloud_msg, k_att=K_ATT, d_obs=D_OBS, alpha=ALPHA, max_speed=MAX_SPEED):
    """Wrapper used when the controller receives the obstacle cloud from ROS."""
    obstacle_points = pointcloud_to_obstacles(cloud_msg)
    return cbf_controller(current_pos, goal_pos, obstacle_points, k_att=k_att, d_obs=d_obs, alpha=alpha, max_speed=max_speed)


def publish_empty_msg(pub):
    if pub is not None:
        pub.publish(Empty())


def drone_pose_callback(msg):
    """Main CBF control loop using the obstacle point cloud subscription."""
    global current_pose, last_time, goal_reached, cbf_active

    current_pose = msg
    if not cbf_active:
        return

    drone_pos = np.array([
        msg.pose.position.x,
        msg.pose.position.y,
    ], dtype=float)

    now = rospy.Time.now().to_sec()
    if last_time is None:
        dt = 0.02
    else:
        dt = max(now - last_time, 1e-3)
    last_time = now

    goal_world = GOAL_POSITION
    if latest_obstacle_cloud is not None:
        obstacles = pointcloud_to_obstacles(latest_obstacle_cloud)
    else:
        obstacles = np.empty((0, 3), dtype=float)

    # The CBF law is computed in the global/odom frame, which is the natural frame for
    # the obstacle cloud and the goal position. The final command is then rotated into the
    # Bebop body frame before publishing, because the drone accepts local-frame velocity.
    control_world = np.asarray(cbf_controller(drone_pos, goal_world, obstacles), dtype=float).reshape(2)
    control_body = np.asarray(transform_velocity_to_bebop_frame(msg.pose, control_world), dtype=float).reshape(2)

    # Force planar motion: the drone should only track x-y velocity and never command z motion.
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

    # Match the PID visualizer: draw the drone pose marker and control arrow in odom.
    visualize_drone_pose(drone_pos)
    visualize_velocity_marker(drone_pos, control_world)

    distance = np.linalg.norm(drone_pos - GOAL_POSITION[:2])
    if distance < GOAL_THRESHOLD:
        goal_reached = True
        cbf_active = False
        twist_msg.linear.x = 0.0
        twist_msg.linear.y = 0.0
        twist_msg.linear.z = 0.0
        velocity_pub.publish(twist_msg)
        rospy.loginfo('Goal reached. CBF controller stopped.')


def start_sequence():
    """Takeoff and enable the CBF controller once the drone is high enough."""
    global cbf_active
    rospy.loginfo('Sending takeoff command...')
    publish_empty_msg(takeoff_pub)
    rospy.sleep(1.0)
    cbf_active = True
    rospy.loginfo('CBF control enabled.')
    return True


def land_and_exit():
    global cbf_active
    cbf_active = False
    zero_twist = Twist()
    zero_twist.linear.x = 0.0
    zero_twist.linear.y = 0.0
    zero_twist.linear.z = 0.0
    velocity_pub.publish(zero_twist)
    rospy.loginfo('Sending landing command...')
    publish_empty_msg(land_pub)


def main():
    global velocity_pub, velocity_viz_pub, drone_pose_viz_pub, goal_viz_pub, takeoff_pub, land_pub

    rospy.init_node('cbf_node', anonymous=False)

    velocity_pub = rospy.Publisher('/bebop/velocity', Twist, queue_size=2)
    velocity_viz_pub = rospy.Publisher('/control_velocity_viz', Marker, queue_size=2)
    drone_pose_viz_pub = rospy.Publisher('/drone_pose_viz', Marker, queue_size=2)
    goal_viz_pub = rospy.Publisher('/goal_viz', Marker, queue_size=2)
    takeoff_pub = rospy.Publisher('/bebop/takeoff', Empty, queue_size=2)
    land_pub = rospy.Publisher('/bebop/land', Empty, queue_size=2)

    rospy.Subscriber('/obs_pcl_topic', PointCloud2, obstacle_cloud_callback, queue_size=1)
    rospy.Subscriber('/vrpn_client_node/bebop/pose', PoseStamped, drone_pose_callback, queue_size=1)

    rospy.loginfo('CBF node started. Subscribed to /obs_pcl_topic and /vrpn_client_node/bebop/pose.')

    # Keep the same startup pattern as the PID file for a simple demo setup.
    rospy.loginfo("Press '%s' to start takeoff -> CBF control", START_KEY)
    rospy.loginfo("Press '%s' to land, or ESC to exit.", LAND_KEY)

    rate = rospy.Rate(PUBLISH_RATE_HZ)
    while not rospy.is_shutdown():
        visualize_goal_marker(GOAL_POSITION, z=1.0)

        key = None
        if hasattr(__import__('sys'), 'stdin'):
            try:
                import select, sys, termios, tty
                fd = sys.stdin.fileno()
                old_settings = termios.tcgetattr(fd)
                try:
                    tty.setcbreak(fd)
                    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
                    if rlist:
                        key = sys.stdin.read(1)
                finally:
                    termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
            except Exception:
                pass

        if key == START_KEY:
            if not cbf_active and not goal_reached:
                start_sequence()
        elif key == LAND_KEY:
            if goal_reached:
                land_and_exit()
                break
        elif key == EXIT_KEY:
            land_and_exit()
            break

        rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
