#!/usr/bin/python3

import rospy
import numpy as np

from tf import transformations
from message_filters import ApproximateTimeSynchronizer, Subscriber

from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped, Point, Twist, Vector3
from visualization_msgs.msg import Marker

import open3d as o3d
import open3d_conversions

import cvxopt
from cvxopt import solvers
import random

from scipy.spatial.transform import Rotation as R
from gazebo_model_collision_plugin.msg import Contact

class ControlBarrierFunction:
	def __init__(self):
		rospy.init_node("ControlBarrierFunction")
		print("[QUADROTOR CBF] Initializing the 'ControlBarrierFunction' Node...")
		#----------------------------------------------------------------------------------

		# Observation topics
		self.cloud_base_topic = rospy.get_param('~cloud_topic','/lidar_pcd_base_link')
		self.odom_topic = rospy.get_param('~odom_topic','/bebop/odom')

		# Publishers
		self._vel_pub = rospy.Publisher("/cmd_vel", Twist,queue_size=10)

		self._path_pub = rospy.Publisher("/bebop_path", Path, queue_size=10)
		self.path_msg = Path()
		self.path_msg.header.frame_id = "odom"
		self.path_points = []

		# RViZ Visualization publishers
		self._spheremarker_pub = rospy.Publisher('/goaltf_marker', Marker, queue_size=10)
		self._arrowmarker_pub = rospy.Publisher('/goal_marker', Marker, queue_size=10)
		#----------------------------------------------------------------------------------

		# Parameters 

		# PointCloud Downsampling
		self.lidar_frame_id = rospy.get_param('~lidar_frame_id', 'base_link')
		self.L_max_lidar = 1000

		# observation data
		self.odom = None
		self.goal = None
		self.transformed_goal = None
		self.goal_arr = None
		self.rotation_mtx = None
		self.heading_aligned = False

		# Odometry
		self.pose = None
		self.linear_vel = None
		self.linear_acc = None
		self.prev_linear_vel = np.array([0.0, 0.0, 0.0])
		self.prev_time = None
		self.prev_yaw_e = 0.0

		self.theta_avg_num = 3
		self.theta_arr = np.zeros(self.theta_avg_num)

		# Barrier constraints
		self.lidar_distance = None
		self.lidar_unit_vec = None

		#----------------------------------------------------------------------------------

		# ############# Goal position ###################
		self.goal_sub = rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.handle_goal)
		self.heading_aligned = False

		#----------------------------------------------------------------------------------			
		# ## Hard-Coding the Goal position
		# self.goal = PoseStamped()
		# self.goal.pose.position.x = 3.5
		# self.goal.pose.position.y = -3.0
		# self.goal.pose.position.z = 3.0
		# self.goal.pose.orientation.x = 0.0
		# self.goal.pose.orientation.y = 0.0
		# self.goal.pose.orientation.z = 0.
		# self.goal.pose.orientation.w = 1.
		# self.init_state = False

		# print("GOAL SET AS :=")
		# print(" x: ", self.goal.pose.position.x)
		# print(" y: ", self.goal.pose.position.y)
		# print(" z: ", self.goal.pose.position.z)
		# self.goal_arr = np.array([self.goal.pose.position.x, self.goal.pose.position.y, self.goal.pose.position.z])

		#----------------------------------------------------------------------------------

		# Observation subscribers
		odometry_sub = Subscriber(self.odom_topic, Odometry) 
		lidar_pcd_sub = Subscriber(self.cloud_base_topic, PointCloud2)

		ats = ApproximateTimeSynchronizer([odometry_sub, lidar_pcd_sub], queue_size=10, slop=0.1)
		ats.registerCallback(self.ats_callback)

		self.collision_sub = rospy.Subscriber(
			"/bebop/base_collision",
			Contact,
			self.callbackCollisions,
			queue_size=1,
			tcp_nodelay=True,
		)
		self.hit_obstacle = False
		#----------------------------------------------------------------------------------

		# CBF 

		self.bebop_radius = 0.1 
		self.k_att = 2.0
		self.alpha = 0.75
		self.D_obs = 0.3 + self.bebop_radius
		self.num_dim = 3

		#----------------------------------------------------------------------------------
		# ROS Timer
		self.config_cbf_frequency = 15       
		self.timer_cbf = rospy.Timer(rospy.Duration(1. / self.config_cbf_frequency),
									self._call_cbf)
		self.num = 0
		self.threshold = 0.3
		#----------------------------------------------------------------------------------
		print("[QUADROTOR CBF] Initialization completed!")

	def _call_cbf(self, event):
		if (self.pose is not None) and \
			(self.lidar_unit_vec is not None) and \
			(self.lidar_distance is not None) and \
			(self.goal_arr is not None) :

			cbf_vel = self.compute_cbf(self.pose, self.goal_arr, self.lidar_unit_vec, self.lidar_distance)

			# print("Iteration: ", self.num)

			# if cbf_vel != None:
			cmd_vels = self.rotation_mtx.T @ cbf_vel
			self.publish_cmd_vel_msg(cmd_vels, cbf_vel, self.num)

			self._visualize_goal()
			self._visualize_goal_tf(self.transformed_goal)
			self._update_path()
			self.num += 1

	def _update_path(self):
		if self.pose is None:
			return

		pose_stamped = PoseStamped()
		pose_stamped.header.stamp = rospy.Time.now()
		pose_stamped.header.frame_id = "odom"
		pose_stamped.pose.position.x = self.pose[0]
		pose_stamped.pose.position.y = self.pose[1]
		pose_stamped.pose.position.z = self.pose[2]

		quat = self.odom.pose.pose.orientation
		pose_stamped.pose.orientation = quat

		self.path_points.append(pose_stamped)
		self.path_msg.poses = self.path_points
		self.path_msg.header.stamp = rospy.Time.now()

		self._path_pub.publish(self.path_msg)

	def callbackCollisions(self, msg):

		if not self.hit_obstacle:
			if msg.objects_hit == ['collision']:
				print("Crashed")
				# self.hit_obstacle = True
				# rospy.signal_shutdown("You did not reach the goal!")

	def compute_cbf(self, pose, goal, lidar_unit_vec, lidar_distance):

		xo = pose[0]
		yo = pose[1]
		zo = pose[2]

		x_goal = goal[0]
		y_goal = goal[1]
		z_goal = goal[2]

		x = xo
		y = yo
		z = zo

		v_des = np.hstack((-self.k_att*(x-x_goal), -self.k_att*(y-y_goal), -self.k_att*(z-z_goal)))

		A_arr = []
		b_arr = []

		for i, (unit_vec, dist) in enumerate(zip(lidar_unit_vec, lidar_distance)): 
			dist_obs = dist - self.D_obs
			grad_hx = unit_vec.reshape(1, self.num_dim)
			A_arr.append(-grad_hx)
			b_arr.append(self.alpha * dist_obs)

		Q = np.identity(self.num_dim)
		q = -v_des
		A_in = np.vstack(A_arr)
		b_in = np.array(b_arr)
		
		solvers.options['show_progress'] = False

		# start_time = time.time()
		sol_data = solvers.qp(cvxopt.matrix(Q, tc='d'), cvxopt.matrix(q, tc='d'), cvxopt.matrix(A_in, tc='d'), cvxopt.matrix(b_in, tc='d'), None, None)
		# print("--- %s seconds ---" % (time.time() - start_time))

		return np.asarray(sol_data['x'])

	def handle_goal(self, goal_data):
		#------------------------
		## clear path
		self.path_points = []
		self.path_msg.poses = []
		#------------------------

		self.goal = goal_data
		# self.goal.pose.position.z = 3.0
		self.goal.pose.position.z = random.uniform(2.5,5.0)
		self.goal_arr = np.array([self.goal.pose.position.x, self.goal.pose.position.y, self.goal.pose.position.z])
		self.num = 0
		self.theta_arr = np.zeros(self.theta_avg_num)
		
		dx = self.goal.pose.position.x - self.pose[0]
		dy = self.goal.pose.position.y - self.pose[1]
		target_yaw = np.arctan2(dy, dx)
		self.heading_aligned = False
		self.target_yaw = target_yaw 

	def ats_callback(self, odom_data, lidar_pcd_ros):
		
		if lidar_pcd_ros is None or \
			odom_data is None:
			return
		#------------------------------------------

		## Odometry Processing

		self.odom = odom_data

		quaternion = (
			odom_data.pose.pose.orientation.x,
			odom_data.pose.pose.orientation.y,
			odom_data.pose.pose.orientation.z,
			odom_data.pose.pose.orientation.w
		)
		roll, pitch, yaw = transformations.euler_from_quaternion(quaternion)
		
		self.pose = np.array([
			odom_data.pose.pose.position.x, 
			odom_data.pose.pose.position.y, 
			odom_data.pose.pose.position.z,
			roll,
			pitch,
			yaw])

		linear_vel_odom = np.array([
			odom_data.twist.twist.linear.x,
			odom_data.twist.twist.linear.y,
			odom_data.twist.twist.linear.z
		])
		
		self.rotation_mtx = R.from_quat(np.array([self.odom.pose.pose.orientation.x, self.odom.pose.pose.orientation.y, self.odom.pose.pose.orientation.z, self.odom.pose.pose.orientation.w])).as_matrix()
		
		if self.goal is not None:    
			self.transformed_goal = self.rotation_mtx.T @ (self.goal_arr - np.array([self.pose[0], self.pose[1], self.pose[2]]))
		
		self.linear_vel = self.rotation_mtx.T @ linear_vel_odom

		# computing acceleration
		current_time = rospy.Time.now().to_sec()
		if self.prev_time is not None:
			self.linear_acc = (self.linear_vel - self.prev_linear_vel)/(current_time - self.prev_time + (1e-10))

		self.prev_time = current_time
		self.prev_linear_vel = self.linear_vel

		#------------------------------------------

		## PCD Processing
		query_point = np.array([0., 0., 0.])                   
		lidar_o3d_pcd = open3d_conversions.from_msg(lidar_pcd_ros)                            

		# Downsample the point clouds
		if np.asarray(lidar_o3d_pcd.points).shape[0] > self.L_max_lidar:
			lidar_o3d_pcd = lidar_o3d_pcd.farthest_point_down_sample(self.L_max_lidar)
		
		if lidar_o3d_pcd.is_empty():
			obs_pt = np.array([ 1e10, 1e10, 1e10])
			obs_pt = np.reshape(obs_pt, (1,3))
			lidar_o3d_pcd.points.extend(o3d.utility.Vector3dVector(obs_pt))    

		lidar_kdtree = o3d.geometry.KDTreeFlann(lidar_o3d_pcd)

		k = 200
		[lidar_k, lidar_idx, _] = lidar_kdtree.search_knn_vector_3d(query_point, k)
		# nearest points' coordinates
		lidar_nearest_points = np.asarray(lidar_o3d_pcd.points)[lidar_idx]
		# distance from the query point to the nearest points for hi(x)
		lidar_distance = np.linalg.norm(query_point - lidar_nearest_points, axis=1)
		# unit vector for ∇hi(x)
		lidar_vec = query_point - lidar_nearest_points
		lidar_unit_vec = lidar_vec/(lidar_distance[:, np.newaxis] + 1e-9)

		self.lidar_distance = lidar_distance
		self.lidar_unit_vec = lidar_unit_vec


	def compute_motion_cmd(self, vel, vel_global, num):
		
		cmd = Twist()
		vel_x = vel[0]
		vel_y = vel[1]
		vel_z = vel[2]

		v_mag = np.sqrt(vel_x**2 + vel_y**2 + vel_z**2)
		v_mag_clip = np.clip(v_mag, 0.0, 2.)
		theta = np.arctan2(vel_y, vel_x)
		phi = np.arctan2(np.sqrt(vel_x**2 + vel_y**2), vel_z)
		vel_x = v_mag_clip * np.sin(phi) * np.cos(theta)
		vel_y = v_mag_clip * np.sin(phi) * np.sin(theta)
		vel_z = v_mag_clip * np.cos(phi)

		cmd.linear.x = vel_x 
		cmd.linear.y = vel_y
		cmd.linear.z = vel_z

		return cmd

	def publish_cmd_vel_msg(self, vel, vel_global, num):
		try:
			cmd = self.compute_motion_cmd(vel, vel_global, num)
			self._vel_pub.publish(cmd)
		except:
			print("")

	def _visualize_goal(self):

		if self.goal is None:
			return
		
		# scale.x is the arrow length, scale.y is the arrow width and scale.z is the arrow height.
		marker = Marker()
		marker.type = Marker.ARROW
		marker.action = Marker.ADD
		marker.scale = Vector3(1.0, 0.15, 0.15)          
		marker.color.r = 1.0                               
		marker.color.g = 1.0                        
		marker.color.b = 0.0                         
		marker.color.a = 1.0                           

		marker.ns = "goal_marker"

		marker.pose.position.x = self.goal.pose.position.x
		marker.pose.position.y = self.goal.pose.position.y
		marker.pose.position.z = self.goal.pose.position.z
		marker.pose.orientation.x = self.goal.pose.orientation.x
		marker.pose.orientation.y = self.goal.pose.orientation.y
		marker.pose.orientation.z = self.goal.pose.orientation.z
		marker.pose.orientation.w = self.goal.pose.orientation.w

		marker.id = 0
		marker.header.stamp = rospy.get_rostime()
		marker.lifetime = rospy.Duration(0.0)
		marker.header.frame_id = "odom"

		self._arrowmarker_pub.publish(marker)

	def _visualize_goal_tf(self, transformed_goal):

		if self.transformed_goal is None:
			return
		
		marker = Marker()
		marker.type = Marker.SPHERE
		marker.action = Marker.ADD
		marker.scale = Vector3(0.1, 0.1, 0.1)              
		marker.color.r = 0.0                               
		marker.color.g = 0.0                       
		marker.color.b = 0.5                         
		marker.color.a = 1.0                               

		marker.ns = "goaltf_marker"

		marker.pose.position.x = transformed_goal[0]
		marker.pose.position.y = transformed_goal[1]
		marker.pose.position.z = transformed_goal[2]

		marker.id = 0
		marker.header.stamp = rospy.get_rostime()
		marker.lifetime = rospy.Duration(0.0)
		marker.header.frame_id = "base_link"

		self._spheremarker_pub.publish(marker)


if __name__ == '__main__':
	quadrotor_navigation = ControlBarrierFunction()
	rospy.spin()