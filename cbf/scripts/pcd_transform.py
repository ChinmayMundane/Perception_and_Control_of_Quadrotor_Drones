#!/usr/bin/python3

import rospy
import tf2_ros
import tf2_sensor_msgs.tf2_sensor_msgs as tf2_sensor_msgs
from sensor_msgs.msg import PointCloud2

class PointCloudTransformer:
    def __init__(self):
        rospy.init_node('pointcloud_transformer', anonymous=True)
        print("3D LIDAR PointCloud transform node initialised")

        self.lidar_tf_base_buffer = tf2_ros.Buffer()
        self.lidar_tf_base_listener = tf2_ros.TransformListener(self.lidar_tf_base_buffer)

        self.lidar_pcd_sub = rospy.Subscriber("/velodyne_points", PointCloud2, self.lidar_pcd_callback)

        self.lidar_base_link_pub = rospy.Publisher("/lidar_pcd_base_link", PointCloud2, queue_size=1)
    
    def lidar_pcd_callback(self, pointcloud_msg):
        try:
            transform_base = self.lidar_tf_base_buffer.lookup_transform("base_link", "velodyne", rospy.Time(0), rospy.Duration(1.0))
            transformed_base_cloud = tf2_sensor_msgs.do_transform_cloud(pointcloud_msg, transform_base)
            
            transformed_base_cloud.header.stamp = pointcloud_msg.header.stamp

            self.lidar_base_link_pub.publish(transformed_base_cloud)
            
        except tf2_ros.LookupException as e:
            rospy.logwarn("Could not find transform: {}".format(e))
        except tf2_ros.ExtrapolationException as e:
            rospy.logwarn("Extrapolation error: {}".format(e))

if __name__ == '__main__':
    try:
        # Create the transformer and run it
        transformer = PointCloudTransformer()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass