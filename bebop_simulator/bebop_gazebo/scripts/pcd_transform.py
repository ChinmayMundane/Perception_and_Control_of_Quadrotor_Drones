#!/usr/bin/env python3

"""Transform the simulated lidar cloud into the Bebop base frame."""

import rospy
import tf2_ros
import tf2_sensor_msgs.tf2_sensor_msgs as tf2_sensor_msgs
from sensor_msgs.msg import PointCloud2


class PointCloudTransformer:
    def __init__(self):
        rospy.init_node("pointcloud_transformer")
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.publisher = rospy.Publisher(
            "/lidar_pcd_base_link", PointCloud2, queue_size=1
        )
        rospy.Subscriber("/velodyne_points", PointCloud2, self.transform)

    def transform(self, cloud):
        try:
            transform = self.tf_buffer.lookup_transform(
                "base_link", "velodyne", rospy.Time(0), rospy.Duration(1.0)
            )
            transformed_cloud = tf2_sensor_msgs.do_transform_cloud(
                cloud, transform
            )
            transformed_cloud.header.stamp = cloud.header.stamp
            self.publisher.publish(transformed_cloud)
        except (tf2_ros.LookupException, tf2_ros.ExtrapolationException) as error:
            rospy.logwarn_throttle(2.0, "Lidar transform unavailable: %s", error)


if __name__ == "__main__":
    PointCloudTransformer()
    rospy.spin()
