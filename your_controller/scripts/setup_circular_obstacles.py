#!/usr/bin/env python3
import rospy
import numpy as np

from sensor_msgs.msg import PointCloud2
import std_msgs.msg
import sensor_msgs.point_cloud2 as pcl2
from geometry_msgs.msg import PoseStamped
from message_filters import ApproximateTimeSynchronizer, Subscriber


DEFAULT_OBSTACLE_TOPICS = [
    '/vrpn_client_node/box1/pose',
    '/vrpn_client_node/box2/pose',
    '/vrpn_client_node/box3/pose',
]


class CircularObstaclePublisher:
    def __init__(self):
        self.radius = rospy.get_param('~radius', 0.6)
        self.height = rospy.get_param('~height', 1.0)
        self.frame_id = rospy.get_param('~frame_id', 'odom')
        self.num_samples = int(rospy.get_param('~num_samples', 90))
        self.obstacle_topics = rospy.get_param('~obstacle_topics', DEFAULT_OBSTACLE_TOPICS)

        self.individual_pcl_topics = [
            '/obs_pcl_topic_1',
            '/obs_pcl_topic_2',
            '/obs_pcl_topic_3',
        ]
        self.combined_pcl_topic = '/obs_pcl_topic'

        self.publishers = [
            rospy.Publisher(topic, PointCloud2, queue_size=10)
            for topic in self.individual_pcl_topics
        ]
        self.combined_pub = rospy.Publisher(self.combined_pcl_topic, PointCloud2, queue_size=10)

        self.subscribers = [Subscriber(topic, PoseStamped) for topic in self.obstacle_topics]
        self.sync = ApproximateTimeSynchronizer(
            self.subscribers,
            queue_size=10,
            slop=0.1,
        )
        self.sync.registerCallback(self.callback)

        rospy.loginfo(
            'Circular obstacle pcl publisher initialized with %d obstacle topics.',
            len(self.obstacle_topics),
        )

    @staticmethod
    def _extract_xy(msg):
        pose = msg.pose if hasattr(msg, 'pose') else msg
        return pose.position.x, pose.position.y

    def _create_circle_points(self, center_x, center_y):
        angles = np.linspace(0.0, 2.0 * np.pi, self.num_samples, endpoint=False)
        x_points = center_x + self.radius * np.cos(angles)
        y_points = center_y + self.radius * np.sin(angles)
        z_points = np.full(x_points.shape, self.height, dtype=np.float32)
        return np.column_stack((x_points, y_points, z_points)).astype(np.float32)

    def _make_cloud(self, points, stamp):
        header = std_msgs.msg.Header()
        header.stamp = stamp
        header.frame_id = self.frame_id
        return pcl2.create_cloud_xyz32(header, points)

    def callback(self, *msgs):
        if len(msgs) != len(self.publishers):
            rospy.logwarn(
                'Received %d obstacle poses, but there are %d publishers. Skipping this callback.',
                len(msgs),
                len(self.publishers),
            )
            return

        stamp = rospy.Time.now()
        all_cloud_points = []

        for idx, msg in enumerate(msgs):
            center_x, center_y = self._extract_xy(msg)
            circular_points = self._create_circle_points(center_x, center_y)
            self.publishers[idx].publish(self._make_cloud(circular_points, stamp))
            all_cloud_points.append(circular_points)

        if all_cloud_points:
            combined_points = np.concatenate(all_cloud_points, axis=0)
            self.combined_pub.publish(self._make_cloud(combined_points, stamp))


if __name__ == '__main__':
    try:
        rospy.init_node('circular_obstacle_publisher')
        CircularObstaclePublisher()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
