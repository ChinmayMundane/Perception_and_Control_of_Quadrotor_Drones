#!/usr/bin/env python3

import rospy
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA

def publish_obstacle(id, x, y, z, radius, height):
    marker = Marker()
    marker.header.frame_id = "odom"
    marker.header.stamp = rospy.Time.now()
    marker.ns = "obstacles"
    marker.id = id
    marker.type = Marker.CYLINDER
    marker.action = Marker.ADD
    marker.pose.position.x = x
    marker.pose.position.y = y
    marker.pose.position.z = z
    marker.pose.orientation.x = 0.0
    marker.pose.orientation.y = 0.0
    marker.pose.orientation.z = 0.0
    marker.pose.orientation.w = 1.0
    marker.scale.x = 2 * radius  
    marker.scale.y = 2 * radius
    marker.scale.z = height
    marker.color = ColorRGBA(0.0, 0.0, 0.0, 1.0)  
    marker.lifetime = rospy.Duration(0)
    return marker

if __name__ == '__main__':
    rospy.init_node('obstacle_marker_publisher')
    pub = rospy.Publisher('/visualization_marker', Marker, queue_size=10)
    rate = rospy.Rate(1) 

    while not rospy.is_shutdown():
        marker1 = publish_obstacle(id=1, x=1, y=-2, z=4, radius=0.5, height=8.0)
        pub.publish(marker1)

        marker2 = publish_obstacle(id=2, x=2.5, y=-3, z=4, radius=0.5, height=8.0)
        pub.publish(marker2)

        rate.sleep()
