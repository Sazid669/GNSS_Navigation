#!/usr/bin/env python3

import rospy
import tf2_ros
import geometry_msgs.msg

# Just incase we need map to odom 
class PublishStaticTransform:
    def __init__(self):
        self.static_broadcaster = tf2_ros.StaticTransformBroadcaster()
        self.StaticMapOdom()

    def StaticMapOdom(self):
        static_transform_stamped = geometry_msgs.msg.TransformStamped()
        static_transform_stamped.header.stamp = rospy.Time.now()
        static_transform_stamped.header.frame_id = "map"
        static_transform_stamped.child_frame_id = "odom"
        static_transform_stamped.transform.translation.x = 0.0
        static_transform_stamped.transform.translation.y = 0.0
        static_transform_stamped.transform.translation.z = 0.0
        static_transform_stamped.transform.rotation.x = 0.0
        static_transform_stamped.transform.rotation.y = 0.0
        static_transform_stamped.transform.rotation.z = 0.0
        static_transform_stamped.transform.rotation.w = 1.0

        self.static_broadcaster.sendTransform(static_transform_stamped)