#!/usr/bin/env python

import rospy
import tf2_ros

from geometry_msgs.msg import TransformStamped

if __name__ == '__main__':
    rospy.init_node('static_tf_broadcaster')

    br = tf2_ros.StaticTransformBroadcaster()

    t = TransformStamped()
    t.header.stamp = rospy.Time.now()
    t.header.frame_id = 'world'
    t.child_frame_id = 'carrot_static'

    t.transform.translation.x = 0.0
    t.transform.translation.y = 0.0
    t.transform.translation.z = 2.0

    t.transform.rotation.x = 0.0
    t.transform.rotation.y = 0.0
    t.transform.rotation.z = 0.0
    t.transform.rotation.w = 1.0

    br.sendTransform(t)

    rospy.spin()
