#!/usr/bin/env python

import rospy
import tf2_ros

from geometry_msgs.msg import TransformStamped
from tf_conversions import transformations
from turtlesim.msg import Pose


def handle_turtle_pose(msg, turtle_name):
    br = tf2_ros.TransformBroadcaster()

    t = TransformStamped()
    t.header.stamp = rospy.Time.now()
    t.header.frame_id = 'world'
    t.child_frame_id = turtle_name

    t.transform.translation.x = msg.x
    t.transform.translation.y = msg.y
    t.transform.translation.z = 0.0

    q = transformations.quaternion_from_euler(0, 0, msg.theta)
    t.transform.rotation.x = q[0]
    t.transform.rotation.y = q[1]
    t.transform.rotation.z = q[2]
    t.transform.rotation.w = q[3]

    br.sendTransform(t)


if __name__ == '__main__':
    rospy.init_node('baselink_tf_broadcaster')
    turtle_name = rospy.get_param('~turtle')
    rospy.Subscriber('/%s/pose' % turtle_name, Pose,
                     handle_turtle_pose, turtle_name)

    rospy.spin()
