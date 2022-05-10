#!/usr/bin/env python

import rospy
import math
import tf2_ros

from turtlesim.srv import Spawn
from geometry_msgs.msg import Twist
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException


if __name__ == '__main__':
    rospy.init_node('turtle_tf_listener')

    tf_buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tf_buffer)

    rospy.wait_for_service('spawn')
    spawner = rospy.ServiceProxy('spawn', Spawn)
    spawner(4, 2, 0, 'turtle2')

    turtle_vel = rospy.Publisher('turtle2/cmd_vel', Twist,
                                 queue_size=1)

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        try:
            trans = tf_buffer.lookup_transform('turtle2', 'turtle1',
                                               rospy.Time(0))
        except (LookupException, ConnectivityException, ExtrapolationException):
            continue

        cmd = Twist()
        cmd.angular.z = 4 * math.atan2(trans.transform.translation.y,
                                       trans.transform.translation.x)
        cmd.linear.x = 0.5 * math.sqrt(trans.transform.translation.x ** 2 +
                                       trans.transform.translation.y ** 2)
        turtle_vel.publish(cmd)

        rate.sleep()
