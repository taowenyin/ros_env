#!/usr/bin/env python

import rospy
import tf2_ros

from tf2_ros import LookupException, ConnectivityException, ExtrapolationException


if __name__ == '__main__':
    rospy.init_node('laser_tf_listener')

    tf_buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tf_buffer)

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        try:
            trans_base = tf_buffer.lookup_transform('turtle1', 'world',
                                                    rospy.Time(0))
            trans_laser = tf_buffer.lookup_transform('laser_point', 'turtle1',
                                                     rospy.Time(0))
        except (LookupException, ConnectivityException, ExtrapolationException):
            continue

        rospy.loginfo('base_laser: (%.2f, %.2f. %.2f) ----'
                      '-> base_link: (%.2f, %.2f, %.2f)',
                      trans_laser.transform.translation.x +
                      trans_base.transform.translation.x,
                      trans_laser.transform.translation.y +
                      trans_base.transform.translation.y,
                      trans_laser.transform.translation.z +
                      trans_base.transform.translation.z,
                      trans_base.transform.translation.x,
                      trans_base.transform.translation.y,
                      trans_base.transform.translation.z)

        rate.sleep()
