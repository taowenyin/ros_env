#!/usr/bin/env python

import rospy

from geometry_msgs.msg import TransformStamped
from tf2_msgs.msg import TFMessage

if __name__ == '__main__':
    rospy.init_node('fixed_tf_broadcaster')

    pub_tf = rospy.Publisher("/tf", TFMessage, queue_size=1)
    rate = rospy.Rate(10.0)

    while not rospy.is_shutdown():
        t = TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = 'world'
        t.child_frame_id = 'carrot_fixed'

        t.transform.translation.x = 0.0
        t.transform.translation.y = 2.0
        t.transform.translation.z = 0.0

        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0

        tfm = TFMessage([t])
        pub_tf.publish(tfm)

        rate.sleep()

    rospy.spin()
