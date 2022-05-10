#!/usr/bin/env python

import rospy
from hello_tutorials.msg import AddParam


def callback(data):
    rospy.loginfo("Step_1 Receive Data [a=%s b=%s]", data.a, data.b)
    rospy.loginfo("Step_1: a[%d] + b[%d] = %d", data.a, data.b, data.a + data.b)

    msg = AddParam()
    msg.a = data.b
    msg.b = data.a + data.b

    pub.publish(msg)


if __name__ == '__main__':
    try:
        rospy.init_node('Bypass', anonymous=True)

        rospy.Subscriber('Step_1', AddParam, callback)
        pub = rospy.Publisher('Step_2', AddParam, queue_size=10)

        rospy.spin()
    except rospy.ROSInterruptException:
        pass
