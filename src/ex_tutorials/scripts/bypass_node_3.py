#!/usr/bin/env python

import rospy
from hello_tutorials.msg import AddParam


def callback(data):
    rospy.loginfo("Step_2 Receive Data [a=%s b=%s]", data.a, data.b)
    rospy.loginfo("Step_2: a[%d] + b[%d] = %d", data.a, data.b, data.a + data.b)
    

if __name__ == '__main__':
    try:
        rospy.init_node('Over', anonymous=True)
        rospy.Subscriber('Step_2', AddParam, callback)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
