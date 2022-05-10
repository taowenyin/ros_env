#!/usr/bin/env python

import rospy
from hello_tutorials.msg import AddParam


if __name__ == '__main__':
    rospy.init_node('Calculate', anonymous=True)
    pub = rospy.Publisher('Step_1', AddParam, queue_size=10)
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        msg = AddParam()
        msg.a = 2
        msg.b = 1
        rospy.loginfo("Step_1: Talk Add Param message;[ %d, %d]",
                      msg.a, msg.b)
        pub.publish(msg)
        rate.sleep()
