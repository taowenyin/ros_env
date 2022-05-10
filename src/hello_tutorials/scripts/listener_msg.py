#!/usr/bin/env python

import rospy
from hello_tutorials.msg import Person


def callback(data):
    rospy.loginfo(
        "this is Person Publisher messgae:[name:%s, age:%d, sex:%d]", data.name, data.age, data.sex)


def listener():
    rospy.init_node('Person_listener', anonymous=True)
    rospy.Subscriber('Person_chatter', Person, callback)
    rospy.spin()


if __name__ == '__main__':
    listener()
