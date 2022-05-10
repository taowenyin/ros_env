#!/usr/bin/env python

import rospy
from hello_tutorials.msg import Person


def talker():
    pub = rospy.Publisher('Person_chatter', Person, queue_size=10)
    rospy.init_node('Person_talker', anonymous=True)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        msg = Person()
        msg.name = 'Lee'
        msg.sex = msg.female
        msg.age = 18
        rospy.loginfo("this publish person message;[ %s, %d, %d]",
                      msg.name, msg.age, msg.sex)
        pub.publish(msg)
        rate.sleep()


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
