#!/usr/bin/env python

import rospy

from std_msgs.msg import String


class ActionLibSimServer:
    def __init__(self):
        self.status_pub = rospy.Publisher('Status', String, queue_size=10)
        self.result_pub = rospy.Publisher('Result', String, queue_size=10)
        self.feedback_pub = rospy.Publisher('Feedback', String, queue_size=10)

    def set_goal_cb(self, cb):
        rospy.Subscriber('Goal', String, cb)

    def set_cancel_cb(self, cb):
        rospy.Subscriber('Cancel', String, cb)

    def status(self, msg):
        self.status_pub.publish(msg)

    def result(self, msg):
        self.result_pub.publish(msg)

    def feedback(self, msg):
        self.feedback_pub.publish(msg)


if __name__ == '__main__':
    actionlib_sim = ActionLibSimServer()

    rospy.spin()
