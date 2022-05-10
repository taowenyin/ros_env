#!/usr/bin/env python

import rospy

from std_msgs.msg import String


class ActionLibSimClient:
    def __init__(self):
        self.goal_pub = rospy.Publisher('Goal', String, queue_size=10)
        self.cancel_pub = rospy.Publisher('Cancel', String, queue_size=10)

    def set_status_cb(self, cb):
        rospy.Subscriber('Status', String, cb)

    def set_result_cb(self, cb):
        rospy.Subscriber('Result', String, cb)

    def set_feedback_cb(self, cb):
        rospy.Subscriber('Feedback', String, cb)

    def goal(self, msg):
        self.goal_pub.publish(msg)

    def cancel(self, msg):
        self.cancel_pub.publish(msg)


if __name__ == '__main__':
    actionlib_sim_client = ActionLibSimClient()

    rospy.spin()
