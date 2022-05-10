#! /usr/bin/env python

import rospy
import actionlib

from std_msgs.msg import Int32
from ex_tutorials.msg import *


class MaxMinAction:
    def __init__(self, name):
        self._action_name = name
        self._data_count = 0
        self._sum_sq = []
        self._goal = 0

        self._feedback = ex_tutorials.msg.MaxMinFeedback()
        self._result = ex_tutorials.msg.MaxMinResult()

        self._as = actionlib.SimpleActionServer(
            self._action_name,
            ex_tutorials.msg.MaxMinAction,
            auto_start=False)

        self._as.register_goal_callback(self.goal_cb)
        self._as.register_preempt_callback(self.preempt_cb)
        rospy.Subscriber('random_number', Int32, self.analysis_cb)

        self._as.start()

    def goal_cb(self):
        self._data_count = 0
        self._sum_sq = []
        self._goal = self._as.accept_new_goal().samples

    def analysis_cb(self, msg):
        if not self._as.is_active():
            return

        self._data_count += 1
        self._feedback.sample = self._data_count
        self._feedback.data = msg.data

        self._sum_sq.append(msg.data)
        self._feedback.min = min(self._sum_sq)
        self._feedback.max = max(self._sum_sq)
        self._as.publish_feedback(self._feedback)
        rospy.loginfo('Data sequence: [%s]',
                      ', '.join([str(n) for n in self._sum_sq]))

        if self._data_count > (self._goal - 1):
            self._result.min = min(self._sum_sq)
            self._result.max = max(self._sum_sq)

            self._as.set_succeeded(self._result)

    def preempt_cb(self):
        rospy.loginfo('%s: Preempted', self._action_name)
        self._as.set_preempted()


if __name__ == '__main__':
    rospy.init_node('maxmin')
    server = MaxMinAction('maxmin_server')
    rospy.spin()
