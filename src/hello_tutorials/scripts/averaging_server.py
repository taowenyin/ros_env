#! /usr/bin/env python

import rospy
import actionlib
import math
import numpy as np

from std_msgs.msg import Int32
from hello_tutorials.msg import *


class AveragingAction:
    def __init__(self, name):
        self._action_name = name
        self._data_count = 0
        self._sum = 0
        self._sum_sq = []
        self._goal = 0

        self._feedback = hello_tutorials.msg.AveragingFeedback()
        self._result = hello_tutorials.msg.AveragingResult()

        self._as = actionlib.SimpleActionServer(
            self._action_name,
            hello_tutorials.msg.AveragingAction,
            auto_start=False)

        self._as.register_goal_callback(self.goal_cb)
        self._as.register_preempt_callback(self.preempt_cb)
        rospy.Subscriber('random_number', Int32, self.analysis_cb)

        self._as.start()

    def goal_cb(self):
        self._data_count = 0
        self._sum = 0
        self._sum_sq = []
        self._goal = self._as.accept_new_goal().samples

    def analysis_cb(self, msg):
        if not self._as.is_active():
            return

        self._data_count += 1
        self._feedback.sample = self._data_count
        self._feedback.data = msg.data

        self._sum += msg.data
        self._feedback.mean = self._sum / self._data_count
        self._sum_sq.append(msg.data)
        self._feedback.std_dev = math.sqrt(np.sum(np.power(
            np.asarray(self._sum_sq) - self._feedback.mean, 2)) / self._data_count)
        self._as.publish_feedback(self._feedback)
        rospy.loginfo('Data sequence: [%s]',
                      ', '.join([str(n) for n in self._sum_sq]))

        if self._data_count > (self._goal - 1):
            self._result.mean = self._feedback.mean
            self._result.std_dev = self._feedback.std_dev

            if self._result.mean < 5.0:
                rospy.loginfo("%s: Aborted", self._action_name)
                self._as.set_aborted(self._result)
            else:
                rospy.loginfo("%s: Succeeded", self._action_name)
                self._as.set_succeeded(self._result)

    def preempt_cb(self):
        rospy.loginfo('%s: Preempted', self._action_name)
        self._as.set_preempted()


if __name__ == '__main__':
    rospy.init_node('averaging')
    server = AveragingAction('averaging_server')
    rospy.spin()
