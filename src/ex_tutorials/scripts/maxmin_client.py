#! /usr/bin/env python

from __future__ import print_function
from enum import Enum

import rospy
import actionlib
import sys
import ex_tutorials.msg


class ActionStatus(Enum):
    PENDING = 0
    ACTIVE = 1
    PREEMPTED = 2
    SUCCEEDED= 3
    ABORTED = 4
    REJECTED = 5
    PREEMPTING = 6
    RECALLING = 7
    RECALLED = 8


def done_cb(state, res):
    rospy.loginfo('Goal State: %s', ActionStatus(state))
    rospy.loginfo('Result: [Data Min = %d] [Data Max = %d]',
                  res.min, res.max)


def active_cb():
    rospy.loginfo('Goal Activate')


def feedback_cb(fb):
    rospy.loginfo('Get Goal Feedback: [Data Count = %d] '
                  '[Data = %d] [Data Min = %d] '
                  '[Data Max = %d]',
                  fb.sample, fb.data, fb.min, fb.max)


def maxmin_client():
    client = actionlib.SimpleActionClient('maxmin_server',
                                          ex_tutorials.msg.MaxMinAction)
    client.wait_for_server()

    goal = ex_tutorials.msg.MaxMinGoal()
    goal.samples = 8
    client.send_goal(goal, done_cb=done_cb, active_cb=active_cb,
                     feedback_cb=feedback_cb)
    client.wait_for_result()

    return client.get_result()


if __name__ == '__main__':
    try:
        rospy.init_node('maxmin_client')
        result = maxmin_client()
    except rospy.ROSInterruptException:
        print("program interrupted before completion", file=sys.stderr)
