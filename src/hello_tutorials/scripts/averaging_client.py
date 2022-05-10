#! /usr/bin/env python

import rospy
import actionlib
import threading
import random

from hello_tutorials.msg import *
from enum import Enum

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


def spin_thread():
    rospy.spin()


def done_cb(state, res):
    rospy.loginfo('Goal State: %s', ActionStatus(state))
    rospy.loginfo('Result: [Data Mean = %0.2f] [Data Std Dev = %0.2f]',
                  res.mean, res.std_dev)


def active_cb():
    rospy.loginfo('Goal Activate')


def feedback_cb(fb):
    rospy.loginfo('Get Goal Feedback: [Data Count = %d] '
                  '[Data = %d] [Data Mean = %0.2f] '
                  '[Data Std Dev = %0.2f]',
                  fb.sample, fb.data, fb.mean, fb.std_dev)


if __name__ == '__main__':
    rospy.init_node('Averaging_Client_' + str(random.randint(1, 10)))

    client = actionlib.SimpleActionClient('averaging_server', AveragingAction)

    spin_thread = threading.Thread(target=spin_thread)
    spin_thread.start()

    rospy.loginfo('Waiting for action server to start.')
    client.wait_for_server()

    rospy.loginfo("Action server started, sending goal.")

    goal = AveragingGoal()
    goal.samples = 10
    client.send_goal(goal, done_cb=done_cb, active_cb=active_cb, feedback_cb=feedback_cb)

    finished_before_timeout = client.wait_for_result(timeout=rospy.Duration(30))

    if not finished_before_timeout:
        rospy.loginfo("Action did not finish before the time out.")

    rospy.signal_shutdown('')
    spin_thread.join()
