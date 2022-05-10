#! /usr/bin/env python

from __future__ import print_function
from enum import Enum

import rospy
# Brings in the SimpleActionClient
import actionlib
import sys

# Brings in the messages used by the fibonacci action, including the
# goal message and the result message.
import actionlib_tutorials.msg


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
    rospy.loginfo("Result: %s", ', '.join([str(n) for n in res.sequence]))


def active_cb():
    rospy.loginfo('Goal Activate')


def feedback_cb(fb):
    rospy.loginfo('Get Goal Feedback: %s',
                  ', '.join([str(n) for n in fb.sequence]))


def fibonacci_client():
    # Creates the SimpleActionClient, passing the type of the action
    # (FibonacciAction) to the constructor.
    client = actionlib.SimpleActionClient('fibonacci',
                                          actionlib_tutorials.msg.FibonacciAction)

    # Waits until the action server has started up and started
    # listening for goals.
    client.wait_for_server()

    # Creates a goal to send to the action server.
    goal = actionlib_tutorials.msg.FibonacciGoal(order=8)

    # Sends the goal to the action server.
    client.send_goal(goal, done_cb=done_cb, active_cb=active_cb, feedback_cb=feedback_cb)

    # Waits for the server to finish performing the action.
    client.wait_for_result()

    # Prints out the result of executing the action
    return client.get_result()  # A FibonacciResult


if __name__ == '__main__':
    try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        if len(sys.argv) == 2:
            node_name = str(sys.argv[1])
        else:
            print("%s [x]" % sys.argv[0])
            sys.exit(1)

        rospy.init_node(node_name)
        result = fibonacci_client()
    except rospy.ROSInterruptException:
        print("program interrupted before completion", file=sys.stderr)
