#!/usr/bin/env python
# coding: utf-8

import rospy

from turtlesim.srv import Spawn
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist


def turtle2_pose_cb(msg):
    rospy.loginfo('Pose Callback x = %f y = %f yaw = %f',
                  msg.x, msg.y, msg.theta)


def teleop_vel_cb(msg, turtle_vel):
    turtle_vel.publish(msg)


if __name__ == '__main__':
    rospy.init_node('Turtle_Spawn')

    rospy.wait_for_service('/spawn')
    turtle_spawn = rospy.ServiceProxy('/spawn', Spawn)
    turtle_spawn(5, 5, 0, 'turtle2')

    turtle_vel = rospy.Publisher('turtle2/cmd_vel', Twist,
                                 queue_size=1)

    rospy.Subscriber('turtle2/pose', Pose, turtle2_pose_cb)
    rospy.Subscriber('/turtle1/cmd_vel', Twist, teleop_vel_cb, turtle_vel)

    rospy.spin()
