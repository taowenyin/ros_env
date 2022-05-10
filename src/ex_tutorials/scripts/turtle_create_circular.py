#!/usr/bin/env python
# coding: utf-8

import rospy

from turtlesim.srv import Spawn
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist


if __name__ == '__main__':
    rospy.init_node('Turtle_Spawn')

    rospy.wait_for_service('/spawn')
    turtle_spawn = rospy.ServiceProxy('/spawn', Spawn)
    turtle_spawn(5, 5, 0, 'turtle2')

    turtle_vel = Twist()
    turtle_vel.linear.x = 2
    turtle_vel.angular.z = 1.57

    turtle2_vel = rospy.Publisher('/turtle2/cmd_vel', Twist,
                                 queue_size=1)
    turtle1_vel = rospy.Publisher('/turtle1/cmd_vel', Twist,
                                 queue_size=1)

    loop_rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        turtle1_vel.publish(turtle_vel)
        turtle2_vel.publish(turtle_vel)

        loop_rate.sleep()
