#!/usr/bin/env python

import time
import rospy
import math
import sys

from geometry_msgs.msg import Twist
from turtlesim.msg import Pose


class TurtleSimMotion:
    def __init__(self):
        self.x, self.y, self.z, self.yaw = 0, 0, 0, 0

        self.velocity_publisher = rospy.Publisher('/turtle1/cmd_vel',
                                                  Twist, queue_size=10)

    def pose_callback(self, pose_message):
        self.x = pose_message.x
        self.y = pose_message.y
        self.yaw = pose_message.theta

        rospy.loginfo('Pose Callback x = %f y = %f yaw = %f',
                      self.x, self.y, self.yaw)

    def go_to_goal(self, x_goal, y_goal):
        velocity_message = Twist()

        while not rospy.is_shutdown():
            K_linear = 0.5
            distance = abs(math.sqrt(
                (x_goal - self.x) ** 2 + (y_goal - self.y) ** 2))
            linear_speed = distance * K_linear

            K_angular = 4.0
            desired_angle_goal = math.atan2(y_goal - self.y, x_goal - self.x)
            angular_speed = (desired_angle_goal - self.yaw) * K_angular

            velocity_message.linear.x = linear_speed
            velocity_message.angular.z = angular_speed

            self.velocity_publisher.publish(velocity_message)
            rospy.loginfo('Go to goal x = %f y = %f', self.x, self.y)

            if distance < 0.01:
                break


if __name__ == '__main__':
    if len(sys.argv) == 3:
        x_goal = int(sys.argv[1])
        y_goal = int(sys.argv[2])
    else:
        print("%s [x_goal y_goal]" % sys.argv[0])
        sys.exit(1)

    rospy.init_node('turtlesim_goal_motion', anonymous=True)

    turtle_sim_motion = TurtleSimMotion()
    pose_subscriber = rospy.Subscriber('/turtle1/pose', Pose,
                                       turtle_sim_motion.pose_callback)
    time.sleep(2)

    turtle_sim_motion.go_to_goal(x_goal=x_goal, y_goal=y_goal)
