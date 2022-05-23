#!/usr/bin/env python
# coding: utf-8

import rospy
import math
import time
import numpy as np

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

            rospy.loginfo('y_goal = %f, self.y = %f, yaw = %f, desired_angle_goal = %f angular_speed = %f',
                          y_goal, self.y, self.yaw, desired_angle_goal, angular_speed)

            velocity_message.linear.x = linear_speed
            velocity_message.angular.z = angular_speed

            self.velocity_publisher.publish(velocity_message)
            rospy.loginfo('Go to goal x = %f y = %f', self.x, self.y)

            if distance < 0.01:
                break

    def rotate(self, angular_speed_degree, relative_angle_degree, clockwise):
        velocity_message = Twist()
        velocity_message.linear.x = 0
        velocity_message.linear.y = 0
        velocity_message.linear.z = 0
        velocity_message.angular.x = 0
        velocity_message.angular.y = 0
        velocity_message.angular.z = 0

        angular_speed = math.radians(angular_speed_degree)

        if clockwise:
            velocity_message.angular.z = -1 * abs(angular_speed)
        else:
            velocity_message.angular.z = abs(angular_speed)

        loop_rate = rospy.Rate(10)

        t_0 = rospy.Time.now().to_sec()
        while not rospy.is_shutdown():
            rospy.loginfo('Turtlesim rotates')
            self.velocity_publisher.publish(velocity_message)

            t_1 = rospy.Time.now().to_sec()
            current_angle_degree = (t_1 - t_0) * angular_speed_degree
            loop_rate.sleep()

            if current_angle_degree > relative_angle_degree:
                rospy.loginfo('Reached')
                break

        velocity_message.angular.z = 0
        self.velocity_publisher.publish(velocity_message)

    def set_desired_orientation(self, desired_angle_degree):
        desired_angle_radians = math.radians(desired_angle_degree)

        if self.yaw < 0:
            self_yaw = 2 * math.pi + self.yaw
        else:
            self_yaw = self.yaw

        relative_angle_radians = desired_angle_radians - self_yaw
        if relative_angle_radians < 0:
            clockwise = 1
        else:
            clockwise = 0

        self.rotate(30, math.degrees(abs(relative_angle_radians)), clockwise)


if __name__ == '__main__':
    rospy.init_node('turtle_grid', anonymous=True)

    turtle_sim_motion = TurtleSimMotion()
    pose_subscriber = rospy.Subscriber('/turtle1/pose', Pose,
                                       turtle_sim_motion.pose_callback)
    time.sleep(2)

    # turtle_sim_motion.set_desired_orientation(270)
    # time.sleep(2)
    # turtle_sim_motion.set_desired_orientation(360)

    turtle_sim_motion.go_to_goal(x_goal=1, y_goal=1)
    turtle_sim_motion.set_desired_orientation(0)
    time.sleep(2)
    # --------------------------
    turtle_sim_motion.go_to_goal(x_goal=10, y_goal=1)
    turtle_sim_motion.set_desired_orientation(90)
    # --------------------------
    turtle_sim_motion.go_to_goal(x_goal=10, y_goal=10)
    turtle_sim_motion.set_desired_orientation(180)
    # --------------------------
    turtle_sim_motion.go_to_goal(x_goal=1, y_goal=10)
    turtle_sim_motion.set_desired_orientation(270)
    # --------------------------
    turtle_sim_motion.go_to_goal(x_goal=1, y_goal=1)
    turtle_sim_motion.set_desired_orientation(360)
