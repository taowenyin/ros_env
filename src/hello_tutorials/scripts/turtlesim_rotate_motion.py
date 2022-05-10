#!/usr/bin/env python

import time
import rospy
import math
import sys

from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from distutils.util import strtobool


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

    def rotate(self, angular_speed_degree, relative_angle_degree, clockwise):
        velocity_message = Twist()
        velocity_message.linear.x = 0
        velocity_message.linear.y = 0
        velocity_message.linear.z = 0
        velocity_message.angular.x = 0
        velocity_message.angular.y = 0
        velocity_message.angular.z = 0

        angular_speed = math.radians(abs(angular_speed_degree))

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
        desired_angle_radians = math.radians(abs(desired_angle_degree))

        relative_angle_degree = desired_angle_radians - self.yaw
        if relative_angle_degree < 0:
            clockwise = 1
        else:
            clockwise = 0

        self.rotate(30, math.degrees(abs(relative_angle_degree)), clockwise)


if __name__ == '__main__':
    type = int(sys.argv[1])

    rospy.init_node('turtlesim_rotate_motion')
    turtle_sim_motion = TurtleSimMotion()
    position_topic = '/turtle1/pose'
    pose_subscriber = rospy.Subscriber(position_topic, Pose,
                                        turtle_sim_motion.pose_callback)
    time.sleep(2)

    if type == 1:
        if len(sys.argv) == 5:
            angular_speed_degree = int(sys.argv[2])
            relative_angle_degree = int(sys.argv[3])
            clockwise = strtobool(sys.argv[4])

            turtle_sim_motion.rotate(angular_speed_degree,
                                     relative_angle_degree, clockwise)
        else:
            print("%s Type = %d Parameter error" % (sys.argv[0], type))
            sys.exit(1)
    elif type == 2:
        if len(sys.argv) == 3:
            desired_degree = int(sys.argv[2])

            turtle_sim_motion.set_desired_orientation(desired_degree)
        else:
            print("%s Type = %d Parameter error" % (sys.argv[0], type))
            sys.exit(1)
