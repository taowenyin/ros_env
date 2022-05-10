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

    def move(self, speed, distance, is_forward):
        velocity_message = Twist()
        x_0, y_0 = self.x, self.y

        if is_forward:
            velocity_message.linear.x = abs(speed)
        else:
            velocity_message.linear.x = -1 * abs(speed)

        distance_move = 0.0
        loop_rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            rospy.loginfo('Turtlesim moves forwards')
            self.velocity_publisher.publish(velocity_message)

            loop_rate.sleep()

            distance_move += abs(0.5 * math.sqrt(
                (self.x - x_0) ** 2 + (self.y - y_0) ** 2))
            rospy.loginfo('Distance Move = %f', distance_move)

            if distance_move > distance:
                rospy.loginfo('Reached')
                break

        velocity_message.linear.x = 0
        self.velocity_publisher.publish(velocity_message)


if __name__ == '__main__':
    if len(sys.argv) > 4:
        speed = int(sys.argv[1])
        distance = int(sys.argv[2])
        is_forward = strtobool(sys.argv[3])
    else:
        print("%s [speed distance is_forward]" % sys.argv[0])
        sys.exit(1)

    rospy.init_node('turtlesim_linear_motion', anonymous=True)

    turtle_sim_motion = TurtleSimMotion()
    pose_subscriber = rospy.Subscriber('/turtle1/pose', Pose,
                                       turtle_sim_motion.pose_callback)
    time.sleep(2)
    turtle_sim_motion.move(speed, distance, is_forward)
