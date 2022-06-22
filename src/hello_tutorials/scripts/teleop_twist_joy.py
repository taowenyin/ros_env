#!/usr/bin/env python

import rospy

from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

VEL_UP_KEY = 5
VEL_DOWN_KEY = 4
VEL_FORWARD_BACK_KEY = 5
ANG_UP_KEY = 7
ANG_DOWN_KEY = 6
ANG_FORWARD_BACK_KEY = 4
STOP_KEY = 2
RESET_KEY = 1


def handle_twist_joy(msg, twist_joy):
    jox_axes = msg.axes
    jox_buttons = msg.buttons

    if jox_buttons[VEL_UP_KEY] == 1:
       twist_joy.linear_speed_up()
    if jox_buttons[VEL_DOWN_KEY] == 1:
       twist_joy.linear_speed_down()
    if jox_buttons[ANG_UP_KEY] == 1:
       twist_joy.angular_speed_up()
    if jox_buttons[ANG_DOWN_KEY] == 1:
       twist_joy.angular_speed_down()
    if jox_buttons[STOP_KEY] == 1:
       twist_joy.stop()
    if jox_buttons[RESET_KEY] == 1:
       twist_joy.reset()
    if jox_axes[VEL_FORWARD_BACK_KEY] == 1.0:
        twist_joy.set_direction(1)
    if jox_axes[VEL_FORWARD_BACK_KEY] == -1.0:
        twist_joy.set_direction(-1)
    if jox_axes[ANG_FORWARD_BACK_KEY] == 1.0:
        twist_joy.set_angle(1)
    if jox_axes[ANG_FORWARD_BACK_KEY] == -1.0:
        twist_joy.set_angle(-1)


class TwistJoy(object):
    def __init__(self):
        self.vel_linear = 0.0
        self.vel_angular = 0.0
        self.def_vel_linear = 0.15
        self.def_vel_angular = 0.25

        self.robot_vel = rospy.Publisher('/cmd_vel', Twist,
                                         queue_size=10)

    def set_vel(self):
        cmd = Twist()
        cmd.linear.x = self.vel_linear
        cmd.angular.z = self.vel_angular

        self.robot_vel.publish(cmd)

    def set_direction(self, forward_back):
        if forward_back == 1:
            if self.vel_linear == 0.0:
                self.vel_linear = abs(self.def_vel_linear)
            else:
                self.vel_linear = abs(self.vel_linear)
        if forward_back == -1:
            if self.vel_linear == 0.0:
                self.vel_linear = -1 * abs(self.def_vel_linear)
            else:
                self.vel_linear = -1 * abs(self.vel_linear)

        self.vel_angular = 0.0

    def set_angle(self, forward_back):
        if forward_back == 1:
            if self.vel_angular == 0.0:
                self.vel_angular = abs(self.def_vel_angular)
            else:
                self.vel_angular = abs(self.vel_angular)
        if forward_back == -1:
            if self.vel_angular == 0.0:
                self.vel_angular = -1 * abs(self.def_vel_angular)
            else:
                self.vel_angular = -1 * abs(self.vel_angular)

        self.vel_linear = 0

    def linear_speed_up(self):
        self.vel_linear += (self.vel_linear * 0.1)

    def linear_speed_down(self):
        self.vel_linear -= (self.vel_linear * 0.1)

    def angular_speed_up(self):
        self.vel_angular += (self.vel_angular * 0.1)

    def angular_speed_down(self):
        self.vel_angular -= (self.vel_angular * 0.1)

    def stop(self):
        self.vel_linear = 0
        self.vel_angular = 0

    def reset(self):
        self.vel_linear = self.def_vel_linear
        self.vel_angular = self.def_vel_angular


if __name__ == '__main__':
    rospy.init_node('teleop_twist_joy')

    rate = rospy.Rate(10)
    twist_joy = TwistJoy()

    rospy.Subscriber('/joy', Joy, handle_twist_joy, twist_joy)

    while not rospy.is_shutdown():
        twist_joy.set_vel()
        rate.sleep()
