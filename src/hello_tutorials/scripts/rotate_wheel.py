#!/usr/bin/env python
import math

import rospy
import time
import threading

from sensor_msgs.msg import JointState


class RotateWheelNode:
    def __init__(self):
        self.joint_states_publisher = rospy.Publisher('joint_states',
                                                      JointState, queue_size=10)
        self._init_joint_state()
        self.rate = rospy.Rate(10)

        self.spin_thread = threading.Thread(target=self._thread_pub)
        self.spin_thread.start()

    def _init_joint_state(self):
        self.joint_speeds = [0.0, 0.0]

        self.joint_state = JointState()
        self.joint_state.header.stamp = rospy.Time.now()
        self.joint_state.header.frame_id = ''

        self.joint_state.name = ['base_l_wheel_joint', 'base_r_wheel_joint']
        self.joint_state.position = [0.0, 0.0]

        self.joint_state.velocity = self.joint_speeds
        self.joint_state.effort = []

    def _thread_pub(self):
        last_update_time = time.time()
        while not rospy.is_shutdown():
            delta_time = time.time() - last_update_time
            last_update_time = time.time()

            self.joint_state.position[0] += \
                (delta_time * abs(self.joint_state.velocity[0]) % math.pi)
            self.joint_state.position[1] += \
                (-1 * (delta_time * abs(self.joint_state.velocity[1]) % math.pi))

            self.joint_state.velocity = self.joint_speeds
            self.joint_state.header.stamp = rospy.Time.now()

            try:
                self.joint_states_publisher.publish(self.joint_state)
            except rospy.ROSException as e:
                rospy.loginfo("Service call failed: %s", e)

    def update_speed(self, speeds):
        self.joint_speeds = speeds


if __name__ == '__main__':
    rospy.init_node('rotate_wheel', anonymous=True)

    rotate_wheel_node = RotateWheelNode()
    rotate_wheel_node.update_speed([15.0, 15.0])

    while not rospy.is_shutdown():
        continue

    rotate_wheel_node.spin_thread.join()
    rospy.signal_shutdown('')
