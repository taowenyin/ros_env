#!/usr/bin/env python

import rospy
import random

from std_msgs.msg import Int32

def talker():
	pub = rospy.Publisher('random_number', Int32, queue_size=10)
	rospy.init_node('random_number', anonymous=True)
	rate = rospy.Rate(2) # 10hz
	while not rospy.is_shutdown():
		random_data = random.randint(10, 20)
		rospy.loginfo('random_data = %d', random_data)
		pub.publish(random_data)
		rate.sleep()

if __name__ == '__main__':
	try:
		talker()
	except rospy.ROSInterruptException:
		pass
