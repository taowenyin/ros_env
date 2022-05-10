#!/usr/bin/env python

import rospy

if __name__ == '__main__':
	rospy.init_node('namespace_param', anonymous=True)

	rospy.set_param('a_string', 'baz')
	rospy.set_param('~private_int', 2)
	rospy.set_param('list_of_floats', [1., 2., 3., 4.])
	rospy.set_param('bool_True', True)
	rospy.set_param('gains', {'p': 1, 'i': 2, 'd': 3})

	if rospy.has_param('to_delete'):
		print("to_delete param existence")
		rospy.delete_param('to_delete')
	else:
		print("not existence to_delete param")

	try:
		param_list = rospy.get_param_names()
		print("print all param")
		print(param_list)
	except rospy.ROSException:
		print("could not get param name")
