#!/usr/bin/env python

import rospy
import numpy as np

from hello_tutorials.srv import *


if __name__ == "__main__":
    rospy.wait_for_service('matrix_transpose')
    try:
        matrix = np.array([[1, 2, 3], [4, 5, 6], [7, 8, 9], [10, 11, 12]])
        width = 3
        height = 4

        data = matrix.flatten()
        matrix_transpose = rospy.ServiceProxy('matrix_transpose', Matrix)
        resp = matrix_transpose(data, width, height)

        print(np.reshape(resp.data, [width, height]))
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)

