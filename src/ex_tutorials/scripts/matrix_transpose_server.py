#!/usr/bin/env python

import rospy
import numpy as np

from hello_tutorials.srv import Matrix, MatrixResponse


def handle_matrix_transpose(req):
    rospy.loginfo("Matrix Size = [%s, %s]", req.height, req.width)
    matrix = np.reshape(req.data, [req.height, req.width])
    data = matrix.T
    return MatrixResponse(data.flatten())


if __name__ == "__main__":
    rospy.init_node('matrix_transpose_server')
    s = rospy.Service('matrix_transpose', Matrix, handle_matrix_transpose)
    print("Ready to matrix transpose.")
    rospy.spin()