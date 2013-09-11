# D-H transformation matrix
# derived from the product of four basic homogeneous transformation matrices.
# Note that PUMA 560 robot is consist of 6 rotation joints.

from numpy  import *
from math import *
from config_IK import *


def DH(theta, alpha, a, d):
    A = array([[cos(theta), -cos(alpha)*sin(theta), sin(alpha)*sin(theta), a*cos(theta)],
                [sin(theta), cos(alpha)*cos(theta), -sin(alpha)*cos(theta), a*sin(theta)],
                [0, sin(alpha), cos(alpha), d],
                [0, 0, 0, 1]])
    return A
