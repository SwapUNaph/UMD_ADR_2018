#!/usr/bin/env python

#  --- Changelog ---
# Goal:     Collect information that are needed at multiple locations


import math
from tf import transformations as tfs
import numpy as np


# vector and rotation from bebop to zed
BZ = np.array([[0], [0], [0]])
zRb = tfs.euler_matrix(-math.pi / 2, 0, -math.pi / 2, 'rzyx')
cam_q = tfs.quaternion_from_matrix(zRb)
zRb = zRb[:3, :3]


# rotate vector by quaternion
def qv_mult(q1, v1):
    length = math.sqrt(v1[0]*v1[0]+v1[1]*v1[1]+v1[2]*v1[2])
    v1 = tfs.unit_vector(v1)
    q2 = list(v1)
    q2.append(0.0)
    return tfs.quaternion_multiply(tfs.quaternion_multiply(q1, q2), tfs.quaternion_conjugate(q1))[:3] * length


class WP:
    def __init__(self, pos, hdg):
        self.x = pos[0]
        self.y = pos[1]
        self.z = pos[2]
        self.hdg = hdg