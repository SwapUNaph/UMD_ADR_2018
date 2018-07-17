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

# how fast do scripts run
frequency = 5


# rotate vector by quaternion
def qv_mult(q1, v1):
    l = length(v1)
    v1 = tfs.unit_vector(v1)
    q2 = list(v1)
    q2.append(0.0)
    return tfs.quaternion_multiply(tfs.quaternion_multiply(q1, q2), tfs.quaternion_conjugate(q1))[:3] * l


def axang2quat(vector):
    l = length(vector)
    s = math.sin(l / 2)
    x = vector[0] / l * s
    y = vector[1] / l * s
    z = vector[2] / l * s
    w = math.cos(l / 2)
    return [x, y, z, w]


def length(list):
    return math.sqrt(list[0] * list[0] + list[1] * list[1] + list[2] * list[2])


def find_average(latest_gates):
    # transpose latest_gates
    count = len(latest_gates)

    pos = np.array([[0], [0], [0]])
    sin = 0
    cos = 0
    for gate in latest_gates:
        pos = pos + gate.pos
        sin = sin + math.sin(gate.hdg)
        cos = cos + math.cos(gate.hdg)

    pos = pos/count
    angle = math.atan2(sin, cos)

    return WP(pos, angle)


class WP:
    def __init__(self, pos, hdg):
        self.pos = np.array(pos)
        self.hdg = hdg

    def __str__(self):
        return str(self.pos) + " and " + str(self.hdg*180/math.pi)

class Gate_Detection_Info:
    def __init__(self, data):
        tvec = np.array(data.tvec)
        tvec.resize([3, 1])
        rvec = np.array(data.rvec)
        rvec.resize([3, 1])

        self.tvec = tvec
        self.rvec = rvec
        self.bebop_pose = data.bebop_pose

    def __str__(self):
        return "tvec " + str(self.tvec) + "\nrvec " + str(self.rvec) + "\n" + str(self.bebop_pose)