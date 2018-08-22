#!/usr/bin/env python

#  --- Changelog ---
# Goal:     Collect information that are needed at multiple locations


import math
from tf import transformations as tfs
import numpy as np


# vector and rotation from bebop to zed
BZ = np.array([0, 0, 0])
zRb = tfs.euler_matrix(-math.pi / 2, 0, -math.pi / 2, 'rzyx')
cam_q = tfs.quaternion_from_matrix(zRb)
zRb = zRb[:3, :3]

# how fast do scripts run
frequency = 5


# rotate vector by quaternion
def qv_mult(q1, v1):
    l = np.linalg.norm(v1)
    v1 = tfs.unit_vector(v1)
    q2 = np.append(v1, 0.0)
    return tfs.quaternion_multiply(tfs.quaternion_multiply(q1, q2), tfs.quaternion_conjugate(q1))[:3] * l


def axang2quat(vector):
    l = np.linalg.norm(vector)
    s = math.sin(l / 2)
    x = vector[0] / l * s
    y = vector[1] / l * s
    z = vector[2] / l * s
    w = math.cos(l / 2)
    return np.array([x, y, z, w])


def find_average(latest_gates):
    count = len(latest_gates)

    pos = np.array([0, 0, 0])
    sin = 0
    cos = 0
    for gate in latest_gates:
        pos = pos + gate.pos
        sin = sin + math.sin(gate.hdg)
        cos = cos + math.cos(gate.hdg)

    pos = pos/count
    angle = math.atan2(sin, cos)

    return WP(pos, angle)


def find_std_dev_waypoints(average, wp_input_history):
    deviation = 0
    for gate in wp_input_history:
        deviation = deviation + (np.linalg.norm(average.pos - gate.pos))**2
    deviation = math.sqrt(deviation/len(wp_input_history))
    return deviation


def min_value(value, minimum):
    if -minimum < value < 0:
        return -minimum
    elif 0 < value < minimum:
        return minimum
    else:
        return value


def limit_value(value, limit):
    if value > limit:
        return limit
    elif value < -limit:
        return -limit
    else:
        return value


def calculate_periods(input_data):
    angles = np.unwrap(input_data[1, :])
    times = input_data[0, :]
    d_a = np.diff(angles)
    d_t = np.diff(times)
    return 2*math.pi * d_t / d_a


class WP:
    def __init__(self, pos, hdg):
        self.pos = np.array(pos)
        self.hdg = hdg

    def __str__(self):
        return str(list(self.pos) + [self.hdg])
#
#
# class Gate_Detection_Info:
#     def __init__(self, data):
#         tvec = np.array(data.tvec)
#         tvec.resize([3, 1])
#         rvec = np.array(data.rvec)
#         rvec.resize([3, 1])
#
#         self.tvec = tvec
#         self.rvec = rvec
#         self.bebop_pose = data.bebop_pose
#
#     def __str__(self):
#         return "\ntvec " + str(self.tvec) + "\nrvec " + str(self.rvec) + "\n" + str(self.bebop_pose)
#


class DynamicData:
    def __init__(self):
        self.state = 0
        self.timer = 0.0
        self.period = None
        self.theta = None
        self.time_taken_to_gate = 1.3
        self.throttle_counter = 0

    def theta_trigger(self):
        rotations = self.time_taken_to_gate/self.period
        theta = -(2*math.pi*rotations+(2*math.pi/(5*self.period)))
        return theta % 2*math.pi


class PID2:
    def __init__(self, p=2.0, i=0.0, d=1.0, derivator=[0.0, 0.0, 0.0, 0.0], integrator=0, integrator_max=.5, integrator_min=-.5):
        self.kp = p
        self.ki = i
        self.kd = d
        self.derivator = derivator
        self.integrator = integrator
        self.integrator_max = integrator_max
        self.integrator_min = integrator_min

        self.p_value = None
        self.i_value = None
        self.d_value = 0.0    

    def update(self, err):
        self.d_value = self.kd*((4*err - sum(self.derivator))/10)

        self.derivator.pop(0)
        self.derivator.append(err)      

        self.p_value = self.kp * err
        self.integrator = self.integrator + err

        if self.integrator > self.integrator_max:
            self.integrator = self.integrator_max
        elif self.integrator < self.integrator_min:
            self.integrator = self.integrator_min

        self.i_value = self.integrator * self.ki

        return [self.p_value, self.i_value, self.d_value]

    def reset(self):
        self.integrator = 0
        self.derivator = 0
