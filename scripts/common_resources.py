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
    x = vector[0][0] / l * s
    y = vector[1][0] / l * s
    z = vector[2][0] / l * s
    w = math.cos(l / 2)
    return [x, y, z, w]


def length(list):
    return math.sqrt(list[0] * list[0] + list[1] * list[1] + list[2] * list[2])


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


class WP:
    def __init__(self, pos, hdg):
        self.pos = np.array(pos)
        self.hdg = hdg

    def __str__(self):
        return str(list(self.pos) + [self.hdg])


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
        return "\ntvec " + str(self.tvec) + "\nrvec " + str(self.rvec) + "\n" + str(self.bebop_pose)


class PID:
    def __init__(self, p=2.0, i=0.0, d=1.0, derivator=0, integrator=0, integrator_max=.5, integrator_min=-.5):
        self.kp = p
        self.ki = i
        self.kd = d
        self.derivator = derivator
        self.integrator = integrator
        self.integrator_max = integrator_max
        self.integrator_min = integrator_min
        self.error = 0.0

        self.p_value = None
        self.i_value = None
        self.d_value = None
        self.set_point = None

        


    def update(self, err):
        self.error = err

        self.p_value = self.kp * self.error
        self.d_value = self.kd * (self.error - self.derivator)
        self.derivator = self.error

        self.integrator = self.integrator + self.error

        if self.integrator > self.integrator_max:
            self.integrator = self.integrator_max
        elif self.integrator < self.integrator_min:
            self.integrator = self.integrator_min

        self.i_value = self.integrator * self.ki

        return [self.p_value, self.i_value, self.d_value]

    def reset(self, set_point):
        self.set_point = set_point
        self.integrator = 0
        self.derivator = 0


class PID2:
    def __init__(self, p=2.0, i=0.0, d=1.0, derivator=0, integrator=0, integrator_max=.5, integrator_min=-.5):
        self.kp = p
        self.ki = i
        self.kd = d
        self.derivator = derivator
        self.derivator2 = derivator
        self.integrator = integrator
        self.integrator_max = integrator_max
        self.integrator_min = integrator_min
        self.error = 0.0

        self.p_value = None
        self.i_value = None
        self.d_value = None
        self.set_point = None

        self.dt = .2
        self.A = np.array([1.0 dt],[0 1])
        self.B = np.array([dt**2/2],[dt])
        self.C = np.array([1.0 0])

        self.Q_estimate = np.array([0 0]) # x_estimate of initial location estimation of where the Quail is (what we are updating)

        process_noise = 8 # process noise: the variability in how fast the Quail is speeding up (stdv of acceleration: meters/sec^2)
        measurement_noise = 1.5 # measurement noise: How mask-blinded is the Ninja (stdv of location, in meters)

        self.Ez = measurement_noise**2; # Ez convert the measurement noise (stdv) into covariance matrix
        self.Ex = process_noise**2 * np.array([process_noise**2*dt**4/4,process_noise**2 *dt**3/2],[process_noise**2 *dt**3/2,process_noise**2 *dt**2]) # Ex convert the process noise (stdv) into covariance matrix

        self.P = self.Ex #estimate of initial Quail position variance (covariance matrix)


    def update(self, err):
        self.Q_estimate = self.A * self.Q_estimate

        P = self.A * self.P * np.transpose(self.A) + self.Ex

        # Kalman Gain
        K = self.P*np.transpose(self.C)*inv(self.C*self.P*np.transpose(self.C)+self.Ez)
        
        # Update the state estimate.
        self.Q_estimate = self.Q_estimate + K * (err - self.C * self.Q_estimate)
        
        # update covariance estimation.
        self.P =  (np.eye(2)-K*self.C)*self.P

        self.error = err

        self.p_value = self.kp * self.error
        self.d_value = self.kd * ((self.error - self.derivator)*.65 + (self.error - self.derivator2)*.35)
        self.derivator = self.error
        self.derivator2 = self.derivator

        self.integrator = self.integrator + self.error

        if self.integrator > self.integrator_max:
            self.integrator = self.integrator_max
        elif self.integrator < self.integrator_min:
            self.integrator = self.integrator_min

        self.i_value = self.integrator * self.ki

        return [self.p_value, self.i_value, self.d_value]

    def reset(self, set_point):
        self.set_point = set_point
        self.integrator = 0
        self.derivator = 0
