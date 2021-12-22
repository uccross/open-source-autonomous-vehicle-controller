"""
.. module:: State.py
	:platform: MacOS, Unix, Windows,
	:synopsis: State class
.. moduleauthor:: Pavlo Vlastos <pvlastos@ucsc.edu>
"""

import numpy as np
import copy

class PointMassState():
    def __init__(self, state_vector):
        self.x = state_vector[0][0]
        self.y = state_vector[1][0]
        self.z = state_vector[2][0]

        self.vx = state_vector[3][0]
        self.vy = state_vector[4][0]
        self.vz = state_vector[5][0]

        # Point mass state vector
        self.vec = np.array([[self.x],
                             [self.y],
                             [self.z],
                             [self.vx],
                             [self.vy],
                             [self.vz]])
        self.m, self.n = self.vec.shape

        return

    def get_state_vector(self):
        return self.vec

    def set_state_vector(self, state_vector):
        m, n = state_vector.shape

        if (self.m == m) and (self.n == n):
            self.vec = copy.deepcopy(state_vector)

            self.x = self.vec[0][0]
            self.y = self.vec[1][0]
            self.z = self.vec[2][0]

            self.vx = self.vec[3][0]
            self.vy = self.vec[4][0]
            self.vz = self.vec[5][0]
        else:
            print("State error: Point Mass!")

    def get_velocity(self):
        velocity = np.zeros((3,1))
        velocity[0][0] = self.vx
        velocity[1][0] = self.vy
        velocity[2][0] = self.vz
        return velocity

    def __repr__(self):
        return "{1.__name__}(x={0.x}, y={0.y}, z={0.z}, vx={0.vx}, vy={0.vy}, vz={0.vz}".format(self, type(self))

class OrientationState():
    def __init__(self, state_vector):
        self.roll = state_vector[0][0]
        self.pitch = state_vector[1][0]
        self.yaw = state_vector[2][0]

        self.p = state_vector[3][0]
        self.q = state_vector[4][0]
        self.r = state_vector[5][0]

        self.vec = np.array([[self.roll],
                             [self.pitch],
                             [self.yaw],
                             [self.p],
                             [self.q],
                             [self.r]])
                             
        self.m, self.n = self.vec.shape
        
    def get_state_vector(self):
        return self.vec
    
    def set_state_vector(self, state_vector):
        m, n = state_vector.shape

        if (self.m == m) and (self.n == n):
            self.vec = copy.deepcopy(state_vector)

            self.roll = self.vec[0][0]
            self.pitch = self.vec[1][0]
            self.yaw = self.vec[2][0]

            self.p = self.vec[3][0]
            self.q = self.vec[4][0]
            self.r = self.vec[5][0]
        else:
            print("State error: Orientation!")

    def __repr__(self):
        return "{1.__name__}(roll={0.roll}, pitch={0.pitch}, yaw={0.yaw}, p={0.p}, q={0.q}, r={0.r})".format(self, type(self))

