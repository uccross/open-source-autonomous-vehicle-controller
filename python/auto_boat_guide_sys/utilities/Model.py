"""
.. module:: Model.py
    :platform: MacOS, Unix, Windows,
    :synopsis: Model for vehicle or other object
.. moduleauthor:: Pavlo Vlastos <pvlastos@ucsc.edu>
"""

import numpy as np
from scipy.linalg import expm

from .utilities import Constants as CNST
from .utilities import Quaternions as QU


class PointMassDynamics():
    def __init__(self, dt=CNST.dt_sim, mass=CNST.mass):

        self.dt = dt

        self.mass = mass

        #######################################################################
        # continous state space (gets discretized below)
        self.A = np.array([
            [0.0, 0.0, 0.0, 1.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        ])

        self.B = np.array([
            [0.0, 0.0, 0.0],  # to x dot
            [0.0, 0.0, 0.0],  # to y dot
            [0.0, 0.0, 0.0],  # to z dot
            [1.0/self.mass, 0.0, 0.0],  # to vx dot
            [0.0, 1.0/self.mass, 0.0],  # to vy dot
            [0.0, 0.0, 1.0/self.mass]  # to vz dot
        ])
        # print("self.A.shape: {}".format(self.A.shape))
        # print("self.B.shape: {}".format(self.B.shape))

        AB = np.concatenate((self.A, self.B), axis=1)
        # print("AB.shape: {}".format(AB.shape))

        m, n = AB.shape
        Z = np.zeros((n-m, n))
        # print("Z.shape: {}".format(Z.shape))
        AB_Z = np.concatenate((AB, Z), axis=0)
        # print("AB_Z:\r\n{}".format(np.round(AB_Z,4)))
        # print("AB_Z.shape: {}".format(AB_Z.shape))

        self.PhiGamma_ZI = expm(AB_Z*self.dt)
        # print("self.PhiGamma_ZI:\r\n{}".format(np.round(self.PhiGamma_ZI,4)))
        # print("self.PhiGamma_ZI.shape: {}".format(self.PhiGamma_ZI.shape))

        _, An = self.A.shape

        #######################################################################
        # Discrete State Space
        self.Phi = self.PhiGamma_ZI[0:m, 0:An]
        self.Gamma = self.PhiGamma_ZI[0:m, An:]
        # print("self.Phi:\r\n{}".format(np.round(self.Phi,4)))
        # print("self.Phi.shape: {}".format(self.Phi.shape))
        # print("self.Gamma:\r\n{}".format(np.round(self.Gamma,4)))
        # print("self.Gamma.shape: {}".format(self.Gamma.shape))

        return

    def update(self, x, u):
        """
        :param x: State vector
        :param u: Net input thrust force in body reference frame including 
        disturbances
        :param F_d: Disturbance force
        """
        return (self.Phi @ x) + (self.Gamma @ u)

    def get_Phi(self):
        return self.Phi

    def get_Gamma(self):
        return self.Gamma


class SolidSphere():
    """
    A sphere with a rudder at the end, which can only actuate for *desired* 
    rotation about the z-body axis. The sphere can still be disturbed to rotate 
    about any and all axes!
    """
    def __init__(self, dt=CNST.dt_sim, mass=CNST.mass,
                 radius=CNST.sphere_radius):
        self.dt = dt

        self.mass = mass

        self.radius = radius

        # Assume solid sphere for moment of inertia for now
        self.moment_of_inertia = (2.0/5.0)*self.mass*(self.radius**2.0)

        self.b = 1.0/self.moment_of_inertia

        self.q = QU.setQuaternionEulerAngles(0.0, 0.0, 0.0)

        #######################################################################
        # continous state space (gets discretized below)
        self.A = np.array([
            [0.0, 0.0, 0.0, 1.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        ])

        self.B = np.array([
            [0.0, 0.0, 0.0],  # to wx dot
            [0.0, 0.0, 0.0],  # to wy dot
            [0.0, 0.0, 0.0],  # to wz dot
            [self.b, 0.0, 0.0],
            [0.0, self.b, 0.0],
            [0.0, 0.0, self.b]
        ])

        AB = np.concatenate((self.A, self.B), axis=1)

        m, n = AB.shape
        Z = np.zeros((n-m, n))

        AB_Z = np.concatenate((AB, Z), axis=0)

        self.PhiGamma_ZI = expm(AB_Z*self.dt)

        _, An = self.A.shape

        #######################################################################
        # Discrete State Space
        self.Phi = self.PhiGamma_ZI[0:m, 0:An]
        self.Gamma = self.PhiGamma_ZI[0:m, An:]

        return

    def update(self, x, u):
        """
        :param x: State vector
        :param u: Net input torque based on thrust and surface drag torque in 
        body reference frame. May include other torque disturbances in the 
        future
        """

        # x = (self.Phi @ x) + (self.Gamma @ u)

        # instantaneous angular rates
        x[3][0] = self.b*u[0][0]*self.dt 
        x[4][0] = self.b*u[1][0]*self.dt  
        x[5][0] = self.b*u[2][0]*self.dt 

        p = np.array([
            [0.0],
            [x[3][0]],
            [x[4][0]],
            [x[5][0]]
        ])

        q_dot = 0.5*QU.multiplyQuaternions(self.q, p)

        self.q = self.q + q_dot*self.dt

        q_mag = np.linalg.norm(self.q)

        self.q /= q_mag

        x[0][0], x[1][0], x[2][0] = QU.quaternionToEulerAngles(self.q)
        
        return x

    def get_Phi(self):
        return self.Phi

    def get_Gamma(self):
        return self.Gamma

    def get_radius(self):
        return self.radius
