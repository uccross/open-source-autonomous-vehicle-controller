"""
.. module:: Model.py
    :platform: MacOS, Unix, Windows,
    :synopsis: Model for vehicle or other object
.. moduleauthor:: Pavlo Vlastos <pvlastos@ucsc.edu>
"""

import numpy as np
from scipy.linalg import expm

from ..Constants import Constants as CNST
from ..Utilities import Quaternions as QU


class PointMassDynamics():
    def __init__(self, dt=CNST.dt_sim, mass=CNST.mass):

        self.dt_sim = dt

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

        self.PhiGamma_ZI = expm(AB_Z*self.dt_sim)
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

    def get_A(self):
        return self.A

    def get_B(self):
        return self.B

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
        self.dt_sim = dt

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

        self.PhiGamma_ZI = expm(AB_Z*self.dt_sim)

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
        x[3][0] = self.b*u[0][0]*self.dt_sim
        x[4][0] = self.b*u[1][0]*self.dt_sim
        x[5][0] = self.b*u[2][0]*self.dt_sim

        p = np.array([
            [0.0],
            [x[3][0]],
            [x[4][0]],
            [x[5][0]]
        ])

        q_dot = 0.5*QU.multiplyQuaternions(self.q, p)

        self.q = self.q + q_dot*self.dt_sim

        q_mag = np.linalg.norm(self.q)

        self.q /= q_mag

        x[0][0], x[1][0], x[2][0] = QU.quaternionToEulerAngles(self.q)

        return x

    def get_A(self):
        return self.A

    def get_B(self):
        return self.B

    def get_Phi(self):
        return self.Phi

    def get_Gamma(self):
        return self.Gamma

    def get_radius(self):
        return self.radius


class orientationModel():
    """
    An orientation model with a rudder at the end, which can only actuate for 
    *desired* rotation about the z-body axis. 
    """

    def __init__(self, dt=CNST.dt_sim, mass=CNST.mass,
                 radius=CNST.sphere_radius):
        self.dt_sim = dt

        self.mass = mass

        self.radius = radius

        # Assume solid sphere for moment of inertia for now
        self.moment_of_inertia = (2.0/5.0)*self.mass*(self.radius**2.0)

        self.b = 1.0/self.moment_of_inertia

        self.q = QU.setQuaternionEulerAngles(0.0, 0.0, 0.0)

        #######################################################################
        # continous state space (gets discretized below)
        self.A = np.array([
            [0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0]
        ])

        self.B = np.array([
            [self.b, 0.0, 0.0],
            [0.0, self.b, 0.0],
            [0.0, 0.0, self.b]
        ])

        AB = np.concatenate((self.A, self.B), axis=1)

        m, n = AB.shape
        Z = np.zeros((n-m, n))

        AB_Z = np.concatenate((AB, Z), axis=0)

        self.PhiGamma_ZI = expm(AB_Z*self.dt_sim)

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
        x[3][0] = self.b*u[0][0]*self.dt_sim
        x[4][0] = self.b*u[1][0]*self.dt_sim
        x[5][0] = self.b*u[2][0]*self.dt_sim

        p = np.array([
            [0.0],
            [x[3][0]],
            [x[4][0]],
            [x[5][0]]
        ])

        q_dot = 0.5*QU.multiplyQuaternions(self.q, p)

        self.q = self.q + q_dot*self.dt_sim # TODO: USE THE MATHEMATICALLY CORRECT WAY

        q_mag = np.linalg.norm(self.q)

        self.q /= q_mag

        x[0][0], x[1][0], x[2][0] = QU.quaternionToEulerAngles(self.q)

        return x

    def get_A(self):
        return self.A

    def get_B(self):
        return self.B

    def get_Phi(self):
        return self.Phi

    def get_Gamma(self):
        return self.Gamma

    def get_radius(self):
        return self.radius

class AugmentedNomotoIdeal():
    def __init__(self, dt_sim=CNST.dt_sim, tYaw=1.0, kYaw=300.0, yaw_0=0.0,
                 maxRudAngle=CNST.max_rudder_angle,
                 minRudAngle=CNST.min_rudder_angle):

        self.dt_sim = dt_sim

        self.tYaw = tYaw

        self.kYaw = kYaw

        self.maxRudAngle = maxRudAngle
        self.minRudAngle = minRudAngle

        # Discrete State Transition Matrix AKA the Dynamics Matrix
        self.Phi = np.array([
            [1.0, self.dt_sim, 0.0, 0.0, 0.0],
            [0.0, -1.0/self.tYaw, 0.0, 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.0, self.dt_sim*np.sin(yaw_0)],
            [0.0, 0.0, 0.0, 1.0, self.dt_sim*np.cos(yaw_0)],
            [0.0, 0.0, 0.0, 0.0, 1.0]
        ])

        # Input matrix
        self.Gamma = np.array([[0.0],
                               [-self.kYaw/self.tYaw],
                               [0.0],
                               [0.0],
                               [0.0]])

        # Output or observation matrix
        self.H = np.array([[1.0, 0.0, 0.0, 0.0, 0.0],
                           [0.0, 1.0, 0.0, 0.0, 0.0],
                           [0.0, 0.0, 1.0, 0.0, 0.0],
                           [0.0, 0.0, 0.0, 1.0, 0.0],
                           [0.0, 0.0, 0.0, 0.0, 1.0]])

        # Measurement vector
        self.y = np.array([[0.0],
                           [0.0],
                           [0.0],
                           [0.0],
                           [0.0]])

        return

    def update(self, x, u):
        """
        :param x: State vector
        :param u: Input rudder angle (radians)
        :param F_d: Disturbance force
        """

        # Apply physical constraints by limiting the rudder angle
        if (u > self.maxRudAngle):
            u = self.maxRudAngle

        elif (u < self.minRudAngle):
            u = self.minRudAngle

        yaw = x[0][0]

        # Update the non-linear discrete state-space transition matrix
        self.Phi = np.array([
            [1.0, self.dt_sim, 0.0, 0.0, 0.0],
            [0.0, -1.0/self.tYaw, 0.0, 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.0, self.dt_sim*np.sin(yaw)],
            [0.0, 0.0, 0.0, 1.0, self.dt_sim*np.cos(yaw)],
            [0.0, 0.0, 0.0, 0.0, 1.0]
        ])

        x = (self.Phi @ x) + (self.Gamma * u)

        # Make sure the heading angle is wrapped to [-pi, pi]
        x[0][0] = (np.mod((x[0][0] + np.pi), (2.0 * np.pi)) - np.pi)

        return x

    def get_Phi(self):
        return self.Phi

    def get_Gamma(self):
        return self.Gamma

    def set_tYaw(self, tYaw_new):
        self.tYaw = tYaw_new
        
    def set_kYaw(self, kYaw_new):
        self.kYaw = kYaw_new
