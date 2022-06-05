"""
.. module:: StateEstimator.py
    :platform: MacOS, Unix, Windows,
    :synopsis: Extended Kalman Filter for state estimation
.. moduleauthor:: Pavlo Vlastos <pvlastos@ucsc.edu>
"""

import numpy as np

from ..Containers import State
from ..Constants import Constants


class NomotoStateEstimator():
    def __init__(self, dt=Constants.dt_uc, X_0=np.zeros((6, 1)), sigma_r=0.2,
                 sigma_v=0.000001, sigma_psi=0.01, vrpx=0.1, vrpy=0.1,
                 vrpsi=0.1, vrr=0.1, vrv=0.1):

        m, _ = X_0.shape

        self.dt = dt

        self.F = np.zeros((m, 1))

        self.Xhm = X_0
        self.Xh = X_0

        self.Phih = np.zeros((m, m))
        self.Gammah = np.zeros((m, 2))
        # self.Gammah[0][0] = sigma_psi
        self.Gammah[0][0] = sigma_r
        self.Gammah[4][1] = sigma_v
        self.Gammah[5][1] = 0.0

        self.Pm = np.eye(m)

        self.Pm = np.eye(m)
        self.P = np.eye(m)
        self.P[0][0] = vrpsi
        self.P[1][1] = vrr
        self.P[2][2] = vrpx
        self.P[3][3] = vrpy
        self.P[4][4] = vrv
        self.P[5][5] = 0.01

        self.vrv = vrv

        self.Q = np.eye(2)
        self.R = np.eye(2)

        # self.R[0][0] = vrpsi
        self.R[0][0] = vrpx
        self.R[1][1] = vrpy

        # self.h = np.array([[1.0, 0.0, 0.0, 0.0, 0.0],
        #                    [0.0, 0.0, 1.0, 0.0, 0.0],
        #                    [0.0, 0.0, 0.0, 1.0, 0.0]])
        # self.H = np.array([[1.0, 0.0, 0.0, 0.0, 0.0],
        #                    [0.0, 0.0, 1.0, 0.0, 0.0],
        #                    [0.0, 0.0, 0.0, 1.0, 0.0]])
        self.h = np.array([[0.0, 0.0, 1.0, 0.0, 0.0, 0.0],
                           [0.0, 0.0, 0.0, 1.0, 0.0, 0.0]])
        self.H = np.array([[0.0, 0.0, 1.0, 0.0, 0.0, 0.0],
                           [0.0, 0.0, 0.0, 1.0, 0.0, 0.0]])

        self.K = np.zeros((m, 1))

        self.k_yaw = 0.5  # for now assume the input gain is 1.0
        self.k_r = 1.0  # for now assume the input gain is 1.0

        self.t_yaw = 10.0
        self.t_yaw_h = 5.0

        self.I = np.eye(m)

        return

    def run(self, y=np.zeros((2, 1)), u=0.0):
        """
        :param y: The sensor measurements as a 2 by 1 vector, including East
        position (meters), North position (meters), @TODO: and heading angle 
        :param u: The commanded rudder angle signal between a bound such as
        -0.8 and 0.8. This is used in system identification
        (rardians)
        """

        # psihatm1 = self.Xhm[0][0]
        # rm1 = self.Xhm[1][0]
        # vhatm1 = self.Xhm[4][0]

        psihat = self.Xh[0][0]
        r = self.Xh[1][0]
        vhat = self.Xh[4][0]

        t_yaw_h = self.Xh[5][0]

        sin_psi_dt = np.sin(psihat)*self.dt
        cos_psi_dt = np.cos(psihat)*self.dt

        v_sin_psi_dt = vhat*sin_psi_dt
        v_cos_psi_dt = vhat*cos_psi_dt

        ####
        # Prediction step
        # Linearization for the prediction step
        self.F[0][0] = r*self.dt  # + np.random.normal(0.0, self.vrv, 1)
        # self.F[1][0] = (-((psihat - psihatm1)/self.dt))/self.t_yaw
        # self.F[1][0] = (-r + np.random.normal(0.0, self.vrv, 1) )/self.t_yaw
        self.F[1][0] = (-r)/self.t_yaw
        self.F[2][0] = v_sin_psi_dt
        self.F[3][0] = v_cos_psi_dt
        self.F[4][0] = 0.0 + np.random.normal(0.0, self.vrv, 1)
        self.F[5][0] = (-r + self.k_yaw*u)/self.t_yaw
        # self.F[5][0] = 0.0 + np.random.normal(0.0, 0.01, 1)

        # This is a place where rk45 could be used or some other integration
        # method, but here we just use forward Euler integration
        self.Xhm = self.Xh + self.F

        self.Phih = np.array([
            [1.0, self.dt, 0.0, 0.0, 0.0, 0.0],
            [0.0, -1.0/self.t_yaw, 0.0, 0.0, 0.0, 0.0],
            [v_cos_psi_dt, 0.0, 1.0, 0.0, sin_psi_dt, 0.0],
            [-v_sin_psi_dt, 0.0, 0.0, 1.0, cos_psi_dt, 0.0],
            [0.0, 0.0, 0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 0.0, 0.0, 1.0]
        ])

        self.Pm = ((self.Phih @ self.P @ self.Phih.T) +
                   (self.Gammah @ self.Q @ self.Gammah.T))

        ####
        # Udpate step
        # Linearization for the update step
        self.K = self.Pm @ self.H.T @ np.linalg.inv(
            (self.H @ self.Pm @ self.H.T) + self.R)
        self.Xh = self.Xhm + (self.K @ (y - (self.h @ self.Xhm)))
        self.P = (self.I - (self.K @ self.H)) @ self.Pm

        ###
        # Estimate model parameters: the yaw rate time constant t_yaw

        self.t_yaw_h = self.Xh[5][0]

        if np.abs(self.t_yaw_h) > 0.00001:  # check for division by zero in Phih
            self.t_yaw = self.t_yaw_h

        return self.Xh, self.P, self.K, self.t_yaw_h


class NomotoStateEstimator2():
    '''
    This version is NOT for system identification
    '''

    def __init__(self, dt=Constants.dt_uc, X_0=np.zeros((5, 1)), sigma_r=0.2,
                 sigma_v=0.000001, sigma_psi=0.01, vrpx=0.1, vrpy=0.1,
                 vrpsi=0.1, vrr=0.1, vrv=0.1):

        m, _ = X_0.shape

        self.dt = dt

        self.F = np.zeros((m, 1))

        self.Xhm = X_0
        self.Xh = X_0

        self.Phih = np.zeros((m, m))
        self.Gammah = np.zeros((m, 2))
        # self.Gammah[0][0] = sigma_psi
        self.Gammah[0][0] = sigma_r
        self.Gammah[4][1] = sigma_v

        self.Pm = np.eye(m)

        self.Pm = np.eye(m)
        self.P = np.eye(m)
        self.P[0][0] = vrpsi
        self.P[1][1] = vrr
        self.P[2][2] = vrpx
        self.P[3][3] = vrpy
        self.P[4][4] = vrv

        self.vrv = vrv

        self.Q = np.eye(2)
        self.R = np.eye(2)

        # self.R[0][0] = vrpsi
        self.R[0][0] = vrpx
        self.R[1][1] = vrpy

        self.h = np.array([[0.0, 0.0, 1.0, 0.0, 0.0],
                           [0.0, 0.0, 0.0, 1.0, 0.0]])
        self.H = np.array([[0.0, 0.0, 1.0, 0.0, 0.0],
                           [0.0, 0.0, 0.0, 1.0, 0.0]])

        self.K = np.zeros((m, 1))

        self.k_yaw = 0.5  # for now assume the input gain is 1.0
        self.k_r = 1.0  # for now assume the input gain is 1.0

        self.t_yaw = 0.1
        self.t_yaw_h = 5.0

        self.I = np.eye(m)

        return

    def run(self, y=np.zeros((2, 1)), u=0.0):
        """
        :param y: The sensor measurements as a 2 by 1 vector, including East
        position (meters), North position (meters), @TODO: and heading angle 
        :param u: The commanded rudder angle signal between a bound such as
        -0.8 and 0.8. This is used in system identification
        (rardians)
        """

        psihat = self.Xh[0][0]
        r = self.Xh[1][0]
        vhat = self.Xh[4][0]

        sin_psi_dt = np.sin(psihat)*self.dt
        cos_psi_dt = np.cos(psihat)*self.dt

        v_sin_psi_dt = vhat*sin_psi_dt
        v_cos_psi_dt = vhat*cos_psi_dt

        ####
        # Prediction step
        self.F[0][0] = r*self.dt  # + np.random.normal(0.0, self.vrv, 1)
        self.F[1][0] = r*(-self.t_yaw)
        self.F[2][0] = v_sin_psi_dt
        self.F[3][0] = v_cos_psi_dt
        self.F[4][0] = 0.0 + np.random.normal(0.0, self.vrv, 1)

        # This is a place where rk45 could be used or some other integration
        # method, but here we just use forward Euler integration
        self.Xhm = self.Xh + self.F

        self.Phih = np.array([
            [1.0, self.dt, 0.0, 0.0, 0.0],
            [0.0, -self.t_yaw, 0.0, 0.0, 0.0],
            [v_cos_psi_dt, 0.0, 1.0, 0.0, sin_psi_dt],
            [-v_sin_psi_dt, 0.0, 0.0, 1.0, cos_psi_dt],
            [0.0, 0.0, 0.0, 0.0, 1.0]
        ])

        self.Pm = ((self.Phih @ self.P @ self.Phih.T) +
                   (self.Gammah @ self.Q @ self.Gammah.T))

        ####
        # Udpate step
        # Linearization for the update step
        self.K = self.Pm @ self.H.T @ np.linalg.inv(
            (self.H @ self.Pm @ self.H.T) + self.R)
            
        self.Xh = self.Xhm + (self.K @ (y - (self.h @ self.Xhm)))
        self.P = (self.I - (self.K @ self.H)) @ self.Pm

        return self.Xh, self.P, self.K, self.t_yaw_h
