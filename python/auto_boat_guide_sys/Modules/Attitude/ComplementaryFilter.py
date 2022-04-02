"""
.. module:: ComplementaryFilter.py
    :platform: MacOS, Unix, Windows,
    :synopsis: Unit tests for attitude estimators, such as Complementary 
    Filter, Extended Kalman Filter, and Triad
.. moduleauthor:: Pavlo Vlastos <pvlastos@ucsc.edu>
"""
from multiprocessing.connection import wait
import numpy as np
from ..Utilities import Quaternions as QU


class Explicit():
    def __init__(self, dt, da=30.0*np.pi/180.0, kp_acc=1.0, kp_gyro=1.0,
                 kp_mag=1.0):
        """
        :param dt: Time step in seconds
        :param da: Magnetic dip angle
        :param kp_acc: Proportional gain for accelerometer
        :param kp_mag: Proportional gain for magnetometer
        """
        self.dt = dt
        self.q_gyro = np.zeros((4, 1))

        self.vgi = np.array([[0.0], [0.0], [1.0]])  # gravity (ENU)

        # magnetic field (ENU)
        c = np.cos(da)
        s = np.sin(da)
        self.vmi = np.array([[0.0],
                             [c],
                             [-s]])/np.sqrt((c**2.0) + (s**2.0))

        self.kp_acc = kp_acc
        self.kp_gyro = kp_gyro
        self.kp_mag = kp_mag

    def update(self, v_gyro_b, v_acc_b, v_mag_b, q_est, RT):
        """
        :param v_gyro_b: 3 by 1 gyro vector, x, y, z
        :param v_acc_b: 3 by 1 accelerometer vector
        :param v_mag_b: 3 by 1 magnetometer vector
        :param q_est: Attitude estimate quaternion
        :param RT: DCM rotation matrix transpose
        :return: Euler angles psi, theta, phi, a rotation matrix transposed,
        and the attitude quaternion
        """

        # normalize aiding vectors
        mag_norm = np.linalg.norm(v_mag_b)
        if mag_norm != 0.0:
            v_mag_b /= mag_norm

        acc_norm = np.linalg.norm(v_acc_b)
        if acc_norm != 0.0:
            v_acc_b /= acc_norm

        # Acc to angular velocity
        v_hat_b = (RT @ self.vgi)[:, 0]
        w_meas_acc = np.cross(v_acc_b[:, 0], v_hat_b)

        # Mag to angular velocity
        v_hat_b = (RT @ self.vmi)[:, 0]
        w_meas_mag = np.cross(v_mag_b[:, 0], v_hat_b)

        w_meas_sum = np.zeros((3,))

        w_meas_sum[0] = self.kp_acc*w_meas_acc[0] + self.kp_mag*w_meas_mag[0]
        w_meas_sum[1] = self.kp_acc*w_meas_acc[1] + self.kp_mag*w_meas_mag[1]
        w_meas_sum[2] = self.kp_acc*w_meas_acc[2] + self.kp_mag*w_meas_mag[2]

        # print('w_meas_acc: {}'.format(w_meas_acc))
        # print('w_meas_mag: {}'.format(w_meas_mag))

        # Gyro quaternion
        self.q_gyro[0][0] = 0.0
        self.q_gyro[1][0] = self.kp_gyro*v_gyro_b[0][0] - w_meas_sum[0]
        self.q_gyro[2][0] = self.kp_gyro*v_gyro_b[1][0] - w_meas_sum[1]
        self.q_gyro[3][0] = self.kp_gyro*v_gyro_b[2][0] - w_meas_sum[2]

        q_est_dot = 0.5*QU.multiplyQuaternions(q_est, self.q_gyro)

        q_est = q_est + q_est_dot*self.dt

        q_norm = np.linalg.norm(q_est)

        if q_norm != 0.0:
            q_est /= q_norm

        # Attitude Euler angles
        phi, theta, psi = QU.quaternionToEulerAngles(q_est)

        # rotate aiding vectors from inertial frame into body frame
        R = QU.quaternionToDCM(q_est)
        RT = R.T

        return psi, theta, phi, RT, q_est


class DCM_Based():
    def __init__(self, dt, kp_acc=1.0, kp_gyro=1.0,
                 kp_mag=1.0, ki_acc=0.001, ki_mag=0.001):
        """
        :param dt: Time step in seconds
        :param kp_acc: Proportional gain for accelerometer
        :param kp_mag: Proportional gain for magnetometer
        """
        self.dt = dt
        self.w_gyro = np.zeros((4, 1))

        self.vgi = np.array([[0.0], [0.0], [1.0]])  # gravity (ENU)

        # magnetic field (ENU)
        # see https://www.ngdc.noaa.gov/geomag/calculators/magcalc.shtml?#igrfwmm
        self.vmi = np.array([[5,232.8], [22,703.1], [41,322.4]]) 
        self.vmi /= np.linalg.norm(self.vmi )

        self.kp_acc = kp_acc
        self.kp_gyro = kp_gyro
        self.kp_mag = kp_mag
        
        self.ki_acc = ki_acc
        self.ki_mag = ki_mag

        # Chosen arbitrarily, you might want to choose one based on a smart insight
        self.norm_zero_threshold = 0.001

    def update(self, v_gyro_b, v_acc_b, v_mag_b, R_k, b_minus):
        """
        Based heavily off of the UCSC ECE167 matlab code and the papers 
        referenced in said class for the AHRS lab
        :param v_gyro_b: 3 by 1 gyro vector, x, y, z (body-fixed frame)
        :param v_acc_b: 3 by 1 accelerometer vector (body-fixed frame)
        :param v_mag_b: 3 by 1 magnetometer vector (body-fixed frame)
        :param R_k: DCM rotation matrix from the last time step
        :return: Euler angles psi, theta, phi, a rotation matrix
        """

        # normalize aiding vectors
        mag_norm = np.linalg.norm(v_mag_b)
        if mag_norm != 0.0:
            v_mag_b /= mag_norm

        acc_norm = np.linalg.norm(v_acc_b)
        if acc_norm != 0.0:
            v_acc_b /= acc_norm

        # Acc to angular velocity
        v_hat_b = (R_k @ self.vgi)
        w_acc = np.cross(v_acc_b, v_hat_b.T)
        # w_acc = np.cross(v_hat_b.T, v_acc_b)

        # Mag to angular velocity
        v_hat_b = (R_k @ self.vmi)
        w_mag = np.cross(v_mag_b, v_hat_b.T)
        # w_mag = np.cross(v_hat_b.T, v_mag_b)

        bdot = np.zeros((3)) 
        bdot[0] = -self.ki_acc*w_acc[0][0] - self.ki_mag*w_mag[0][0];
        bdot[1] = -self.ki_acc*w_acc[0][1] - self.ki_mag*w_mag[0][1];
        bdot[2] = -self.ki_acc*w_acc[0][2] - self.ki_mag*w_mag[0][2];

        v_gyro_corrected = np.zeros((3))

        v_gyro_corrected[0] = (self.kp_gyro*v_gyro_b[0]
                               + self.kp_acc*w_acc[0][0]
                               + self.kp_mag*w_mag[0][0]
                               + bdot[0])

        v_gyro_corrected[1] = (self.kp_gyro*v_gyro_b[1]
                               + self.kp_acc*w_acc[0][1]
                               + self.kp_mag*w_mag[0][1]
                               + bdot[1])

        v_gyro_corrected[2] = (self.kp_gyro*v_gyro_b[2]
                               + self.kp_acc*w_acc[0][2]
                               + self.kp_mag*w_mag[0][2]
                               + bdot[2])

        R_k_plus_1 = self.R_exp(v_gyro_corrected) @ R_k

        psi = np.arctan2(R_k_plus_1[1][2], R_k_plus_1[2][2])  # Yaw
        theta = np.arcsin(-R_k_plus_1[0][2])  # Pitch
        phi = np.arctan2(R_k_plus_1[0][1], R_k_plus_1[0][0])  # Roll

        b_plus = b_minus + bdot*self.dt

        return psi, theta, phi, R_k_plus_1, b_plus

    def R_exp(self, w):
        """
        :param w: 3 by 1 vector
        :return: R_exp a 3 by 3 DCM rotation matrix
        """
        # normalize
        w_norm = np.linalg.norm(w)

        w_skew = np.array([[0,     -w[2], w[1]],
                           [w[2],      0, - w[0]],
                           [- w[1], w[0],  0]])

        if w_norm < self.norm_zero_threshold:
            sinc_w = (self.dt - ((self.dt**3)*w_norm**2) /
                      6.0 + ((self.dt**5)*w_norm**4)/120.0)
            one_minus_cosw = ((self.dt**2)/2.0 - ((self.dt**4) *
                                                  w_norm**2)/24.0
                              + ((self.dt**6) * w_norm**4)/720.0)
        else:
            sinc_w = np.sin(w_norm*self.dt)/w_norm
            one_minus_cosw = (1.0 - np.cos(w_norm * self.dt)) / (w_norm**2)

        return np.eye(3) - (sinc_w * w_skew) + (one_minus_cosw * w_skew @ w_skew)
