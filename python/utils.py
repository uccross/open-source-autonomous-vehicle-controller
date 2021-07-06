import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

"""
Common classes and functions required for the stack
"""


class Imu:
	"""
	Single IMU reading"
	"""

	def __init__(self, values):
		"""
		values: 9d vector
		[ax ay az gx gy gz mx my mz]
		"""
		self.acc = np.array(values[0:3]).reshape([3,1])
		self.gyro = np.array(values[3:6]).reshape([3,1])
		self.mag = np.array(values[6:9]).reshape([3,1])


	def vec9d(self):
		return np.array([*self.acc, *self.gyro, *self.mag]).reshape([1,9])

	def calibrate(p_acc, p_gyro, p_mag):
		"""
		Calibrates an IMU measurement using known parameters
		Args:
			p_acc, p_gyro, p_mag: CalibParams objects

		Reurns: Imu object of calibrated measurements
		"""
		xcal_acc = np.linalg.inv(p_acc.A) @(self.acc - p_acc.B)
		xcal_gyro = np.linalg.inv(p_gyro.A) @(self.gyro - p_gyro.B)
		xcal_mag = np.linalg.inv(p_mag.A) @(self.mag - p_mag.B)

		return Imu([*xcal_acc, *xcal_gyro, *xcal_mag])


#
class CalibParams:
	"""
	Calibration parameters
	"""
	def __init__(self, A, B, R=None, scales=None):
		self.A = A
		self.B = B.reshape([1,3])


	@classmethod
	def from_implicit(cls, w, verbose=False):
		"""
		Converts implicit linear form of the ellipse to 
		the matrix form (AX+B)@(AX+B).T = 1
		and initializes class using computed A & B

		Args:
			w: 9x1 matrix of implicit parameters of the ellipse
		"""
		w = w.reshape([9,])

		#Matrix form of ellipse
		Q = np.array([[w[0], w[3]/2, w[4]/2, w[6]/2],\
						[w[3]/2, w[1], w[5]/2, w[7]/2],\
						[w[4]/2, w[5]/2, w[2], w[8]/2],\
						[w[6]/2, w[7]/2, w[8]/2, -1.0]])

		#Recover bias
		B = np.linalg.lstsq(Q[:3,:3], -Q[:3,3], rcond=None)[0]

		#Translate
		T = np.eye(4)
		T[:3,3] = B
		Q2 = T.T @Q @T

		#Find eigenvalues
		eig_vals, eig_vecs = np.linalg.eig(Q2[:3,:3])

		#Recover Rotation and Scales
		rearrange = np.array([[0,0,1], [1,0,0],[0,1,0]]) #Temporary workaround for axis rearrange
		R = -eig_vecs @ rearrange
		scales = np.sqrt(-Q2[3,3]/eig_vals) @ rearrange

		#Recover A
		A = R@ np.diag(scales) @R.T

		if verbose:
			print("\nRot: ",R)
			print("\nCross matrix: ", R.T@R)
			print("\nScales ", scales)
			print("\n---\nA = ", A)
			print("\nBias = ", B)
		
		return cls(A,B)

	def correct(self, x_raw):
		"""Calibrates x_raw and returns x_calib"""

		return np.linalg.inv(self.A) @(x_raw - self.B).T