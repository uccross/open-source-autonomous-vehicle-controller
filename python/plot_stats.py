"""
Plot Raw and Calibrated IMU data in realtime
"""


import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import time

from utils import Imu


class Stats:
	"""
	Stores raw & calibrated IMU measurements
	Calculates & plots stats
	"""
	def __init__(self):
		self.num_measurements = 0
		
		#List of measurements
		self.raw = np.zeros([1,9])
		self.calib = np.zeros([1,9])
		
		#Errors (Order: acc, gyro, mag)
		self.errors = np.zeros([1,3])
		self.mse = np.zeros(3)
		self.var = np.zeros(3)

		#Temporary (load globally from yaml later)
		self.gravity = 9.799
		self.magnetic = 0.57#0.4749 #Gauss
		self.attitude = None #Gyro: Not implemented yet

		#Figure
		self.fig, self.ax = plt.subplots(3,2)
		self.fig.suptitle("Errors", fontsize=15)


	def append(self, imu_raw, imu_calib, from_ctrl_input=None):

		#Add new measurment
		self.raw = np.concatenate((self.raw, imu_raw.vec9d()), axis=0)
		self.calib = np.concatenate((self.calib, imu_calib.vec9d()), axis=0)

		#Calculate errors
		e_acc = np.linalg.norm(imu_calib.acc)/self.gravity -1
		e_mag = np.linalg.norm(imu_calib.mag)/self.magnetic -1
		e_gyro = 0 #Todo
		e = np.array([e_acc,e_gyro,e_mag]).reshape([1,3])

		self.errors = np.concatenate((self.errors, e), axis=0)

		#Update stats
		self.mse = e**2
		#self.mse = (self.mse*self.num_measurements + e*e)/(self.num_measurements+1)
		self.var = np.var(self.errors[1:,:], axis=0)

		self.num_measurements +=1



	def plot_rt(self):
		"""Plot error stats in real time"""

		#Plot rolling averages
		num_vals = 40 #Average of 40 values
		if self.num_measurements < num_vals:
			return 
		max_prev = 1000 #Plot these many previous values

		x_labels = np.arange(max(0, self.num_measurements-max_prev+1), self.num_measurements-num_vals+2)
		
		self.ax[0,0].clear()
		self.ax[0,0].plot(x_labels, np.convolve(self.errors[-max_prev:,0]**2, np.ones(num_vals), 'valid')/num_vals)

		self.ax[1,0].clear()
		self.ax[1,0].plot(x_labels, np.convolve(self.errors[-max_prev:,1]**2, np.ones(num_vals), 'valid')/num_vals)

		self.ax[2,0].clear()
		self.ax[2,0].plot(x_labels, np.convolve(self.errors[-max_prev:,2]**2, np.ones(num_vals), 'valid')/num_vals)
		
		#Show current MSE
		self.ax[0,0].set_title("MSE (Accelerometer) = " +str(round(self.mse[0,0],4)))
		self.ax[1,0].set_title("MSE (Gyroscope) = " +str(round(self.mse[0,1],4)))
		self.ax[2,0].set_title("MSE: (Magnetometer) = " +str(round(self.mse[0,2],4)))		


		#Plot histograms
		num_bins = 40
		max_errors = 1000 #Plot these many previous values

		self.ax[0,1].clear()
		self.ax[0,1].set_title("Error distribution: Accelerometer, " \
								+"Variance = "+str(np.round(np.var(self.errors[-max_errors:,0]),10)))
		self.ax[0,1].hist(self.errors[1:,0], num_bins)

		self.ax[1,1].clear()
		self.ax[1,1].set_title("Error distribution: Gyroscope, "\
								+"Variance = "+str(np.round(np.var(self.errors[-max_errors:,1]),10)))

		self.ax[1,1].hist(self.errors[1:,1], num_bins)

		self.ax[2,1].clear()
		self.ax[2,1].set_title("Error distribution: Magnetometer, "\
								+"Variance = "+str(np.round(np.var(self.errors[-max_errors:,2]),10)))
		self.ax[2,1].hist(self.errors[1:,2], num_bins)

		#Second plot: calib measurements

		#show
		plt.pause(1e-9)
		pass

	def show(self):

		print("MSE: ", self.mse, "Var: ", self.var)



#Test with csvs
def test():

	stats = Stats()

	#Change test later
	#temporary test:

	debug = False
	n=5_000
	for i in range(n):
		vec = np.random.randn(9)*0.01+9.8
		imu_raw = Imu(vec)
		imu_calib = Imu(vec+np.random.randn(9)*1*(n-i)/(20*n))

		if debug:
			print(imu_raw.vec9d())
			print(imu_calib.vec9d())

		stats.append(imu_raw, imu_calib)
		if not i%10:
			stats.plot_rt()
		plt.pause(0.01)

	pass

if __name__ == "__main__":
	test()