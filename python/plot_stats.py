"""
Plot Raw and Calibrated IMU data in realtime
"""


import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

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
		self.magnetic = 0.4749 #Gauss
		self.attitude = None #Gyro: Not implemented yet

	def append(self, imu_raw, imu_calib, from_ctrl_input=None):

		#Add new measurment
		self.raw = np.concatenate(self.raw, imu_raw.vec9d(), axis=0)
		self.calib = np.concatenate(self.calib, imu_calib.vec9d(), axis=0)

		#Calculate errors
		e_acc = np.linalg.norm(imu_calib.acc)/self.gravity -1
		e_mag = np.linalg.norm(imu_calib.mag)/self.magnetic -1
		e_gyro = 0 #Todo
		e = np.array([e_acc,e_gyro,e_mag])

		self.errors = np.concatenate(self.errors, e, axis=0)
		self.mse = (self.mse*self.num_measurements + e*e)/(self.num_measurements+1)
		self.var = np.var(errors[1:,:], axis=0)

		self.num_measurements +=1



	def plot_rt(imu_raw, imu_calib):
		#Not implemented
		raise NotImplementedError

		#Create plots & subplots
		fig,ax = plt.subplots(3,2)
		fig.suptitle("", fontsize=15)

		#Plot rolling averages

		#Plot histograms

		#Second plot: calib measurements

		#show
		plt.show()
		pass

	def show_stats(self):
		pass



#Test with csvs
def test():
	pass

if __name__ == "__main__":
	test()