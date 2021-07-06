import numpy as np
import pandas as pd
from matplotlib import pyplot as plt
import sys, yaml

from utils import Imu, CalibParams
from plot_stats import Stats
from recursive_least_squares import RecursiveLeastSquares

#Import Parameters from yaml

f = open("config.yaml")
dict_config = yaml.safe_load(f)

g = dict_config['gravity']
mfe = dict_config['magnetic']	#Magnetic Field Earth

delta_t = 1.0/dict_config['freq']
theta_calc_prev = None

#RLS parameters
w_initial = None
P_initial = None
lambda_ = dict_config['lambda']

#Create RLS and Stats objects 
#(Temporarily using the same initializations)
rls_acc = RecursiveLeastSquares(lambda_, w_initial, P_initial)
rls_mag = RecursiveLeastSquares(lambda_, w_initial, P_initial)
rls_gyro = RecursiveLeastSquares(lambda_, w_initial, P_initial)

stats = Stats()

#Get data continuously

while True:

	#Function to get Imu data
	#(GPIO or Sim)
	#ToDo

	#Create Imu object
	raw = Imu(None)

	#Use control input for g (if needed)
	ctlr_acc = None

	#RLS Iteration for acc & mag
	acc_vec = rls_acc.create_data_vector(raw.acc, g) #Change g to g+ctrl
	w_acc = rls_acc.step(acc_vec)
	p_acc = CalibParams.from_implicit(w_acc)
	
	mag_vec = rls_mag.create_data_vector(raw.mag, mfe)
	w_mag = rls_mag.step(mag_vec)
	p_mag = CalibParams.from_implicit(w_mag)

	#Correct magnetometer and accelerometer
	acc_calib = p_acc.correct(raw.acc)
	mag_calib = p_mag.correct(raw.mag)
	
	#Extra steps for gyro
	theta_calc_curr = np.cross(acc_calib, mag_calib) #May need transpose; may be in degrees
	if theta_calc_prev == None:
		#Skip gyro calib for first reading
		gyro_calib = raw.gyro
	else:
		delta_theta = np.arccos(theta_calc_curr.T @theta_calc_prev) #May need transpose
		x_gyro = raw.gyro * delta_t

		#RLS for gyro
		gyro_vec = rls_gyro.create_data_vector(x_gyro, delta_theta)
		w_gyro = rls_gyro.step(gyro_vec)
		p_gyro_temp = CalibParams.from_implicit(w_gyro)
		p_gyro = CalibParams(p_gyro_temp.A/np.sqrt(delta_theta), p_gyro_temp.B/delta_theta)

		#Correct gyro
		gyro_calib = p_gyro.correct(raw.gyro)

	#Calibrate
	calib = raw.calibrate(p_acc, p_gyro, p_mag)

	#Update & plot stats
	theta_calc_prev = theta_calc_curr
	stats.append(raw, calib)
	if not stats.num_measurements%20:
		stats.plot_rt()
