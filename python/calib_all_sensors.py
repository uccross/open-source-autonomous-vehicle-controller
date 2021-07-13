import numpy as np
import pandas as pd
from matplotlib import pyplot as plt
import sys, yaml

from utils import Imu, CalibParams
from plot_stats import Stats
from recursive_least_squares import RecursiveLeastSquares
import mavlink_msgs as mav

#Import Parameters from yaml

f = open("config.yaml")
dict_config = yaml.safe_load(f)

g = dict_config['gravity']
mfe = dict_config['magnetic']	#Magnetic Field Earth

delta_t = 1.0/dict_config['freq']
theta_calc_prev = None

#Mavlink parameters
baud_rate = dict_config['baud']
port = dict_config['port']

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

#Initialize mavlink channel
channel = mav.Channel(port, baud_rate)

#Recv & calibrate data continuously

while True:

	#Function to get Imu data
	bool_imu, imu_raw = channel.recv_imu()
	while not bool_imu:
		print("Waiting for IMU")
		bool_imu, imu_raw = channel.recv_imu()

	#Create Imu object using imu_raw attributes
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
	bool_gps, gps_raw = channel.recv_gps()
	if not bool_gps:
		#Todo: interpolate GPS heading
		gps_raw = None
	#Todo: Get COD
	#Todo: Use filter to get orientation
	delta_theta = None 
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
	stats.append(raw, calib)
	if not stats.num_measurements%20:
		stats.plot_rt()
