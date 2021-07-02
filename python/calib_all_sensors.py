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

	#Create Imu object
	raw = Imu(None)

	#RLS Iteration for acc & mag
	acc_vec = rls_acc.create_data_vector(Imu.acc)
	w_acc = rls_acc.step(acc_vec)
	p_acc = CalibParams.from_implicit(w_acc)
	
	mag_vec = rls_mag.create_data_vector(Imu.mag)
	w_mag = rls_mag.step(mag_vec)
	p_mag = CalibParams.from_implicit(w_mag)
	
	#Extra steps for gyro
	#ToDo


	#RLS for gyro
	p_gyro = None #ToDo


	#Calibrate
	calib = raw.calibrate(p_acc, p_gyro, p_mag)

	#Update & plot stats
	stats.append(raw, calib)
	if not stats.num_measurements%20:
		stats.plot_rt()
