import numpy as np
import pandas as pd
from matplotlib import pyplot as plt
import sys, yaml

from utils import Imu, CalibParams
from plot_stats import Stats

#Import Parameters from yaml

f = open("config.yaml")
dict_config = yaml.safe_load(f)

g = dict_config['gravity']
mfe = dict_config['magnetic']	#Magnetic Field Earth


#Create Stats object
stats = Stats()

#Get data continuously

while True:

	#Function to get Imu data
	#(GPIO or Sim)

	#Create Imu object
	raw = Imu(None)

	#RLS Iteration for acc & mag
	p_acc = None
	p_mag = None

	#Extra steps for gyro

	#RLS for gyro
	p_gyro = None


	#Calibrate
	calib = raw.calibrate(p_acc, p_gyro, p_mag)

	#Update & plot stats
	stats.append(raw, calib)
	if not stats.num_measurements%20:
		stats.plot_rt()
