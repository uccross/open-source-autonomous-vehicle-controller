import numpy as np
import pandas as pd
from matplotlib import pyplot as plt
import sys, yaml

from utils import Imu, CalibParams
from plot_stats import Stats
from recursive_least_squares import RecursiveLeastSquares
import mavlink_msgs as mav
import test_utils, filters

#Import Parameters from yaml

f = open("config.yaml")
dict_config = yaml.safe_load(f)

g = dict_config['gravity']
mfe = dict_config['magnetic']	#Magnetic Field Earth
use_tolerance = dict_config['use_tolerance']
g_tolerance = g*dict_config['acc_tolerance']/100
m_tolerance = mfe*dict_config['mag_tolerance']/100

delta_t = 1.0/dict_config['freq']
theta_calc_prev = None

#Mavlink parameters
baud_rate = dict_config['baud']
port = dict_config['port']

#Initial batch calibration parameters
b_use_batch = dict_config['use_initial_batch']
if b_use_batch:
	p_acc_init = CalibParams.from_saved(dict_config['acc_file'])
	p_gyro_init = CalibParams.from_saved(dict_config['gyro_file'])
	p_mag_init = CalibParams.from_saved(dict_config['mag_file'])

#RLS parameters
w_initial = np.zeros([9,1])	#Get from initial batch
P_initial = np.eye(9)
lambda_ = dict_config['lambda']

#-------------------------------------------------------

#Create RLS and Stats objects 
#(Temporarily using the same initializations)
rls_acc = RecursiveLeastSquares(lambda_, w_initial, P_initial)
rls_mag = RecursiveLeastSquares(lambda_, w_initial, P_initial)
rls_gyro = RecursiveLeastSquares(lambda_, w_initial, P_initial)

stats = Stats()

#Initialize mavlink channel
#channel = mav.Channel(port, baud_rate)

#Temporary code for testing:
test_utils.get_next_meas.index = 1
test_utils.get_next_meas_calib.index = 1
df_imu = pd.read_csv(sys.argv[1], header=None)

#Recv & calibrate data continuously
while True:

	#Function to get Imu data
	"""
	bool_imu, imu_raw = channel.recv_imu()
	while not bool_imu:
		print("Waiting for IMU")
		bool_imu, imu_raw = channel.recv_imu()
	"""

	#Temporary: Get test data:
	if test_utils.get_next_meas.index>5000 or test_utils.get_next_meas_calib.index>2000:
		break
	
	if b_use_batch:
		imu_raw, stamp = test_utils.get_next_meas_calib(df_imu, [p_acc_init,p_gyro_init,p_mag_init])
	else:
		imu_raw, stamp = test_utils.get_next_meas(df_imu)

	#Create Imu object using imu_raw attributes
	raw = Imu(imu_raw, stamp)

	#Use control input for g (if needed)
	ctlr_acc = np.zeros([3,1]) #Replace with acc from dynamic model or encoders
	raw.acc = raw.acc - ctlr_acc

	#RLS Iteration for acc & mag
	acc_vec = rls_acc.create_data_vector(raw.acc, g)#Change g to g+ctrl
	w_acc = rls_acc.step(acc_vec)
	try:
		p_acc = CalibParams.from_implicit(w_acc)
		
		#Prune outliers
		if use_tolerance:
			assert(np.abs(np.linalg.norm(raw.acc)-g)<g_tolerance)
	except:
		w_acc = rls_acc.restore()
		p_acc = CalibParams.from_implicit(w_acc)
	
	mag_vec = rls_mag.create_data_vector(raw.mag, mfe)
	w_mag = rls_mag.step(mag_vec)
	
	try:
		p_mag = CalibParams.from_implicit(w_mag)
		
		#Prune outliers
		if use_tolerance:
			assert(np.abs(np.linalg.norm(raw.mag)-mfe)<m_tolerance)
	except:
		w_mag = rls_mag.restore()
		p_mag = CalibParams.from_implicit(w_mag)

	#Fix constant / denormalize
	p_mag.B = p_mag.B*mfe
	p_acc.B = p_acc.B*g

	#Correct magnetometer and accelerometer
	acc_calib = p_acc.correct(raw.acc)
	mag_calib = p_mag.correct(raw.mag)
	
	#Extra steps for gyro

	"""
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
	"""
	#Calibrate
	calib = raw.calibrate(p_acc, p_mag, p_mag)

	#Update & plot stats
	if np.sum(np.isnan(calib.acc)):
		continue #Why is it NaN though??

	stats.append(raw, calib)
	
	if not stats.num_measurements%20:
		pass
		stats.plot_rt()
		print(raw.acc,calib.acc)
		stats.show()
plt.show()
