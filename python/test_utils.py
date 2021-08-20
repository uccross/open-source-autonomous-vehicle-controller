"""
All utilities required for testing algorithms
"""

import numpy as np
import pandas as pd
import csv


"""
Function for reading logged data line by line
(Replacement for actual data stream)
"""

def get_next_meas(df_imu):
	
	stamp = int(df_imu.loc[get_next_meas.index, 1]) #uSec

	fs = 2**15

	acc = df_imu.loc[get_next_meas.index, 2:4].to_numpy(dtype=np.float32)#*2*9.80665/fs
	gyro = df_imu.loc[get_next_meas.index, 5:7].to_numpy(dtype=np.float32)#*(500*np.pi/180)/fs
	mag = df_imu.loc[get_next_meas.index, 8:10].to_numpy(dtype=np.float32)#*0.0049/fs

	values = acc.tolist() + gyro.tolist() + mag.tolist()

	get_next_meas.index+=1

	return values, stamp

get_next_meas.index=1



def get_next_meas_calib(df_imu, calib_params):
	"""
	Uses initial batch calibration parameters from calib_params
	to pre-calibrate each new measurement
	
	Args:
		df_imu: Pandas dataframe
		calib_params: array containing utils.CalibParams objects for
						acc, gyro and mag respectively

	Returns pre-calibrated values for further RLS calibration
	"""

	p_acc, p_gyro, p_mag = calib_params

	#Get raw vaues from dataframe
	stamp = int(df_imu.loc[get_next_meas_calib.index, 1]) #uSec

	acc_raw = df_imu.loc[get_next_meas_calib.index, 2:4].to_numpy(dtype=np.float32)
	gyro_raw = df_imu.loc[get_next_meas_calib.index, 5:7].to_numpy(dtype=np.float32)
	mag_raw = df_imu.loc[get_next_meas_calib.index, 8:10].to_numpy(dtype=np.float32)


	#Calibrated measurements
	acc = p_acc.correct(acc_raw)
	gyro = p_gyro.correct(gyro_raw)
	mag = p_mag.correct(mag_raw)

	values = acc.tolist() + gyro.tolist() + mag.tolist()

	#Update index
	get_next_meas_calib.index+=1

	return values, stamp

get_next_meas_calib.index=1