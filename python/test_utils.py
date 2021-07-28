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