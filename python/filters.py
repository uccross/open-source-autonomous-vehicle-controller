import numpy as np
import pandas as pd
from scipy.spatial.transform import Rotation

import ahrs

import sys, yaml

#Dictionary of static filters
static_filters = {'DAVEN':ahrs.filters.davenport.Davenport ,\
					'FAMC':ahrs.filters.FAMC ,\
					'FLAE':ahrs.filters.FLAE ,\
					'FQA':ahrs.filters.fqa.FQA ,\
					'OLEQ':ahrs.filters.OLEQ ,\
					'QUEST':ahrs.filters.quest.QUEST ,\
					#'ROLEQ':ahrs.filters.ROLEQ, \
					'SAAM':ahrs.filters.SAAM ,\
					'TILT':ahrs.filters.Tilt ,\
					'TRIAD':ahrs.filters.TRIAD \
					}


#
def estimate_orientation(filter_name, acc, mag, args=[], as_angles=False, create_filter=True):
	"""
	Estimates the orientation using acc, mag & the given filter
	Args:
		filter_name: Key from static_filters dict
		acc: numpy array of size 3
		mag: numpy arrat of size 3
		args: list of args for filter object
		as_angles: return angles instead of quaternions
		create_filter: If true, creates new filter object of type filtername
						if false, uses filter_name as the object	
	Returns:
		quaternion (np array) if as_angles is False
		euler angles if as_angles is True
	"""
	if create_filter:
		filter_ = static_filters[filter_name](*args)
	else:
		filter_ = filter_name


	q = filter_.estimate(acc, mag)

	if not as_angles:
		return q
	else:
		r = Rotation.from_quat(q).as_euler('xyz')
		return r

def get_delta_theta(angles):
	"""
	Finds the magnitude of difference between consecutive angles
	angles: np array of shape [n,3] containing xyz euler angles
	"""
	return np.linalg.norm(angles[:-1,:]-angles[1:,:], axis=1)

#TESTS

#Tst 1: Single values, all filters, default params
def test1(*args):
	"""
	For debugging
	Hardcoded values: consecutive acc-mag pairs and timestamp difference
	Prints angular velocity estimated by all static filters
	"""
	acc1 = np.array([34, 188, 16615])
	mag1 = np.array([79, 156, 418])

	acc2 = np.array([-10, 195, 16536])
	mag2 = np.array([84, 165, 420])

	#print("Mag: ",mag,"\nAcc: ",acc)

	print("\nAngular velocity (radians/s) -")
	for key in static_filters:
		try:
			theta1 = estimate_orientation(key, acc1, mag1, as_angles=True)#*180/np.pi
			theta2 = estimate_orientation(key, acc2, mag2, as_angles=True)#*180/np.pi
			print(key+": ", (theta2-theta1)/20097e-6)
		except:
			pass
			#print(key+": ERROR")


#Test 2: Multiple values from files, single filter, params from yaml
def test2(csv_acc, csv_mag, ahrs_params_file, key, *args):
	"""
	Takes input from logged acc & mag csv files
	Loads params from yaml file
	Returns attitude estimates for each acc-mag pair using the given filter (key)
	"""
	#Create matrices for first num_val values of mag & acc
	num_vals = 100
	start_index = 150

	df_acc = pd.read_csv(csv_acc)
	df_mag = pd.read_csv(csv_mag)

	acc_raw = df_acc.values[start_index:num_vals+start_index,:]
	mag_raw = df_mag.values[start_index:num_vals+start_index,:]

	#Import parameters for each AHRS algorithm from YAML

	f = open(ahrs_params_file)
	params = yaml.safe_load(f)

	#Iterate through list of filters and estimate attitude:
	if key in ['DAVEN', 'FLAE', 'QUEST']:
		#Create object
		filter_ = static_filters[key](acc_raw, mag_raw,\
										weights = np.array(params[key][2]),\
										magnetic_dip = params[key][0],\
										gravity = params[key][1],\
										method = params[key][3])
		#print(filter_.estimate(acc_raw[0], mag_raw[0]))

		angles = []
		for i in range (num_vals):
			angles.append(estimate_orientation(filter_, acc_raw[i], mag_raw[i], as_angles=True, create_filter=False))
		
		return np.array(angles)

	elif key in ['FAMC', 'FQA', 'SAAM', 'TILT']:
		filter_ = static_filters[key](acc_raw, mag_raw, *params[key])
		q = filter_.Q
		return Rotation.from_quat(q).as_euler('xyz')


	elif key in ['OLEQ']:
		filter_ = static_filters[key](acc_raw, mag_raw, *params[key])#np.array(params[key][0]), frame=params[key][1])
		angles = []
		for i in range (num_vals):
			angles.append(estimate_orientation(filter_, acc_raw[i], mag_raw[i], as_angles=True, create_filter=False))
		return np.array(angles)


	# elif key == 'TRIAD':
	# 	angles = []
	# 	for i in range(num_vals):
	# 		filter_ = static_filters[key](acc_raw[i], mag_raw[i], *params[key])
	# 		q = filter_.A
	# 		angles.append(Rotation.from_quat(q).as_euler('xyz'))
	# 	return np.array(angles)


	else:
		print("Key not found: ", key)
		return

#Test3: Multiple values from files, multiple filters, params from YAML
def test3(csv_acc, csv_mag, ahrs_params_file, ref, *args, **kwargs):
	"""
	Runs test 2 for all filters and compares results with a given filter
	"""
	#Dictionary for results
	results = {}

	#Iterate through list of filters and estimate attitude:

	for key in static_filters:
		try:
			angles = test2(csv_acc, csv_mag, ahrs_params_file, key)
		except:
			continue

		results[key] = angles



	#Compare results of reference filter to all other filters
	#Difference between results should be constant, i.e. std.dev should be near zero
	#ref = 'DAVEN'
	print("\nReference: ", key, '\n')
	for key in static_filters:
		if key == 'TRIAD':
			continue
		print( np.std(results[ref]-results[key], axis=0) *180/np.pi, key )

	#End
	return


#Main
if __name__ == "__main__":
	test1()

	#print(test2(*sys.argv[1:], 'DAVEN'))
	#print ( get_delta_theta(test2(*sys.argv[1:])) )
	test3(*sys.argv[1:])

