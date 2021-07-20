import numpy as np
from scipy.spatial.transform import Rotation

import ahrs

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
def estimate_orientation(filter_name, acc, mag, args=[], as_angles=False):
	"""
	Estimates the orientation using acc, mag & the given filter
	Args:
		filter_name: Key from static_filters dict
		acc: numpy array of size 3
		mag: numpy arrat of size 3
		args: list of args for filter object
	
	Returns:
		quaternion (np array) if as_angles is False
		euler angles if as_angles is True
	"""
	filter_ = static_filters[filter_name](*args)

	q = filter_.estimate(acc, mag)

	if not as_angles:
		return q
	else:
		r = Rotation.from_quat(q).as_euler('xyz')
		return r

#Test
if __name__ == "__main__":
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
