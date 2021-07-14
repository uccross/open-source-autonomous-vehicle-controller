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
					#'ROLEQ':ahrs.filters.ROLEQ \
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
	mag = np.array([0.004, 0.002, 0.001])
	acc = np.array([0.001, -9.823, 0.12])

	for key in static_filters:
		try:
			print(key+": ", estimate_orientation(key, acc, mag, as_angles=True))
		except:
			print(key+": ERROR")
