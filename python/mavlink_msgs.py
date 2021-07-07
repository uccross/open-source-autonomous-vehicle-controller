from pymavlink import mavutil

class Channel:

	"""
	Channel for mavlink messages for IMU, GPS & CalibParams
	"""

	def __init__(self, port, baud):
		self.connection = mavutil.mavlink_connection(port, baud=baud)
		self.type_imu = 'RAW_IMU'
		self.type_gps = None

	def recv(self):
		pass

	def send_params(self, params):
		"""Converts CalibParams object to str & sends msg"""
		pass