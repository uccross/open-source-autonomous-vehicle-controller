from pymavlink import mavutil

class Channel:

	"""
	Channel for mavlink messages for IMU, GPS & CalibParams
	"""

	def __init__(self, port, baud):
		self.connection = mavutil.mavlink_connection(port, baud=baud)
		self.type_imu = 'RAW_IMU'
		self.type_gps = 'GPS_RAW_INT'

	def recv_gps(self):
		try:
			gps_raw = self.connection.messages[self.type_gps]
			stamp = None
			return True, gps_raw
		except:
			print("No GPS message receeived. Extrapolating from previous mesaurement")
			return False, None
	

	def recv_imu(self):
		try:
			imu_raw = self.connection.messages[self.type_imu]
			stamp = None
			return True, imu_raw
		except:
			return False, None

	
	def send_params(self, params):
		"""Converts CalibParams object to str & sends msg"""
		pass