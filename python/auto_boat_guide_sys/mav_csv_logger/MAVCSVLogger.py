"""
.. module:: MAVCSVLogger.py
	:platform: MacOS, Unix, Windows,
	:synopsis: MAVLink csv logging module to be used with main autonomous
    guidance system. The csv will be stored on a USB drive on the Raspberry Pi
    An example call if looks like this:
    python3 MAVCSVLogger.py -c "dev/ttyUSB0" -
.. moduleauthor:: Pavlo Vlastos <pvlastos@ucsc.edu>
"""

from math import log
from numpy.core.fromnumeric import reshape
from pymavlink import mavutil
import csv
import time


class MAVCSVLogger():
    def __init__(self, port='/dev/ttyUSB1', baud=115200,
                 log_file='./logfile.log', csv_file='./logfile.csv',
                 msg_list=[mavutil.mavlink.MAVLink_distance_sensor_message(
            0,
            0,  # echo_sensor_min
            300000,  # echo_sensor_max
            0,  # echo_sensor_distance
            mavutil.mavlink.MAV_DISTANCE_SENSOR_UNKNOWN,  # echo_sensor_type
            0,  # echo_sensor_id
            270,  # echo_sensor_orientation
            0)],  # echo_sensor_covariance
            extra_headers=[], debug_flag=False):
        """
        :param port: The usb com port for serial communication between the
        raspberry pi and the Max32 (or similar microcontroller)
        :param baud: The baudrate
        :param log_file: The file path for the log file
        :param csv_file: The file path for the csv file
        :param msg_list: A list of MAVLink messages that may also be added to
        the set of message types to be logged to the csv file.
        :param extra_headers: A list of any additional headers that might not
        get collected EVEN after specifying additional messages in msg_list.
            example: extra_headers=['signal_quality', 'horizontal_fov']
        """
        self.debug_flag = debug_flag
        if self.debug_flag:
            print("Waiting for vehicle heartbeat")
        self.mav_conn = mavutil.mavlink_connection(port, baud)  # usb on Pi
        self.mav_conn.wait_heartbeat(timeout=5)

        if self.debug_flag:
            if self.mav_conn.target_system == 0:
                print('No system detected!')
            else:
                print('target_system {}, target component {} \n'.format(
                    self.mav_conn.target_system,
                    self.mav_conn.target_component))

        self.log_file = log_file

        # initiate logging
        self.mav_conn.setup_logfile(self.log_file)

        self.csv_file = csv_file

        self.msgs_dict = {}

        # Listen to the usb serial port connecting the microcontroller and the
        # Raspberry Pi
        self.collect_msg_types(wait_time=5, extra_msg_list=msg_list)

        # Put all  keys for all the incoming messages into the headers list
        self.headers = list(self.msgs_dict)

        # If the extra_headers list is not empty, add the extra headers
        if extra_headers != []:
            if self.debug_flag:
                print("Adding extra headers")
            self.headers += extra_headers

        if self.debug_flag:
            print("CSV Header: {}".format(self.headers))

        with open(self.csv_file, 'w', newline='') as csvfile:
            self.writer = csv.DictWriter(csvfile, fieldnames=self.headers)
            self.writer.writeheader()

        # Change to appending
        self.csv_file_obj = open(self.csv_file, 'a')
        self.writer = csv.DictWriter(self.csv_file_obj,
                                     fieldnames=self.headers)
        # extrasaction='ignore',

        return None

    def collect_msg_types(self, wait_time=5, extra_msg_list=None):
        """
        :param wait_time:
        """
        start_time = time.time()
        while (time.time() - start_time) < wait_time:
            try:
                msg = self.mav_conn.recv_match(blocking=True)
                # add msg to the msgs_dict

                if self.debug_flag:
                    print("added msg type: {}".format(msg.get_type()))
                    print("msg content: {}".format(msg))

                self.msgs_dict.update(msg.to_dict())

            except:
                print('msg error, or dict error!')

        # Add all additional messages for expected sensors that are directly
        # connected to the Raspberry Pi via USB
        if extra_msg_list:
            if self.debug_flag:
                print(
                    "Adding extra MAVLink messages for sensors NOT connected to the microcontroller")

            for msg in extra_msg_list:
                if self.debug_flag:
                    print("added extra msg type: {}".format(msg.get_type()))
                    print("extra msg content: {}".format(msg))

                self.msgs_dict.update(msg.to_dict())

        return None

    def log(self,
            msg=mavutil.mavlink.MAVLink_distance_sensor_message(
            0,
            0,  # echo_sensor_min
            300000,  # echo_sensor_max
            0,  # echo_sensor_distance
            mavutil.mavlink.MAV_DISTANCE_SENSOR_UNKNOWN,  # echo_sensor_type
            0,  # echo_sensor_id
            270,  # echo_sensor_orientation
            0)):
        """
        Opens the csv_file and appends a row to the csv file
        :param msg: A MAVlink message
        :return: The MAVLink message type if not none, else return None
        """

        ret_type = None

        # Don't log bad data
        if msg:
            if msg.get_type() != 'BAD_DATA':
                # add msg to the msgs_dict
                self.msgs_dict.update(msg.to_dict())
                # and write it to the file
                self.writer.writerow(self.msgs_dict)

            ret_type = msg.get_type()

        return ret_type

    
    def send_mav_cmd_nav_waypoint(self, wp_lat_lon):
        """
        :param wp_lat_lon: A waypoint with the following structure: 
        np.array([[0.0, 0.1]]) with lattitude as the first element and 
        longitude as the second element
        """
        self.mav_conn.mav.command_long_send(
            self.mav_conn.target_system,
            self.mav_conn.target_component,
            mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
            0,  # Hold
            0.0,  # Accept Radius
            0.0,  # Pass Radius
            0.0,  # Yaw
            wp_lat_lon[0, 0],  # Latitude
            wp_lat_lon[0, 1],  # Longitude
            0.0,
            0.0)

    def close_log(self):
        """
        Wrapper function
        """
        self.csv_file_obj.close()
        self.mav_conn.close()


###############################################################################
# MODULE TEST EXAMPLE
###############################################################################
if __name__ == '__main__':
    import argparse
    from pymavlink import mavutil

    ###########################################################################
    # Parse Arguments
    parser = argparse.ArgumentParser()
    parser.add_argument('-b', '--baudrate', type=int, dest='baudrate',
                        default=115200, help='Specify baudrate for serial \
                            communication. Default is 115200')
    parser.add_argument('-c', '--com', type=str, dest='com', default="COM4",
                        help='Specify COM port number for serial \
                        communication with micro. Default is 4, as in COM4')  # /dev/ttyUSB1
    parser.add_argument('-e', '--echo', dest='echo_sensor',
                        action='store_true', help='To use Blue Robotic\'s echo\
                            sensor')
    parser.add_argument('--ebaudrate', type=int, dest='ebaudrate',
                        default=115200, help='Specify baudrate for serial \
                            communication with echo sounder. Default is \
                            115200')
    parser.add_argument('--ecom', type=str, dest='ecom', default="COM3",
                        help='Specify COM port number for serial \
                            communication with echo sounder. Default is 3, as \
                            in COM3')  # /dev/ttyUSB0

    parser.add_argument('--csv_file', type=str, dest='csv_file',
                        default='./log_file.csv',
                        help='log file path')

    parser.add_argument('--log_file', type=str, dest='log_file',
                        default='./log_file.log',
                        help='log file path')

    arguments = parser.parse_args()

    baudrate = arguments.baudrate
    com = arguments.com
    echo_sensor = arguments.echo_sensor
    sensor_com = arguments.ecom
    sensor_baudrate = arguments.ebaudrate
    csv_file = arguments.csv_file
    log_file = arguments.log_file

    ###########################################################################
    # Example of an echo distance sensor connected to the Raspberry Pi over USB

    # Ping Echo Sounder for depth measurements in water
    if echo_sensor:
        print("Using echo distance sensor")
        from brping import Ping1D
        myPing = Ping1D()
        myPing.connect_serial(sensor_com, sensor_baudrate)

        if myPing.initialize() is False:
            print("Failed to initialize Ping!")
            exit(1)

        time.sleep(1)
        echo_sensor_time = 1000
        echo_sensor_min = 0  # Units: mm, Minimum
        echo_sensor_max = 300000  # Units: mm, Maximum is 300 meters

        # currently we log for a specified period of time

        echo_sensor_type = mavutil.mavlink.MAV_DISTANCE_SENSOR_UNKNOWN
        echo_sensor_id = 0
        echo_sensor_orientation = 270  # Degrees (pointing down)
        echo_sensor_covariance = 0

        echo_sensor_distance = 1

        echo_msg = mavutil.mavlink.MAVLink_distance_sensor_message(
            echo_sensor_time,
            echo_sensor_min,
            echo_sensor_max,
            echo_sensor_distance,
            echo_sensor_type,
            echo_sensor_id,
            echo_sensor_orientation,
            echo_sensor_covariance)

        print("echo field names")
        print(echo_msg.ordered_fieldnames)

        msg_list = [echo_msg]

        for msg in msg_list:
            print("msg type to be added: {}".format(msg.get_type()))
            print("msg to be added: {}".format(msg))

        i = 0

    else:
        msg_list = []

    if echo_sensor:
        print("MAVLink msg list for sensors connected directly to companion\
 computer (Raspberry Pi):")
        for msg in msg_list:
            print(msg)

    extra_headers = ['vertical_fov', 'signal_quality',
                     'horizontal_fov', 'quaternion', 'data', 'reason']

    my_logger = MAVCSVLogger(
        com, baudrate, log_file, csv_file, msg_list, extra_headers,
        debug_flag=True)

    status = None

    # currently we log for a specified period of time
    start_time = time.time()
    logging_time = 10
    while (time.time() - start_time) < logging_time:
        status = my_logger.log(my_logger.mav_conn.recv_match(blocking=True))

        if echo_sensor:
            if status == 'HEARTBEAT':
                for msg in msg_list:
                    my_logger.log(msg)

                i += 100

            echo_sensor_time = i

            echo_data = myPing.get_distance()
            echo_sensor_distance = echo_data["distance"]
            echo_confidence = echo_data["confidence"]

            echo_msg = mavutil.mavlink.MAVLink_distance_sensor_message(
                echo_sensor_time,
                echo_sensor_min,
                echo_sensor_max,
                echo_sensor_distance,
                echo_sensor_type,
                echo_sensor_id,
                echo_sensor_orientation,
                echo_sensor_covariance)

            msg_list = [echo_msg]

    my_logger.close_log()

    print('MAVCSVLogger.py module test finished')
