"""
.. module:: main_guide_sys.py
	:platform: MacOS, Unix, Windows,
	:synopsis: MAVLink csv logging module to be used with main autonomous 
    guidance system. The csv will be stored on a USB drive on the raspberry pi
.. moduleauthor:: Pavlo Vlastos <pvlastos@ucsc.edu>
"""

from math import log
from pymavlink import mavutil
import csv
import time


class mav_csv_logger():
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
            0)]):  # echo_sensor_covariance
        """
        :param port: The usb com port for serial communication between the 
        raspberry pi and the Max32 (or similar microcontroller)
        :param baud: The baudrate 
        :param log_file: The file path for the log file
        :param csv_file: The file path for the csv file
        :param msg_list: A list of MAVLink messages that may also be added to 
        the set of message types to be logged to the csv file.
        """
        self.log_con = mavutil.mavlink_connection(port, baud)  # usb on Pi
        self.log_con.wait_heartbeat(timeout=5)

        if self.log_con.target_system == 0:
            print('No system detected!')
        else:
            print('target_system {}, target component {} \n'.format(
                self.log_con.target_system, self.log_con.target_component))

        self.log_file = log_file

        # initiate logging
        self.log_con.setup_logfile(self.log_file)

        self.csv_file = csv_file

        self.msgs_dict = {}

        # Listen to the usb serial port connecting the microcontroller and the
        # Raspberry Pi
        self.wait_for_msg_types_from_micro()

        # Add all additional messages for expected sensors that are directly
        # connected to the Raspberry Pi via USB
        for msg in msg_list:
            self.add_msg_type(msg)

        # Put all  keys for all the incoming messages into the headers list
        self.headers = list(self.msgs_dict)
        with open(self.csv_file, 'w', newline='') as csvfile:
            self.writer = csv.DictWriter(csvfile, fieldnames=self.headers)
            self.writer.writeheader()

        return None

    def wait_for_msg_types_from_micro(self, wait_time=5):
        """
        :param wait_time:
        """
        start_time = time.time()
        while (time.time() - start_time) < wait_time:
            try:
                msg = self.log_con.recv_match(blocking=True)
                # add msg to the msgs_dict
                self.msgs_dict.update(msg.to_dict())
            except:
                print('msg error, or dict error!')

        return None

    def add_msg_type(self, msg=mavutil.mavlink.MAVLink_distance_sensor_message(
            0,
            0,  # echo_sensor_min
            300000,  # echo_sensor_max
            0,  # echo_sensor_distance
            mavutil.mavlink.MAV_DISTANCE_SENSOR_UNKNOWN,  # echo_sensor_type
            0,  # echo_sensor_id
            270,  # echo_sensor_orientation
            0)):
        """
        :param msg: A MAVLink message to be added to the dicitonary. This 
        message should probably come from a sensor that is NOT connected to the 
        microcontroller but *is* connected to the raspberry pi, because the
        function 'wait_for_msg_types_from_micro()' only listens to the serial 
        port associated with the micrcontroller.
        """
        self.msgs_dict.update(msg.to_dict())

        return None

    def log(self):
        """
        Opens the csv_file and appends a row to the csv file
        :return: The MAVLink message type if not none, else return None
        """
        msg = None

        with open(self.csv_file, 'a') as csvfile:
            self.writer = csv.DictWriter(csvfile, fieldnames=self.headers)
            try:
                msg = self.log_con.recv_match(blocking=True)

                print("Type:")
                print(type(msg))

                print("\r\nMsg:")
                print(msg)

                # add msg to the msgs_dict
                self.msgs_dict.update(msg.to_dict())
                # and write it to the file
                self.writer.writerow(self.msgs_dict)

            except:
                time.sleep(1)  # TODO:get rid of this sleep
                print('msg error, or dict error!')

        # self.log_con.close()

        if msg != None:
            return msg.get_type()

        return None

    def add_message_row_to_csv(
        self,
        msg=mavutil.mavlink.MAVLink_distance_sensor_message(
            0,
            0,  # echo_sensor_min
            300000,  # echo_sensor_max
            0,  # echo_sensor_distance
            mavutil.mavlink.MAV_DISTANCE_SENSOR_UNKNOWN,  # echo_sensor_type
            0,  # echo_sensor_id
            270,  # echo_sensor_orientation
            0)):

        with open(self.csv_file, 'a'):
            msg = self.log_con.recv_match(blocking=True)

            print("Type:")
            print(type(msg))

            print("\r\nMsg:")
            print(msg)

            # add msg to the msgs_dict
            self.msgs_dict.update(msg.to_dict())
            # and write it to the file
            self.writer.writerow(self.msgs_dict)

        return None

###############################################################################
if __name__ == '__main__':
    from brping import Ping1D
    import argparse

    ###########################################################################
    # Parse Arguments
    parser = argparse.ArgumentParser()
    parser.add_argument('-b', '--baudrate', type=int, dest='baudrate',
                        default=115200, help='Specify baudrate for serial \
                            communication. Default is 115200')
    parser.add_argument('-c', '--com', type=str, dest='com', default="COM4",
                        help='Specify COM port number for serial \
                        communication with micro. Default is 4, as in COM4')  # /dev/ttyUSB1

    parser.add_argument('--ebaudrate', type=int, dest='ebaudrate',
                        default=115200, help='Specify baudrate for serial \
                            communication with echo sounder. Default is \
                            115200')
    parser.add_argument('--ecom', type=str, dest='ecom', default="COM3",
                        help='Specify COM port number for serial \
                            communication with echo sounder. Default is 3, as \
                            in COM3')  # /dev/ttyUSB0

    parser.add_argument('--csv_file', type=str, dest='csv_file',
                        default="./csv_file",
                        help='log file path')

    parser.add_argument('--log_file', type=str, dest='log_file',
                        default="./log_file",
                        help='log file path')

    arguments = parser.parse_args()

    baudrate = arguments.baudrate
    com = arguments.com
    sensor_com = arguments.ecom
    sensor_baudrate = arguments.ebaudrate
    csv_file = arguments.csv_file
    log_file = arguments.log_file

    ###########################################################################
    # Example of a sensor connected to the Raspberry Pi over USB

    # Ping Echo Sounder for depth measurements in water
    myPing = Ping1D()
    myPing.connect_serial(sensor_com, sensor_baudrate)

    if myPing.initialize() is False:
        print("Failed to initialize Ping!")
        exit(1)

    msg_list = [mavutil.mavlink.MAVLink_distance_sensor_message(
        0,
        0,  # echo_sensor_min
        300000,  # echo_sensor_max
        0,  # echo_sensor_distance
        mavutil.mavlink.MAV_DISTANCE_SENSOR_UNKNOWN,  # echo_sensor_type
        0,  # echo_sensor_id
        270,  # echo_sensor_orientation
        0)]

    my_logger = mav_csv_logger(port=com, baud=baudrate, csv_file=csv_file,
                               log_file=log_file, msg_list=msg_list)

    status = None
    i = 0

    # currently we log for a specified period of time
    start_time = time.time()
    logging_time = 10
    while (time.time() - start_time) < logging_time:
        status = my_logger.log()

        if status != None:
            for msg in msg_list:
                my_logger.add_message_row_to_csv(msg)

        msg_list = [mavutil.mavlink.MAVLink_distance_sensor_message(
        0,
        0,  # echo_sensor_min
        300000,  # echo_sensor_max
        i,  # echo_sensor_distance
        mavutil.mavlink.MAV_DISTANCE_SENSOR_UNKNOWN,  # echo_sensor_type
        0,  # echo_sensor_id
        270,  # echo_sensor_orientation
        0)]

        i += 100

    print('mav_csv_logger.py module test finished')