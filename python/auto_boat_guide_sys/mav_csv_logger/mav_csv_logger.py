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
        self.mav_conn = mavutil.mavlink_connection(port, baud)  # usb on Pi
        self.mav_conn.wait_heartbeat(timeout=5)

        if self.mav_conn.target_system == 0:
            print('No system detected!')
        else:
            print('target_system {}, target component {} \n'.format(
                self.mav_conn.target_system, self.mav_conn.target_component))

        self.log_file = log_file

        # initiate logging
        self.mav_conn.setup_logfile(self.log_file)

        self.csv_file = csv_file

        self.msgs_dict = {}

        # Listen to the usb serial port connecting the microcontroller and the
        # Raspberry Pi
        self.wait_for_msg_types_from_micro()

        # Add all additional messages for expected sensors that are directly
        # connected to the Raspberry Pi via USB
        print("Adding MAVLink messages for sensors NOT connected to the microcontroller")
        for msg in msg_list:
            print("added: {}".format(msg.get_type()))
            print("msg: {}".format(msg))
            self.msgs_dict.update(msg.to_dict())

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
                msg = self.mav_conn.recv_match(blocking=True)
                # add msg to the msgs_dict
                self.msgs_dict.update(msg.to_dict())
            except:
                print('msg error, or dict error!')

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

        with open(self.csv_file, 'a') as csvfile:
            self.writer = csv.DictWriter(csvfile, fieldnames=self.headers)
            # try:

            print("Type:")
            print(type(msg))

            print("\r\nMsg:")
            print(msg)

            # add msg to the msgs_dict
            self.msgs_dict.update(msg.to_dict())
            # and write it to the file
            self.writer.writerow(self.msgs_dict)

            # except:
            #     time.sleep(1)  # TODO:get rid of this sleep
            #     print('msg error, or dict error!')

        # self.mav_conn.close()

        return msg.get_type()


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
                        default="./csv_file.csv",
                        help='log file path')

    parser.add_argument('--log_file', type=str, dest='log_file',
                        default="./log_file",
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
    # Example of a sensor connected to the Raspberry Pi over USB

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
        echo_sensor_time = 1
        echo_sensor_min = 0  # Units: mm, Minimum
        echo_sensor_max = 300000  # Units: mm, Maximum is 300 meters

        echo_data = myPing.get_distance()
        echo_sensor_distance = echo_data["distance"]  # Units: mm
        echo_confidence = echo_data["confidence"]

        echo_sensor_type = mavutil.mavlink.MAV_DISTANCE_SENSOR_UNKNOWN
        echo_sensor_id = 0
        echo_sensor_orientation = 270  # Degrees (pointing down)
        echo_sensor_covariance = 0

        echo_msg = mavutil.mavlink.MAVLink_distance_sensor_message(
            echo_sensor_time,
            echo_sensor_min,
            echo_sensor_max,
            echo_sensor_distance,
            echo_sensor_type,
            echo_sensor_id,
            echo_sensor_orientation,
            echo_sensor_covariance)

        print("msg type to be added: {}".format(echo_msg.get_type()))
        print("msg to be added: {}".format(echo_msg))
        msg_list = [echo_msg]

        i = 0

    else:
        msg_list = []

    print("MAVLink msg list for sensors connected directly to companion computer (Raspberry Pi):")
    for msg in msg_list:
        print(msg)

    my_logger = mav_csv_logger(com, baudrate, csv_file, log_file, msg_list)

    status = None

    # currently we log for a specified period of time
    start_time = time.time()
    logging_time = 10
    while (time.time() - start_time) < logging_time:
        status = my_logger.log(
            msg=my_logger.mav_conn.recv_match(blocking=True))

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

    print('mav_csv_logger.py module test finished')
