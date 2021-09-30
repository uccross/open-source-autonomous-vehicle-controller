"""
.. module:: main_guide_sys.py
	:platform: MacOS, Unix, Windows,
	:synopsis: The main guidance system for a small autonomous boat
.. moduleauthor:: Pavlo Vlastos <pvlastos@ucsc.edu>
"""
# import numpy as np
import argparse
from mav_csv_logger import MAVCSVLogger as MCL
import time
from signal import signal, SIGINT
from sys import exit

###############################################################################
# Parse Arguments
parser = argparse.ArgumentParser()
parser.add_argument('-b', '--baudrate', type=int, dest='baudrate',
                    default=115200, help='Specify baudrate for serial \
                        communication. Default is 115200')
parser.add_argument('-c', '--com', type=str, dest='com', default="COM4",
                    help='Specify COM port number for serial \
                    communication with micro. Default is 4, as in COM4')  # /dev/ttyUSB1
parser.add_argument('-d', '--debug', dest='debug_flag',
                    action='store_true', help='Flag to print helpful info')
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
debug_flag = arguments.debug_flag
echo_sensor = arguments.echo_sensor
sensor_com = arguments.ecom
sensor_baudrate = arguments.ebaudrate
csv_file = arguments.csv_file
log_file = arguments.log_file


###############################################################################
if __name__ == '__main__':
    # Initialization
    extra_headers = ['data', 'reason', 'base_mode', 'mavlink_version',
                     'custom_mode', 'autopilot', 'system_status']

    my_logger = MCL.MAVCSVLogger(
        com, baudrate, log_file, csv_file, msg_list=[],
        extra_headers=extra_headers, debug_flag=debug_flag)

    ###########################################################################
    # Helper method, based on
    # https://www.devdungeon.com/content/python-catch-sigint-ctrl-c
    def handler(signal_received, frame):  # *args
        """
        Use the MAVCSVLogger object to close the csv file upon ctrl-c program
        exit. The MAVCSVLogger object is in the scope of this main
        :param signal_recieved:
        :param frame:
        """
        # Handle any cleanup here

        my_logger.close_log()  # Close the csv file and mavlink connection

        print('\r\nSIGINT or CTRL-C detected. Exiting gracefully.')
        print('csv file closed')
        print('mavlink connection closed')
        exit(0)

    # Tell Python to run the handler() function when SIGINT is recieved
    signal(SIGINT, handler)

    # Timing
    t_old = time.time()
    t_new = 0
    dt = 0.500  # seconds

    ###########################################################################
    # Main Loop
    while True:

        # Timing
        t_new = time.time()

        # Read the state of the vehicle
        # Request MAVLINK_MSG_ID_RAW_IMU
        # Request MAVLINK_MSG_ID_ATTITUDE
        # Request LOCAL_POSITION_NED
        # Log the vehicle data
        msg = my_logger.mav_conn.recv_match()

        if msg:
            # if msg.get_type() == 'RAW_IMU':
            #     print("\r\nType:")
            #     print(type(msg))

            #     print("\r\nMsg:")
            #     print(msg)

            # if msg.get_type() == 'HIGHRES_IMU':
            # print("\r\nType:")
            # print(type(msg))

            if debug_flag:
                print("\r\nMsg:")
                print(msg)

                print("Time: {}".format(t_new))

        # DO NOT log every message, because tht will quikcly slow down
        # everything
        if (t_new - t_old) >= dt:
            my_logger.log(msg)

        # If the microcontroller indicates that we are in autonomous mode then
        # depending on vehicle position, update the next waypoint to travel to.
        # Else, the we guidance system is not engaged
