"""
.. module:: main_guide_sys.py
	:platform: MacOS, Unix, Windows,
	:synopsis: The main guidance system for a small autonomous boat
.. moduleauthor:: Pavlo Vlastos <pvlastos@ucsc.edu>
"""
import numpy as np
import argparse
from mav_csv_logger import MAVCSVLogger as MCL
from path_planner import WaypointQueue as WQ
from utilities import LTPconvert as LTP
from utilities import AttitudeVisualization as AV
from pymavlink import mavutil
import time
from signal import signal, SIGINT
from sys import exit
import copy

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
parser.add_argument('-m', '--mode', dest='mode_print_flag',
                    action='store_true', help='Flag to print mode changes')
parser.add_argument('-v', '--vizualize', dest='vizualize_attitude_flag',
                    action='store_true', help='Flag to print mode changes')

arguments = parser.parse_args()

baudrate = arguments.baudrate
com = arguments.com
debug_flag = arguments.debug_flag
echo_sensor = arguments.echo_sensor
sensor_com = arguments.ecom
sensor_baudrate = arguments.ebaudrate
csv_file = arguments.csv_file
log_file = arguments.log_file
mode_print_flag = arguments.mode_print_flag
vizualize_attitude_flag = arguments.vizualize_attitude_flag

###############################################################################
if __name__ == '__main__':
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

    msg_list = [echo_msg]

    # Initialization
    extra_headers = ['vertical_fov', 'signal_quality',
                     'horizontal_fov', 'quaternion',
                     'data', 'reason', 'base_mode', 'mavlink_version',
                     'custom_mode', 'autopilot', 'system_status', 'type',
                     'vy', 'y', 'vx', 'vz', 'z', 'x', 'command', 'result']

    # Data logger
    logger = MCL.MAVCSVLogger(
        com, baudrate, log_file, csv_file, msg_list=msg_list,
        extra_headers=extra_headers, debug_flag=False)

    # Attitude Vizualizor
    if vizualize_attitude_flag:
        av = AV.AttitudeVizualizer(debugFlag=False, graphInterval=100)

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

        logger.close_log()  # Close the csv file and mavlink connection

        print('\r\nSIGINT or CTRL-C detected. Exiting gracefully.')
        print('csv file closed')
        print('mavlink connection closed')
        exit(0)

    # Tell Python to run the handler() function when SIGINT is recieved
    signal(SIGINT, handler)

    status = None

    # Timing
    t_new = 0
    t_old = time.time()
    # t_sm = time.time()
    dt = 0.005  # seconds
    xacc = 0.0
    yacc = 0.0
    zacc = 0.0

    yaw = 0.0
    pitch = 0.0
    roll = 0.0

    heading_angle = 0.0

    # State Machine Transitions
    last_base_mode = -1
    state = 'IDLE'
    last_state = state
    ack_result = {'ERROR_WP': 0,
                  'FINDING_REF_WP': 1,
                  'CHECKING_REF_WP': 2,
                  'WAITING_FOR_PREV_WP': 3,
                  'CHECKING_PREV_WP': 4,
                  'WAITING_FOR_NEXT_WP': 5,
                  'CHECKING_NEXT_WP': 6,
                  'TRACKING_WP': 7}

    # Waypoint Queue       N/S Lat   , E/W Long
    waypoints = np.array([[36.9557439, -122.0604691],
                          [36.9556638, -122.0606960],
                          [36.9554362, -122.0607348],
                          [36.9556224, -122.0604107]])
    wpq = WQ.WaypointQueue(waypoint_queue=waypoints, threshold=2.5)

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
        msg = logger.mav_conn.recv_match()  # TODO: Make a getter() for this

        # Check messages to update the state machine
        if msg:
            msg_type = msg.get_type()

            if ((msg_type != 'RC_CHANNELS_RAW') and
                    (msg_type != 'HIGHRES_IMU') and
                    (msg_type != 'GPS_RAW_INT') and
                    (msg_type != 'HEARTBEAT') and
                    (msg_type != 'ATTITUDE') and
                    (msg_type != 'COMMAND_ACK') and
                    (msg_type != 'BAD_DATA')):
                print("    msg.get_type() = {}".format(msg_type))

            ##################################################################
            # START OF STATE MACHINE
            if state == 'IDLE':
                state = 'WAITING_FOR_REF_POINT'

            elif state == 'WAITING_FOR_REF_POINT':

                # If we get the following message type, echo back the reference
                # waypoint
                if msg_type == 'LOCAL_POSITION_NED':
                    nav_msg = msg.to_dict()

                    lon = nav_msg['x']  # longitude
                    lat = nav_msg['y']  # latitude

                    wp_ref_lat_lon = np.array([[lat, lon]])
                    wp_ref_lla = np.array([[lat, lon, 0.0]])

                    logger.send_mav_cmd_nav_waypoint(wp_ref_lat_lon)

                # Exit this state after getting an acknowledgment with a result
                # equal to 1
                if msg_type == 'COMMAND_ACK':
                    nav_msg = msg.to_dict()
                    result = nav_msg['result']

                    if nav_msg['result'] == ack_result['CHECKING_REF_WP']:
                        print("    lat: {}, type: {}".format(lat,
                                                             type(lat)))
                        print("    lon: {}, type: {}".format(lon,
                                                             type(lon)))

                        wp_prev = wpq.getNext()
                        state = 'SENDING_PREV_WP'

            elif state == 'SENDING_PREV_WP':
                # Send the previous waypoint (not the reference) for the
                # linear trajectory tracking

                logger.send_mav_cmd_nav_waypoint(wp_prev)

                # Exit this state after getting an acknowledgment with a result
                # equal to 1
                if msg.get_type() == 'COMMAND_ACK':

                    nav_msg = msg.to_dict()
                    result = nav_msg['result']

                    if nav_msg['result'] == ack_result['CHECKING_PREV_WP']:
                        wp_prev_lla = np.array([[wp_prev[0, 0],  # lat
                                                 wp_prev[0, 1],  # lon
                                                 0.0]])          # alt
                        print("    wp_prev_lla = {}".format(wp_prev_lla))

                        wp_next = wpq.getNext()
                        state = 'SENDING_NEXT_WP'

            elif state == 'SENDING_NEXT_WP':
                # Send the next waypoint for the linear trajectory tracking

                logger.send_mav_cmd_nav_waypoint(wp_next)

                if msg.get_type() == 'COMMAND_ACK':

                    nav_msg = msg.to_dict()
                    result = nav_msg['result']

                    if nav_msg['result'] == ack_result['CHECKING_NEXT_WP']:
                        wp_next = wpq.getNext()
                        wp_next_lla = np.array([[wp_next[0, 0],  # lat
                                                 wp_next[0, 1],  # lon
                                                 0.0]])          # alt
                        print("    wp_next_lla = {}".format(wp_next_lla))

                        state = 'WAITING_TO_UPDATE_WPS'

            elif state == 'WAITING_TO_UPDATE_WPS':
                if msg_type == 'HIGHRES_IMU':
                    nav_msg = msg.to_dict()
                    xacc = nav_msg['xacc']
                    yacc = nav_msg['yacc']
                    zacc = nav_msg['zacc']
                    xmag = nav_msg['xmag']
                    ymag = nav_msg['ymag']
                    zmag = nav_msg['zmag']
                    xgyro = nav_msg['xgyro']
                    ygyro = nav_msg['ygyro']
                    zgyro = nav_msg['zgyro']

                if msg_type == 'ATTITUDE':
                    nav_msg = msg.to_dict()

                    yaw = nav_msg['yaw']
                    pitch = nav_msg['pitch']
                    roll = nav_msg['roll']

                if msg_type == 'GPS_RAW_INT':

                    nav_msg = msg.to_dict()

                    lon = nav_msg['lon']  # longitude
                    lat = nav_msg['lat']  # latitude

                    lon = float(lon) / 10000000.0
                    lat = float(lat) / 10000000.0

                    cog = nav_msg['cog']

                    print("**************************************************")
                    print("    cog: {}".format(cog))
                    print("    yaw (heading): {}, type: {}".format(yaw,
                                                                   type(yaw)))
                    print("    pitch: {}, type: {}".format(pitch, type(pitch)))
                    print("    roll: {}, type: {}".format(roll, type(roll)))
                    print("    xacc: {}".format(xacc))
                    print("    yacc: {}".format(yacc))
                    print("    zacc: {}".format(zacc))
                    print("    xmag: {}".format(xmag))
                    print("    ymag: {}".format(ymag))
                    print("    zmag: {}".format(zmag))
                    print("    xgyro: {}".format(xgyro))
                    print("    ygyro: {}".format(ygyro))
                    print("    zgyro: {}".format(zgyro))

                    vehi_pt_lla = np.array([[lat, lon, 0.0]])

                    vehi_pt_lla_copy = copy.deepcopy(vehi_pt_lla)
                    wp_next_lla_copy = copy.deepcopy(wp_next_lla)
                    print("    vehi_pt_lla_copy = {}".format(vehi_pt_lla_copy))
                    print("    wp_next_lla_copy = {}".format(wp_next_lla_copy))

                    vehi_pt_ned = LTP.lla2ned2(vehi_pt_lla_copy, wp_ref_lla)
                    wp_next_ned = LTP.lla2ned2(wp_next_lla_copy, wp_ref_lla)

                    print("    vehi_pt_lla_copy = {}".format(vehi_pt_lla_copy))
                    print("    wp_next_lla_copy = {}".format(wp_next_lla_copy))

                    vehi_pt_en = np.array([[vehi_pt_ned[0, 0],
                                            vehi_pt_ned[0, 1]]])
                    wp_next_en = np.array([[wp_next_ned[0, 1],
                                            wp_next_ned[0, 0]]])

                    print("    vehi_pt_en = {}".format(vehi_pt_en))
                    print("    wp_next_en = {}".format(wp_next_en))
                    print("    norm = {}".format(
                        np.linalg.norm(vehi_pt_en - wp_next_en)))

                    if wpq.isNearNext(vehi_pt_en):
                        state = 'SENDING_NEXT_WP'

            # Print the state transition
            if state != last_state:
                print("State: {} --> {}".format(last_state, state))
                last_state = state

            # END OF STATE MACHINE
            ##################################################################

            if msg.get_type() == 'HEARTBEAT':
                if mode_print_flag:
                    heartbeat_msg = msg.to_dict()
                    current_base_mode = heartbeat_msg['base_mode']

                    if current_base_mode != last_base_mode:
                        last_base_mode = current_base_mode

                        print("MAVLink base_mode changed: {}".format(
                            current_base_mode))

            if debug_flag:
                print("\r\nMsg:")
                print(msg)

                print("Time: {}".format(t_new))

            # Graphing
            if vizualize_attitude_flag:
                if ((msg_type == 'ATTITUDE') or (msg_type == 'HIGHRES_IMU')):
                    av.update(msg)

        ##################################################################
        # Log messages (at intervals)
        if (t_new - t_old) >= dt:
            t_old = t_new
            status = logger.log(msg)

            if echo_sensor:
                if status == 'GPS_RAW_INT':
                    for msg in msg_list:
                        logger.log(msg)

                    echo_sensor_time = int(t_new)

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

        # If the microcontroller indicates that we are in autonomous mode then
        # depending on vehicle position, update the next waypoint to travel to.
        # Else, the guidance system is not engaged
