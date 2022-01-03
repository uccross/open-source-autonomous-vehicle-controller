"""
.. module:: main_guide_sys.py
	:platform: MacOS, Unix, Windows,
	:synopsis: The main guidance system for a small autonomous boat
.. moduleauthor:: Pavlo Vlastos <pvlastos@ucsc.edu>
"""
from re import I
import numpy as np
import argparse
from mav_csv_logger import MAVCSVLogger as MCL
from path_planner import WaypointQueue as WQ
from utilities import LTPconvert as LTP
from utilities import AttitudeVisualization as AV
from utilities import Tracker as TR
from pymavlink import mavutil
import time
from signal import signal, SIGINT
from sys import exit
import copy
from utilities import Linear as LN

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

parser.add_argument('-s', '--simulation', dest='simulation_flag',
                    action='store_true', help='Flag to print mode changes')

parser.add_argument('-t', '--tracker', dest='tracker_flag',
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
simulation_flag = arguments.simulation_flag
tracker_flag = arguments.tracker_flag
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
    else:
        msg_list = []

    # Initialization
    extra_headers = ['vertical_fov', 'signal_quality',
                     'horizontal_fov', 'quaternion',
                     'data', 'reason', 'base_mode', 'mavlink_version',
                     'custom_mode', 'autopilot', 'system_status', 'type',
                     'vy', 'y', 'vx', 'vz', 'z', 'x', 'command', 'result',
                     'port', 'chan8_raw', 'chan7_raw', 'chan6_raw',
                     'chan5_raw', 'chan4_raw', 'chan3_raw', 'chan2_raw',
                     'chan1_raw', 'rssi', 'servo16_raw', 'servo15_raw',
                     'servo14_raw', 'servo13_raw', 'servo12_raw',
                     'servo11_raw', 'servo10_raw', 'servo9_raw', 'servo8_raw',
                     'servo7_raw', 'servo6_raw', 'servo5_raw', 'servo4_raw',
                     'servo3_raw', 'servo2_raw', 'servo1_raw']

    # Data logger
    logger = MCL.MAVCSVLogger(
        com, baudrate, log_file, csv_file, msg_list=msg_list,
        extra_headers=extra_headers, debug_flag=False)

    # Attitude Vizualizor
    if vizualize_attitude_flag:
        av = AV.AttitudeVizualizer(debugFlag=False, graphInterval=1, wt0=0,
                                   wtf=30, wdt=0.05)

    # Tracker
    if tracker_flag:
        tracker = TR.Tracker(graphInterval=1)
        trajectory = LN.Linear()

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

        # Close the csv file and mavlink connection
        logger.close_log()
        logger.close_mav_conn()

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
    t_transmit = time.time()
    t_HIL_transmit = time.time()
    t_sim = time.time()
    t_uc = time.time()
    t_graph = time.time()
    t_hard_write = time.time()
    t_info = time.time()

    dt_sim = 0.001  # seconds
    dt_uc = 0.01  # seconds
    dt_log = 0.05  # seconds
    dt_transmit = 1.0  # seconds
    dt_HIL_transmit = 1.0  # seconds
    dt_graph = 0.5
    dt_hard_write = 5.0  # seconds
    dt_info = 1.0  # seconds

    point_mass_state_vec = np.zeros((6, 1))
    orientation_state_vec = np.zeros((6, 1))

    reference_speed = 4000.00

    mass = 20.0  # kg
    radius = 1.5  # meters

    xacc = 0.0
    yacc = 0.0
    zacc = 0.0

    yaw = 0.0
    pitch = 0.0
    roll = 0.0
    yawspeed = 0.0
    pitchspeed = 0.0
    rollspeed = 0.0

    cf_heading_angle = 0.0
    path_angle = 0.0
    path_angle_checked = 0.0
    angle_diff = 0.0
    servo4_raw = 0
    input_angle = 0.0

    xacc = 0.0
    yacc = 0.0
    zacc = 0.0
    xmag = 0.0
    ymag = 0.0
    zmag = 0.0
    xgyro = 0.0
    ygyro = 0.0
    zgyro = 0.0

    heading_angle = 0.0

    cog = 0.0

    rad2deg = 180.0/np.pi

    # Waypoint Queue       N/S Lat   , E/W Long
    # waypoints = np.array([[36.9557439, -122.0604691], # Bad pond
    #                       [36.9556638, -122.0606960],
    #                       [36.9554362, -122.0607348],
    #                       [36.9556224, -122.0604107]])
    waypoints = np.array([[36.9836576, -122.0238656],  # Franklin street
                          [36.9835265, -122.0241790],
                          [36.9834655, -122.0241469]])
    wpq = WQ.WaypointQueue(waypoint_queue=waypoints, threshold=2.5)

    wp_ref_lla = np.array([[36.9836576, -122.0241886, 0.0]])
    wp_ref_lat_lon = np.array([[36.9836576, -122.0238656]])

    vehi_pt_lla = np.array([[0.0, 0.0, 0.0]])

    clst_pt_en = np.zeros((1, 2))

    wp_prev_lla = np.array([[0.0, 0.0, 0.0]])
    vehi_pt_lla = np.array([[0.0, 0.0, 0.0]])
    wp_next_lla = np.array([[0.0, 0.0, 0.0]])

    wp_prev_lla_copy = np.array([[0.0, 0.0, 0.0]])
    vehi_pt_lla_copy = np.array([[0.0, 0.0, 0.0]])
    wp_next_lla_copy = np.array([[0.0, 0.0, 0.0]])

    wp_prev_lla_copy2 = np.array([[0.0, 0.0, 0.0]])
    vehi_pt_lla_copy2 = np.array([[0.0, 0.0, 0.0]])
    wp_next_lla_copy2 = np.array([[0.0, 0.0, 0.0]])

    wp_prev_ned = LTP.lla2ned2(wp_prev_lla_copy, wp_ref_lla)
    vehi_pt_ned = LTP.lla2ned2(vehi_pt_lla_copy, wp_ref_lla)
    wp_next_ned = LTP.lla2ned2(wp_next_lla_copy, wp_ref_lla)

    wp_prev_en = np.array([[wp_prev_ned[0, 1], wp_prev_ned[0, 0]]])
    vehi_pt_en = np.array([[vehi_pt_ned[0, 1], vehi_pt_ned[0, 0]]])
    wp_next_en = np.array([[wp_next_ned[0, 1], wp_next_ned[0, 0]]])

    msg_type = None
    current_base_mode = -1

    ###########################################################################
    # State Machine Transitions
    last_base_mode = -1
    state = 'WAITING_FOR_REF_POINT'

    last_state = state

    found_ref_point = False

    ack_result = {'ERROR_WP': 0,
                  'FINDING_REF_WP': 1,
                  'SENDING_REF_WP': 2,
                  'WAITING_FOR_REF_WP': 3,
                  'CHECKING_REF_WP': 4,
                  'WAITING_FOR_PREV_WP': 5,
                  'CHECKING_PREV_WP': 6,
                  'WAITING_FOR_NEXT_WP': 7,
                  'CHECKING_NEXT_WP': 8,
                  'TRACKING_WP': 9}

    pic32_wp_state = 'FINDING_REF_WP'  # The Pic32's current waypoint state

    i_tx = 0

    ###########################################################################
    # Simulation
    if simulation_flag:
        from utilities import HIL_DummyVehicle

        angle_bound = 30 # angle bound for rudder in degrees

        Slug3 = HIL_DummyVehicle.DualModel(dt_sim, dt_uc, mass,
                                           point_mass_state_vec,
                                           orientation_state_vec,
                                           radius,
                                           reference_speed=reference_speed,
                                           min_angle=-angle_bound*np.pi/180,
                                           max_angle=angle_bound*np.pi/180)

        state = 'SENDING_REF_POINT'
        last_state = state

        pic32_wp_state = 'WAITING_FOR_REF_WP'

    ###########################################################################
    # Main Loop
    while True:

        # Timing
        t_new = time.time()

        if simulation_flag or found_ref_point:
            wp_prev_lla_copy = copy.deepcopy(wp_prev_lla)
            vehi_pt_lla_copy = copy.deepcopy(vehi_pt_lla)
            wp_next_lla_copy = copy.deepcopy(wp_next_lla)

            wp_prev_lla_copy2 = copy.deepcopy(wp_prev_lla)
            vehi_pt_lla_copy2 = copy.deepcopy(vehi_pt_lla)
            wp_next_lla_copy2 = copy.deepcopy(wp_next_lla)

            wp_prev_ned = LTP.lla2ned2(wp_prev_lla_copy, wp_ref_lla)
            vehi_pt_ned = LTP.lla2ned2(vehi_pt_lla_copy, wp_ref_lla)
            wp_next_ned = LTP.lla2ned2(wp_next_lla_copy, wp_ref_lla)

            wp_prev_en = np.array([[wp_prev_ned[0, 1],
                                    wp_prev_ned[0, 0]]])
            if not simulation_flag:
                vehi_pt_en = np.array([[vehi_pt_ned[0, 1],
                                        vehi_pt_ned[0, 0]]])

            wp_next_en = np.array([[wp_next_ned[0, 1],
                                    wp_next_ned[0, 0]]])

        # Read the state of the vehicle
        # Request MAVLINK_MSG_ID_RAW_IMU
        # Request MAVLINK_MSG_ID_ATTITUDE
        # Request LOCAL_POSITION_NED
        # Log the vehicle data
        msg = logger.mav_conn.recv_match()  # TODO: Make a getter() for this

        # Simulation vehicle state
        if simulation_flag and (state == 'WAITING_TO_UPDATE_WPS'):
            x_pm = Slug3.get_vehicle_point_state()
            x_os = Slug3.get_vehicle_orientation_state()

            if (t_new - t_HIL_transmit) >= dt_HIL_transmit:
                t_HIL_transmit = t_new

                # if i_tx == 0:
                #     logger.send_HIL_sensor(
                #         0.0,  # t_usec
                #         0.0,  # xacc
                #         0.0,  # yacc
                #         0.0,  # zacc
                #         x_os[0][0],  # xgyro --> actually sending roll angle
                #         x_os[1][0],  # ygyro --> actually sending pitch angle
                #         x_os[2][0],  # zgyro --> actually sending yaw angle
                #         0.0,  # xmag
                #         0.0,  # ymag
                #         0.0,  # zmag
                #     )

                yaw = x_os[2][0]
                yaw = (yaw + np.pi) % (2.0 * np.pi) - np.pi

                # Send GPS position of vehicle to be echoed back
                vehi_pt_en[0][0] = x_pm[0][0]
                vehi_pt_en[0][1] = x_pm[1][0]
                if i_tx == 1:
                    logger.send_HIL_GPS(vehi_pt_en, yaw)
                    i_tx = 0

                i_tx += 1

        # Check messages to update the state machine
        if msg:
            msg_type = msg.get_type()

            if ((msg_type != 'RC_CHANNELS_RAW') and
                    (msg_type != 'HIGHRES_IMU') and
                    (msg_type != 'GPS_RAW_INT') and
                    (msg_type != 'HEARTBEAT') and
                    (msg_type != 'ATTITUDE') and
                    (msg_type != 'COMMAND_ACK') and
                    (msg_type != 'SERVO_OUTPUT_RAW') and
                    (msg_type != 'BAD_DATA')):
                print("    msg.get_type() = {}".format(msg_type))

            ##################################################################
            # START OF STATE MACHINE

            ###################################################################
            if state == 'WAITING_FOR_REF_POINT':

                # If we get the following message type, echo back the reference
                # waypoint
                if msg_type == 'LOCAL_POSITION_NED':
                    nav_msg = msg.to_dict()

                    lon = nav_msg['x']  # longitude
                    lat = nav_msg['y']  # latitude

                    wp_ref_lat_lon = np.array([[lat, lon]])
                    wp_ref_lla = np.array([[lat, lon, 0.0]])

                    if (t_new - t_transmit) >= dt_transmit:
                        t_transmit = t_new
                        logger.send_mav_cmd_nav_waypoint(wp_ref_lat_lon)

                # Exit this state after getting an acknowledgment with a result
                # equal to 1
                if msg_type == 'COMMAND_ACK':
                    nav_msg = msg.to_dict()

                    if nav_msg['result'] == ack_result['CHECKING_REF_WP']:
                        print("    lat: {}, type: {}".format(lat,
                                                             type(lat)))
                        print("    lon: {}, type: {}".format(lon,
                                                             type(lon)))

                        wp_prev = wpq.getNext()
                        found_ref_point = True
                        state = 'SENDING_PREV_WP'

            ###################################################################
            elif state == 'SENDING_REF_POINT':
                # Send the reference ponit for the linear trajectory tracking

                if (t_new - t_transmit) >= dt_transmit:
                    t_transmit = t_new
                    logger.send_mav_cmd_nav_waypoint(wp_ref_lat_lon)

                # Exit this state after getting an acknowledgment with a result
                # equal to 1
                if msg.get_type() == 'COMMAND_ACK':

                    nav_msg = msg.to_dict()

                    if nav_msg['result'] == ack_result['CHECKING_REF_WP']:
                        wp_ref_lla = np.array([[wp_ref_lat_lon[0, 0],  # lat
                                                wp_ref_lat_lon[0, 1],  # lon
                                                0.0]])          # alt
                        print("    wp_ref_lla = {}".format(wp_ref_lla))

                        wp_prev = wpq.getNext()
                        state = 'SENDING_PREV_WP'

            ###################################################################
            elif state == 'SENDING_PREV_WP':
                # Send the previous waypoint (not the reference) for the
                # linear trajectory tracking

                if (t_new - t_transmit) >= dt_transmit:
                    t_transmit = t_new
                    logger.send_mav_cmd_nav_waypoint(wp_prev)

                # Exit this state after getting an acknowledgment with a result
                # equal to 1
                if msg.get_type() == 'COMMAND_ACK':

                    nav_msg = msg.to_dict()

                    if nav_msg['result'] == ack_result['CHECKING_PREV_WP']:
                        wp_prev_lla = np.array([[wp_prev[0, 0],  # lat
                                                 wp_prev[0, 1],  # lon
                                                 0.0]])          # alt
                        print("    wp_prev_lla = {}".format(wp_prev_lla))

                        wp_next = wpq.getNext()
                        state = 'SENDING_NEXT_WP'

            ###################################################################
            elif state == 'SENDING_NEXT_WP':
                # Send the next waypoint for the linear trajectory tracking

                if (t_new - t_transmit) >= dt_transmit:
                    t_transmit = t_new
                    logger.send_mav_cmd_nav_waypoint(wp_next)

                if msg.get_type() == 'COMMAND_ACK':

                    nav_msg = msg.to_dict()

                    if nav_msg['result'] == ack_result['CHECKING_NEXT_WP']:
                        wp_next_lla = np.array([[wp_next[0, 0],  # lat
                                                 wp_next[0, 1],  # lon
                                                 0.0]])          # alt
                        print("    wp_next_lla = {}".format(wp_next_lla))

                        state = 'WAITING_TO_UPDATE_WPS'

            ###################################################################
            elif state == 'WAITING_TO_UPDATE_WPS':

                if msg_type == 'SERVO_OUTPUT_RAW':
                    nav_msg = msg.to_dict()
                    servo4_raw = nav_msg['servo4_raw']

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

                    rollspeed = nav_msg['rollspeed']  # Using as path angle
                    # Using as cross track error
                    pitchspeed = nav_msg['pitchspeed']
                    yawspeed = nav_msg['yawspeed']  # Using as pic32_wp_state

                    if int(yawspeed) == 0:
                        pic32_wp_state = 'ERROR_WP'
                    if int(yawspeed) == 1:
                        pic32_wp_state = 'FINDING_REF_WP'
                    if int(yawspeed) == 2:
                        pic32_wp_state = 'SENDING_REF_WP'
                    if int(yawspeed) == 3:
                        pic32_wp_state = 'WAITING_FOR_REF_WP'
                    if int(yawspeed) == 4:
                        pic32_wp_state = 'CHECKING_REF_WP'
                    if int(yawspeed) == 5:
                        pic32_wp_state = 'WAITING_FOR_PREV_WP'
                    if int(yawspeed) == 6:
                        pic32_wp_state = 'CHECKING_PREV_WP'
                    if int(yawspeed) == 7:
                        pic32_wp_state = 'WAITING_FOR_NEXT_WP'
                    if int(yawspeed) == 8:
                        pic32_wp_state = 'CHECKING_NEXT_WP'
                    if int(yawspeed) == 9:
                        pic32_wp_state = 'TRACKING_WP'

                    if not simulation_flag:
                        cf_heading_angle = yaw*rad2deg
                        # if cf_heading_angle < 0.0:
                        #     cf_heading_angle = 360.0 + cf_heading_angle

                        # if simulation_flag:
                        #     cf_heading_angle = x_os[2][0]*rad2deg

                        path_angle = rollspeed*rad2deg
                        # if path_angle < 0.0:
                        #     path_angle = 360.0 + path_angle

                        angle_diff = pitchspeed*rad2deg
                        # if angle_diff < 0.0:
                        #     angle_diff = 360.0 + angle_diff
                    else:
                        cf_heading_angle = yaw*rad2deg
                        path_angle = rollspeed*rad2deg
                        angle_diff = pitchspeed*rad2deg

                if msg_type == 'GPS_RAW_INT':

                    nav_msg = msg.to_dict()

                    lon = nav_msg['lon']  # longitude
                    lat = nav_msg['lat']  # latitude

                    lon = float(lon) / 10000000.0
                    lat = float(lat) / 10000000.0

                    cog = nav_msg['cog']

                    vehi_pt_lla = np.array([[lat, lon, 0.0]])

                if wpq.isNearNext(clst_pt_en):
                    state = 'SENDING_PREV_WP'

                if simulation_flag:
                    if np.linalg.norm(clst_pt_en - wp_next_en) < wpq.threshold:
                        wp_prev = wp_next
                        state = 'SENDING_PREV_WP'

            ###################################################################
            # Print the state transition
            if state != last_state:
                print("State: {} --> {}".format(last_state, state))
                last_state = state

            # END OF STATE MACHINE
            ###################################################################

            if debug_flag:
                print("\r\nMsg:")
                print(msg)

                print("Time: {}".format(t_new))

            ###################################################################
            # Information
            if (t_new - t_info) >= dt_info:
                t_info = t_new

                if (msg_type == 'HEARTBEAT'):
                    heartbeat_msg = msg.to_dict()
                    current_base_mode = heartbeat_msg['base_mode']

                print("**************************************************")
                print("    mode:            {0:.6g}".format(current_base_mode))
                print("    cog:             {0:.6g}".format(cog))
                print("    CF heading:      {0:.6g}".format(cf_heading_angle))
                print("    path angle:      {0:.6g}".format(path_angle))
                print("    ^__checked:      {0:.6g}".format(
                    path_angle_checked))
                print("    angle_diff:      {0:.6g}".format(angle_diff))
                print("    servo4_raw:      {0:.6g}".format(servo4_raw))
                print("    input_angle:     {0:.6g}".format(input_angle))
                print("    yaw:             {0:.6g}".format(yaw*rad2deg))
                print("    pitch:           {0:.6g}".format(pitch*rad2deg))
                print("    roll:            {0:.6g}".format(roll*rad2deg))
                print("    xacc:            {0:.6g}".format(xacc))
                print("    yacc:            {0:.6g}".format(yacc))
                print("    zacc:            {0:.6g}".format(zacc))
                print("    xmag:            {0:.6g}".format(xmag))
                print("    ymag:            {0:.6g}".format(ymag))
                print("    zmag:            {0:.6g}".format(zmag))
                print("    xgyro:           {0:.6g}".format(xgyro*rad2deg))
                print("    ygyro:           {0:.6g}".format(ygyro*rad2deg))
                print("    zgyro:           {0:.6g}".format(zgyro*rad2deg))

                print("    wp_prev_lla_copy = {}".format(wp_prev_lla_copy))
                print("    vehi_pt_lla_copy = {}".format(vehi_pt_lla_copy))
                print("    wp_next_lla_copy = {}".format(wp_next_lla_copy))

                print("    wp_prev_lla_copy2 = {}".format(wp_prev_lla_copy2))
                print("    vehi_pt_lla_copy2 = {}".format(vehi_pt_lla_copy2))
                print("    wp_next_lla_copy2 = {}".format(wp_next_lla_copy2))

                print("    wp_prev_en = {}".format(wp_prev_en))
                print("    vehi_pt_en = {}".format(vehi_pt_en))
                print("    wp_next_en = {}".format(wp_next_en))

                print("    state:          {}".format(state))
                print("    pic32_wp_state: {}".format(pic32_wp_state))

                print("    norm = {}".format(
                    np.linalg.norm(vehi_pt_en - wp_next_en)))

                if current_base_mode != last_base_mode:
                    last_base_mode = current_base_mode

                    if mode_print_flag:
                        print("MAVLink base_mode changed: {}".format(
                            current_base_mode))

        #######################################################################
        # Simulation Update Vehicle
        if simulation_flag:
            if (t_new - t_sim) >= dt_sim:
                t_sim = t_new

                input_angle = servo4_raw-1500
                Slug3.update(input_angle)

        #######################################################################
        # Graphing
        if (t_new - t_graph) >= dt_graph:
            t_graph = t_new
            if (vizualize_attitude_flag and ((state == 'WAITING_TO_UPDATE_WPS')
                                             or state == 'SENDING_NEXT_WP')):
                if ((msg_type == 'ATTITUDE') or (msg_type == 'HIGHRES_IMU')):
                    if msg:
                        av.update(msg)

            if tracker_flag:
                ###############################################################
                # @TODO: Add closest point from micro?
                # Trajectory (might get rid of this)
                trajectory.setPreviousWaypoint(wp_prev_en)
                trajectory.setNextWaypoint(wp_next_en)

                trajectory.udpate(vehi_pt_en)

                path_angle_checked = trajectory.getPathAngle()
                path_angle_checked *= 180.0/np.pi
                # if path_anlge_checked < 0.0:
                #     path_anlge_checked = 360.0 + path_anlge_checked

                clst_pt_en = trajectory.getClosestPoint()

                tracker.update(yaw, pitch, roll, wp_prev_en=wp_prev_en,
                               wp_next_en=wp_next_en,
                               position_en=vehi_pt_en,
                               clst_pt_en=clst_pt_en)

        #######################################################################
        # Log messages (at intervals)
        if (t_new - t_old) >= dt_log:
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

        # Periodically close and re-open the file to ensure that the data was
        # written to the CSV file, this unfortunately very slow, especially
        # compared to the MAVLink .log file.
        # if (t_new - t_hard_write) >= dt_hard_write:
        #     t_hard_write = t_new
        #     logger.close_log()
        #     logger.open_log()

        # If the microcontroller indicates that we are in autonomous mode then
        # depending on vehicle position, update the next waypoint to travel to.
        # Else, the guidance system is not engaged
