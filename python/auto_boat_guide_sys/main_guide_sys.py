"""
.. module:: main_guide_sys.py
	:platform: MacOS, Unix, Windows,
	:synopsis: The main guidance system for a small autonomous boat
.. moduleauthor:: Pavlo Vlastos <pvlastos@ucsc.edu>
"""

import numpy as np
import argparse
from mav_csv_logger import MAVCSVLogger as MCL
from utilities import LTPconvert as LTP
from utilities import AttitudeVisualization as AV
from utilities import Tracker as TR
from Modules.Trajectories import Linear as LN
from Modules.Field import Field
from Modules.Utilities import Quaternions as QU
from Modules.Utilities import GaussianProcess as GP
from Modules.Utilities import Kriging as KG
from Modules.Utilities import Graph as Graph
from Modules.Utilities import PathPlanner as PATH
from pymavlink import mavutil
import time
from signal import signal, SIGINT
from sys import exit
import copy

###############################################################################
# Arbitrary default values
x0 = -35.0
xf = 35.0
y0 = -30.0
yf = 30.0
ds = 2.50

###############################################################################
# Parse Arguments
parser = argparse.ArgumentParser()
parser.add_argument('-b', '--baudrate', type=int, dest='baudrate',
                    default=115200, help='Specify baudrate for serial \
                        communication. Default is 115200')
parser.add_argument('-c', '--com', type=str, dest='com', default="COM4",
                    help='Specify COM port number for serial \
                    communication with micro. Default is 4, as in COM4')  # /dev/ttyUSB1
parser.add_argument('--dynamic_hyperparams', dest='dynamic_hyperparams',
                    action='store_true', help='Flag to approximate GPR/PGPR hyperparameters')
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
                        in COM3, x for CTE overload')  # /dev/ttyUSB0

parser.add_argument('--estimate_interval', dest='estimate_interval',
                    type=int, default=10,
                    help='Interval for estimating the field every n-new measurements')

parser.add_argument('--csv_file', type=str, dest='csv_file',
                    default='./log_file.csv',
                    help='log file path')

parser.add_argument('--kp', type=float, dest='kp',
                    default=1.0,
                    help='Proportional gain for waypoint tracking')

parser.add_argument('--kd', type=float, dest='kd',
                    default=1.0,
                    help='Derivative gain for waypoint tracking')

parser.add_argument('--log_file', type=str, dest='log_file',
                    default='./log_file.log',
                    help='log file path')

parser.add_argument('-m', '--mode', dest='mode_print_flag',
                    action='store_true', help='Flag to print mode changes')

parser.add_argument('--node_separation', dest='node_separation',
                    type=int, default=3,
                    help='Discrete field point node index separation')

parser.add_argument('--path_planner', dest='path_planner',
                    type=str, default="Zig-zag",
                    help='Path planner type: Zig-zag, Myopic, HV, HV-Bellman-Ford, Random')

parser.add_argument('-s', '--simulation', dest='simulation_flag',
                    action='store_true', help='Flag to print mode changes')

parser.add_argument('-t', '--tracker', dest='tracker_flag',
                    action='store_true', help='Flag to print mode changes')

parser.add_argument('--vizualize', dest='vizualize_attitude_flag',
                    action='store_true', help='Flag to print mode changes')

parser.add_argument('--ox0', type=float, dest='ox0',
                    default=0.0,
                    help='Grid shift in meters in the body-x axis')

parser.add_argument('--oy0', type=float, dest='oy0',
                    default=0.0,
                    help='Grid shift in meters in the body-y axis')

parser.add_argument('--omega_yaw', type=float, dest='omega_yaw',
                    default=0.0,
                    help='Yaw rate bias in radians per second')

parser.add_argument('--wp_angle', type=float, dest='wp_angle',
                    default=30.0*np.pi/180.0,
                    help='The orientation of the grid on the LTP in degrees')

parser.add_argument('--w_threshold', type=float, dest='threshold',
                    default=2.5,
                    help='The threshold distance that qualifies a waypoint\
                        transition')

parser.add_argument('-r', '--resolution', type=float, dest='resolution',
                    default=ds, help='The distance between discrete points')
                    
parser.add_argument('--max_recur', dest='max_recur',
                    type=int, default=2,
                    help='Maximum level of recursion for POK and PGPR')

parser.add_argument('--spatial_estimator', dest='spatial_estimator',
                    type=str, default="GPR",
                    help='Spatial estimator type: GPR, PGPR, OK, IIOK, or POK')

parser.add_argument('-v', '--vario_interval', dest='vario_interval',
                    type=int, default=50,
                    help='Record animation if animate flag')

parser.add_argument('--x0', type=float, dest='x0',
                    default=x0,
                    help='Minimum x0 coordinate (must be less than xf)')
parser.add_argument('--xf', type=float, dest='xf',
                    default=xf,
                    help='Minimum xf coordinate (must be more than x0)')
parser.add_argument('--y0', type=float, dest='y0',
                    default=y0,
                    help='Minimum y0 coordinate (must be less than yf)')
parser.add_argument('--yf', type=float, dest='yf',
                    default=yf,
                    help='Minimum yf coordinate (must be more than y0)')

arguments = parser.parse_args()

baudrate = arguments.baudrate
com = arguments.com
dynamic_hyperparams = arguments.dynamic_hyperparams
debug_flag = arguments.debug_flag
echo_sensor = arguments.echo_sensor
estimate_interval = arguments.estimate_interval
sensor_com = arguments.ecom
sensor_baudrate = arguments.ebaudrate
csv_file = arguments.csv_file
kp = arguments.kp
kd = arguments.kd
log_file = arguments.log_file
mode_print_flag = arguments.mode_print_flag
node_separation = arguments.node_separation
omega_yaw = arguments.omega_yaw
path_planner_type = arguments.path_planner
simulation_flag = arguments.simulation_flag
tracker_flag = arguments.tracker_flag
vizualize_attitude_flag = arguments.vizualize_attitude_flag

resolution = arguments.resolution
max_recur = arguments.max_recur
spatial_est_type = arguments.spatial_estimator
threshold = 2.5  # meters

vario_interval = arguments.vario_interval
x0 = arguments.x0
xf = arguments.xf
y0 = arguments.y0
yf = arguments.yf

ox0 = arguments.ox0
oy0 = arguments.oy0
wp_angle = arguments.wp_angle*np.pi/180.0

###############################################################################
if __name__ == '__main__':
    # Ping Echo Sounder for depth measurements in water
    if echo_sensor or (sensor_com == "x"):
        if sensor_com != "x":
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

    ###########################################################################
    # Blank field setup

    # Double check this vvvvvvvvvvvvvvvvv``

    field = Field.Field(ds=resolution, x0=x0+ox0, y0=y0+oy0, xf=xf+ox0, yf=yf+oy0,
                        alpha=-3.0, mMin=-10, mMax=0)

    # Double check this ^^^^^^^^^^^^^^^^^

    ###########################################################################
    # Position Setup
    starting_position = np.array([[x0, y0]])
    stopping_position = np.array([[xf, yf]])

    ###########################################################################
    # Path Planner
    if path_planner_type == "Zig-zag":
        path_planner = PATH.Zigzag(
            field=field,
            starting_position=starting_position,
            stopping_position=stopping_position,
            node_separation=node_separation)

    elif path_planner_type == "Myopic":
        path_planner = PATH.Myopic(
            field=field,
            look_int_dist=15,
            starting_position=starting_position,
            stopping_position=stopping_position)

    elif path_planner_type == "Random":
        path_planner = PATH.RandomizedWaypoints(field=field)

    elif path_planner_type == "HV":
        path_planner = PATH.HighestVariance(field=field)

    elif path_planner_type == "HV-Bellman-Ford":
        path_planner = PATH.BellmanFord(
            # Can be changed, but this works well.
            field=field, distance_weight=10.0,
            node_separation=node_separation, threshold=threshold,
            starting_position=starting_position,
            stopping_position=stopping_position)

    else:
        raise ValueError("main_guide_sys.py: Invalid path planner!")

    # Shift and rotate waypoints
    path_planner.move_wp(wp_angle, ox0, oy0)

    ###########################################################################
    # Spatiel Estimation Setup
    new_est_flag_internal = False

    sigma_w = 0.0

    est_update_interval = estimate_interval  # every 'N' measurements
    vario_update_interval = vario_interval  # every 'N' measurements
    est_updt_meas_cnt = 0

    # Number of measurements and estimate update counter
    est_updt_meas_cnt_last = 0
    vario_updt_meas_cnt_last = 0
    Zhat = np.zeros((field.get_yLen(), field.get_xLen()))
    V_Zhat = np.ones((field.get_yLen(), field.get_xLen())) * \
        100  # Make sure it's non-zero

    max_recursion_level = max_recur
    min_recursion_level = 1

    l_max = 1  # Default max level of recursion

    # Variogram stuff
    maxlag = 50
    lagsize = 0.25
    lagvec = np.arange(0, maxlag, lagsize)
    lagvec = np.reshape(lagvec, (len(lagvec), 1))
    nLags = len(lagvec)
    E = np.zeros((nLags,1))


    k_sill=1.0
    k_rng=100.0

    # Gaussian Process Regression and Partitioned Gaussian Process
    # Regression
    if (spatial_est_type == "GPR") or (spatial_est_type == "PGPR"):

        dynamic_hyperparams = dynamic_hyperparams

        spatial_estimator = GP.TwoDimensional(m_field=field.get_yLen(),
                                                    n_field=field.get_xLen(),
                                                    sigma_f=1.0,
                                                    ell=5.5,
                                                    sigma_w=0.1,
                                                    rlvl=max_recursion_level,
                                                    type=spatial_est_type)

        if dynamic_hyperparams or (spatial_est_type == "PGPR"):
            x = starting_position[0][0]
            y = starting_position[0][1]
            kriging_zxy = np.array([[0.0, x, y]])
            variogrammer = KG.OrdinaryKriging(field=field,
                                                    tol=field.ds, 
                                                    nLags=nLags)

    # Ordinary Kriging and Partitioned Ordinary Kriging
    elif ((spatial_est_type == "OK") or (spatial_est_type == "IIOK") or
            (spatial_est_type == "POK")):

        dynamic_hyperparams = False  # This only applies to GPR/PGPR

        x = starting_position[0][0]
        y = starting_position[0][1]
        kriging_zxy = np.array([[0.0, x, y]])

        if (spatial_est_type == "OK"):
            spatial_estimator = KG.OrdinaryKriging(field=field,
                                                        tol=field.ds,
                                                        nLags=nLags)
        elif (spatial_est_type == "IIOK"):
            spatial_estimator = KG.OrdinaryKriging(field=field,
                                                        tol=field.ds,
                                                        nLags=nLags)

        elif (spatial_est_type == "POK"):
            spatial_estimator = KG.PartitionedKriging(
                nLags=nLags,
                tol=field.ds,
                ds=field.ds,
                xMag=field.get_width(),
                yMag=field.get_length(),
                Zhat=np.zeros(field.getZ().shape),
                Vhat=np.ones(field.getZ().shape)*1000,
                rlvl=max_recursion_level, x0=field.get_x0(),
                mMax=field.get_max(),
                mMin=field.get_min(),
                y0=field.get_y0(), xf=field.get_xf(), yf=field.get_yf())

    else:
        print("**** No valid spatial estimator specified")

    [iqy, iqx] = field.getTrueFieldIndices(starting_position[0][1],
                                           starting_position[0][0])

    trajectory = LN.Linear(vehiPT=starting_position,
                           prevWP=starting_position,
                           nextWP=path_planner.get_wp_next_en(
                               iqy, iqx,
                               V_Zhat,
                               new_est_flag_internal,
                               heading_angle=0.0))

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
                     'servo3_raw', 'servo2_raw', 'servo1_raw',
                     'roll', 'yaw', 'pitch', 'rollspeed', 'pitchspeed',
                     'yawspeed', 'cog', 'fix_type', 'lat', 'alt_ellipsoid',
                     'eph', 'alt', 'h_acc', 'satellites_visible', 'hdg_acc',
                     'v_acc', 'epv', 'vel', 'lon', 'vel_acc']

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
        tracker = TR.Tracker(graphInterval=1,
                             wp_grid=path_planner.get_wp_queue(),
                             x_bound=field.get_width()*1.10,
                             y_bound=field.get_length()*1.10)

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
    t_prev_update = time.time()
    t_next_update = time.time()
    t_HIL_transmit = time.time()
    t_trajectory = time.time()
    t_sim = time.time()
    t_uc = time.time()
    t_graph = time.time()
    t_hard_write = time.time()
    t_info = time.time()

    dt_sim = 0.001  # seconds
    dt_uc = 0.01  # seconds
    dt_log = 0.001  # seconds
    dt_transmit = 0.500  # seconds
    dt_update = 4.000  # seconds
    dt_HIL_transmit = 0.5  # seconds
    dt_trajectory = 0.5  # seconds
    dt_graph = 0.5
    dt_hard_write = 5.0  # seconds
    dt_info = 1.0  # seconds

    point_mass_state_vec = np.zeros((6, 1))
    orientation_state_vec = np.zeros((6, 1))

    reference_speed = 2000.00

    mass = 7.5  # kg
    radius = 0.5  # meters

    lon = 0
    lat = 0

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
    delta_angle = 0.0
    tvc_angle = 0.0
    cte = 0.0

    xacc = 0.0
    yacc = 0.0
    zacc = 0.0
    xmag = 0.0
    ymag = 0.0
    zmag = 0.0
    xgyro = 0.0
    ygyro = 0.0
    zgyro = 0.0

    # Echo sounder depth sensor
    echo_sensor_distance = 0
    echo_confidence = 0

    heading_angle = 0.0

    cog = 0.0

    rad2deg = 180.0/np.pi
    deg2rad = np.pi/180.0

    wp_ref_lla = np.array([[0.0, 0.0, 0.0]])
    wp_ref_lat_lon = np.array([[0.0, 0.0]])

    vehi_pt_lla = np.array([[0.0, 0.0, 0.0]])

    clst_pt_en = np.zeros((1, 2))

    vehi_pt_lla = np.array([[0.0, 0.0, 0.0]])
    wp_next_lla = np.array([[0.0, 0.0, 0.0]])

    wp_ref_lla_copy = np.array([[0.0, 0.0, 0.0]])
    vehi_pt_lla_copy = np.array([[0.0, 0.0, 0.0]])
    # wp_next_lla_copy = np.array([[0.0, 0.0, 0.0]])

    vehi_pt_lla_copy2 = np.array([[0.0, 0.0, 0.0]])
    # wp_next_lla_copy2 = np.array([[0.0, 0.0, 0.0]])

    vehi_pt_ned = LTP.lla2ned2(np.array([[0.0, 0.0, 0.0]]), wp_ref_lla)
    wp_next_ned = LTP.lla2ned2(np.array([[0.0, 0.0, 0.0]]), wp_ref_lla)

    wp_prev_en = np.zeros((1, 2))
    vehi_pt_en = np.zeros((1, 2))
    uc_vehi_en = np.zeros((1, 2))
    wp_next_en = path_planner.get_wp_next_en()
    uc_prev_en = np.zeros((1, 2))
    uc_next_en = np.zeros((1, 2))

    is_first_gps = True

    msg_type = None
    current_base_mode = -1

    ###########################################################################
    # State Machine Transitions
    last_base_mode = -1
    state = 'TRACKING'

    last_state = state

    found_ref_point = False

    ack_result = {'ERROR_WP': 0,
                  'FINDING_REF_WP': 1,
                  'SENDING_REF_WP': 2,
                  'CHECKING_REF_WP': 3,
                  'WAITING_FOR_NEXT_WP': 4}

    pic32_wp_state = 'FINDING_REF_WP'  # The Pic32's current waypoint state

    i_tx = 0
    prev_or_next_tx = 0.0
    prev_next_vehi = 0.0
    check0 = 0.0
    check1 = 0.0
    check2 = 0.0
    tolerance = 0.00001

    # was_waypoint_flag = False

    ###########################################################################
    # Simulation
    if simulation_flag:
        from utilities import HIL_DummyVehicle

        Slug3 = HIL_DummyVehicle.DualModel(dt_sim, dt_uc, mass,
                                           point_mass_state_vec,
                                           orientation_state_vec,
                                           radius,
                                           reference_speed=0.0)

    ###########################################################################
    # Main Loop
    while True:

        # was_waypoint_flag = False

        # Timing
        t_new = time.time()

        # Trajectory
        if (t_new - t_trajectory) >= dt_trajectory:
            t_trajectory = t_new

            trajectory.udpate(vehi_pt_en)

            path_angle_checked = trajectory.getPathAngle()
            path_angle_checked *= 180.0/np.pi
            # if path_anlge_checked < 0.0:
            #     path_anlge_checked = 360.0 + path_anlge_checked

            clst_pt_en = trajectory.getClosestPoint()

            cte = trajectory.getCte()

        # Read the state of the vehicle
        # Request MAVLINK_MSG_ID_RAW_IMU
        # Request MAVLINK_MSG_ID_ATTITUDE
        # Request LOCAL_POSITION_NED
        # Log the vehicle data
        msg = logger.mav_conn.recv_match()  # TODO: Make a getter() for this

        # Simulation vehicle state
        if simulation_flag:

            x_pm = Slug3.get_vehicle_point_state()
            x_os = Slug3.get_vehicle_orientation_state()

            if (t_new - t_HIL_transmit) >= dt_HIL_transmit:
                t_HIL_transmit = t_new

                wp_yaw = x_os[2][0]
                wp_yaw = (wp_yaw + np.pi) % (2.0 * np.pi) - np.pi

                # Send GPS position of vehicle to be echoed back
                vehi_pt_en[0][0] = x_pm[0][0]
                vehi_pt_en[0][1] = x_pm[1][0]

                logger.send_mav_ltp_HIL_en(vehi_pt_en, wp_yaw)
                i_tx = 0

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
                    (msg_type != 'LOCAL_POSITION_NED') and
                    (msg_type != 'BAD_DATA')):
                print("    msg.get_type() = {}".format(msg_type))

            ###################################################################
            if msg_type == 'ATTITUDE':
                nav_msg = msg.to_dict()

                yaw = nav_msg['yaw']  # Already in degrees
                pitch = nav_msg['pitch']  # radians
                roll = nav_msg['roll']  # radians

                rollspeed = nav_msg['rollspeed']  # Using as path angle
                # Using as cross track error
                # Using as rudder angle command
                pitchspeed = nav_msg['pitchspeed']
                yawspeed = nav_msg['yawspeed']  # Using as pic32_wp_state

                if int(yawspeed) == 0:
                    pic32_wp_state = 'ERROR_WP'
                if int(yawspeed) == 1:
                    pic32_wp_state = 'FINDING_REF_WP'
                if int(yawspeed) == 2:
                    pic32_wp_state = 'SENDING_PREV'
                if int(yawspeed) == 3:
                    pic32_wp_state = 'SENDING_NEXT'
                if int(yawspeed) == 4:
                    pic32_wp_state = 'WAITING_FOR_NEXT_WP'
                if int(yawspeed) == 5:
                    pic32_wp_state = 'TRACKING'

                cf_heading_angle = yaw  # Already in degrees
                path_angle = rollspeed*rad2deg
                angle_diff = pitchspeed*rad2deg

                delta_angle = pitchspeed

            ###################################################################
            if msg_type == 'LOCAL_POSITION_NED':
                nav_msg = msg.to_dict()

                e = nav_msg['x']  # East
                n = nav_msg['y']  # North
                prev_next_vehi = nav_msg['z']  #

                check0 = nav_msg['vx']  # using vx as a check value
                check1 = nav_msg['vy']  # using vy as a check value
                check2 = nav_msg['vz']  # using vz as a check value

                if ((np.abs(check0 - 0.2) <= tolerance) and
                    (np.abs(check1 - 0.4) <= tolerance) and
                        (np.abs(check2 - 0.6) <= tolerance)):

                    if (prev_next_vehi-1.0) <= tolerance:
                        # was_waypoint_flag = True
                        uc_prev_en = np.array([[e, n]])

                    elif (prev_next_vehi-1.5) <= tolerance:
                        # was_waypoint_flag = True
                        uc_next_en = np.array([[e, n]])

                    elif (prev_next_vehi-2.0) <= tolerance:
                        uc_vehi_en = np.array([[e, n]])

            ##################################################################
            # START OF STATE MACHINE
            if state == 'UPDATING_PREV':
                # Send the previous waypoint for the linear trajectory tracking
                if (t_new - t_transmit) >= dt_transmit:
                    t_transmit = t_new
                    prev_or_next_tx = 1.0
                    logger.send_mav_ltp_en_waypoint(wp_prev_en,
                                                    omega_yaw,
                                                    kp, kd,
                                                    prev_or_next_tx)

                if (np.linalg.norm(uc_prev_en-wp_prev_en) <= tolerance):
                    state = 'TRACKING'

            ##################################################################
            elif state == 'UPDATING_NEXT':
                # Send the next waypoint for the linear trajectory tracking
                if (t_new - t_transmit) >= dt_transmit:
                    t_transmit = t_new
                    prev_or_next_tx = 1.5
                    logger.send_mav_ltp_en_waypoint(wp_next_en,
                                                    omega_yaw,
                                                    kp, kd,
                                                    prev_or_next_tx)

                if (np.linalg.norm(uc_next_en-wp_next_en) <= tolerance):
                    if simulation_flag:
                        Slug3.set_reference_speed(reference_speed)
                    state = 'TRACKING'

            ##################################################################
            elif state == 'TRACKING':
                
                ##############################################################
                # EXIT CASE: PREVIOUS
                if (np.linalg.norm(uc_prev_en-wp_prev_en) > tolerance):
                    state = 'UPDATING_PREV'

                if (t_new - t_prev_update) >= dt_update:
                    t_prev_update = t_new
                    state = 'UPDATING_PREV'

                ##############################################################
                # EXIT CASE: NEXT
                if (np.linalg.norm(uc_next_en-wp_next_en) > tolerance):
                    state = 'UPDATING_NEXT'

                ##############################################################
                if msg_type == 'SERVO_OUTPUT_RAW':
                    nav_msg = msg.to_dict()
                    servo4_raw = nav_msg['servo4_raw']

                ##############################################################
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

                ##############################################################
                if msg_type == 'GPS_RAW_INT':

                    nav_msg = msg.to_dict()

                    lon = nav_msg['lon']  # longitude
                    lat = nav_msg['lat']  # latitude

                    lon = float(lon) / 10000000.0
                    lat = float(lat) / 10000000.0

                    cog = nav_msg['cog']

                    if is_first_gps:
                        wp_ref_lla = np.array([[lat, lon, 0.0]])
                        is_first_gps = False

                    vehi_pt_lla = np.array([[lat, lon, 0.0]])

                    vehi_pt_lla_copy = copy.deepcopy(vehi_pt_lla)
                    vehi_pt_lla_copy2 = copy.deepcopy(vehi_pt_lla)

                    vehi_pt_ned = LTP.lla2ned2(vehi_pt_lla, wp_ref_lla)
                    vehi_pt_en[0][0] = vehi_pt_ned[0][1]  # East
                    vehi_pt_en[0][1] = vehi_pt_ned[0][0]  # North

                if (trajectory.is_closest_point_near_next_wp(threshold)):
                    # or (trajectory.is_closest_point_beyond_next(
                    #     threshold))):
                    wp_next_en_old = trajectory.getNextWaypoint()

                    ######################################################
                    ######################################################
                    ######################################################

                    # wp_next_en = path_planner.get_wp_next_en()
                    wp_next_en = path_planner.get_wp_next_en(
                        iqy, iqx,
                        V_Zhat,
                        new_est_flag_internal,
                        heading_angle=yaw*deg2rad)

                    new_est_flag_internal = False
                    
                    trajectory.setNextWaypoint(wp_next_en)

                    if (np.linalg.norm(wp_next_en-wp_next_en_old) > 1.0):
                        wp_prev_en = wp_next_en
                        trajectory.setPreviousWaypoint(wp_next_en_old)
                    
                    # state = 'SENDING_NEXT_WP'

            # ###################################################################
            # # Print the state transition
            # if state != last_state:
            #     print("State: {} --> {}".format(last_state, state))
            #     last_state = state

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
                # print("    angle_diff:      {0:.6g}".format(angle_diff))
                print("    cte:             {0:.6g}".format(cte))
                print("    servo4_raw:      {0:.6g}".format(servo4_raw))
                print("    delta_angle:     {0:.6g}".format(delta_angle))
                print("    tvc_angle:       {0:.6g}".format(tvc_angle*rad2deg))
                print("    yaw:             {0:.6g}".format(yaw))
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

                print("    depth:           {}".format(echo_sensor_distance))
                print("    depth cnfdnc:    {}".format(echo_confidence))

                print("    vehi_pt_lla = {}".format(vehi_pt_lla))
                print("    wp_next_lla = {}".format(wp_next_lla))

                print("    vehi_pt_lla_copy = {}".format(vehi_pt_lla_copy))
                print("    vehi_pt_lla_copy2= {}".format(vehi_pt_lla_copy2))
                # print("    wp_next_lla_copy = {}".format(wp_next_lla_copy))

                # print("    check0 =     {}".format(check0))
                # print("    check1 =     {}".format(check1))
                # print("    check2 =     {}".format(check2))

                print("    prevNextTx    = {}".format(prev_or_next_tx))
                print("    prev_next_vehi = {}".format(prev_next_vehi))

                print("    wp_prev_en    = {}".format(wp_prev_en))
                print("    uc_prev_en    = {}".format(uc_prev_en))

                print("    vehi_pt_en    = {}".format(vehi_pt_en))
                print("    uc_vehi_en    = {}".format(uc_vehi_en))

                print("    wp_next_en    = {}".format(wp_next_en))
                print("    uc_next_en    = {}".format(uc_next_en))

                print("    state:          {}".format(state))
                print("    pic32_wp_state: {}".format(pic32_wp_state))

                print("    norm = {}".format(
                    np.linalg.norm(vehi_pt_en - wp_next_en)))

                if current_base_mode != last_base_mode:
                    last_base_mode = current_base_mode

                    if mode_print_flag:
                        print("MAVLink base_mode changed: {}".format(
                            current_base_mode))

        ###################################################################
        # Can a new field measurement be taken?
        y = vehi_pt_en[0][1]
        x = vehi_pt_en[0][0]
        [iqy, iqx] = field.getTrueFieldIndices(y, x)
        # [iqy, iqx] = field.quantizePosition(y, x)
        q = [iqy, iqx]

        # Gaussian Process Regression and Partitioned Gaussian Process
        # Regression
        if ((spatial_est_type == "GPR") or
                (spatial_est_type == "PGPR")):

            if not (q in spatial_estimator.get_observed_yxs().tolist()):
                spatial_estimator.update_observed_yxs(iqy, iqx)

                spatial_estimator.update_observed_zs(echo_sensor_distance)

                est_updt_meas_cnt += 1

            # If we are using dynamic hyperparameters based on the
            # variogram
            if (dynamic_hyperparams or
                (spatial_est_type == "PGPR")):

                qy, qx = field.quantizePosition(y, x)
                qxy = [qx, qy]

                if not (qxy in kriging_zxy[:, 1:].tolist()):
                    z = field.getTrueFieldValue(y, x)
                    z += sigma_w * np.random.randn()

                    zxy = np.array([[z, qx, qy]])

                    kriging_zxy = np.concatenate(
                        (kriging_zxy, zxy), axis=0)

        # Ordinary Kriging, Iterative Inverse Ordinary Kriging and
        # Partitioned Ordinary Kriging
        elif ((spatial_est_type == "OK") or
                (spatial_est_type == "IIOK") or
                (spatial_est_type == "POK")):

            qy, qx = field.quantizePosition(y, x)
            qxy = [qx, qy]

            if not (qxy in kriging_zxy[:, 1:].tolist()):
                z = field.getTrueFieldValue(y, x)
                z += sigma_w * np.random.randn()

                zxy = np.array([[z, qx, qy]])

                kriging_zxy = np.concatenate(
                    (kriging_zxy, zxy), axis=0)

                est_updt_meas_cnt += 1


        ###################################################################
        # Is it time to update the empirical variogram
        # Currently setup to update once, but can be changed for multiple
        # updates. Keep in mind that it is computationally expensive to
        # update the empirical variogram
        if ((np.mod(est_updt_meas_cnt, vario_update_interval) == 0)
                and (vario_updt_meas_cnt_last != est_updt_meas_cnt)
                and (empirical_formed == False)):

            # Prevent repeat variograms within a single interval
            vario_updt_meas_cnt_last = est_updt_meas_cnt

            ###############################################################
            # GPR general cases
            if ((spatial_est_type == "GPR") or
                    (spatial_est_type == "PGPR")):
                    
                if (dynamic_hyperparams or
                        (spatial_est_type == "PGPR")):

                    E = variogrammer.makeEmpericalSemiVariogram(
                        kriging_zxy, lag_vec=lagvec)

                    sill, _, _ = variogrammer.fit2EmpiricalScipy(
                        lagvec[:len(E)], E)

                    f = variogrammer.variogramGauss(lagvec)
                    rng = 1
                    for lag, fl in zip(lagvec, f):
                        if fl >= 0.95*sill:
                            rng = lag[0]
                            break

                    # Set the hyperparameters:
                    # 1) the common length scale
                    # 2) the variance scale factor
                    spatial_estimator.set_ell(k_rng*rng)
                    spatial_estimator.set_sigma_f(k_sill*sill)
                    


            ###############################################################
            # OK, IIOK, and POK variogram cases
            if ((spatial_est_type == "OK") or
                    (spatial_est_type == "IIOK") or
                    (spatial_est_type == "POK")):

                # Global variogram for whole field
                if ((spatial_est_type == "OK") or
                        (spatial_est_type == "IIOK")):

                    # print(kriging_zxy)

                    E = spatial_estimator.makeEmpericalSemiVariogram(
                        kriging_zxy, lag_vec=lagvec)

                    # print(E)

                elif (spatial_est_type == "POK"):
                    E = spatial_estimator.makeEmpericalSemiVariogram(
                        sensorLog=kriging_zxy,
                        endlevel=1)

                # Fit the global empirical variogram
                sill, _, _ = spatial_estimator.fit2EmpiricalScipy(
                    lagvec[:len(E)], E)

            ###############################################################
            # Special Cases for recursive partition of the field into
            # subfields for PGPR and POK
            if ((spatial_est_type == "PGPR") or
                    (spatial_est_type == "POK")):

                if (spatial_est_type == "PGPR"):
                    f = variogrammer.variogramGauss(lagvec)

                elif (spatial_est_type == "POK"):
                    f = spatial_estimator.variogramGauss(lagvec)

                rng = 1
                for lag, fl in zip(lagvec, f):
                    if fl >= 0.95*sill:
                        rng = lag[0]
                        break

                qx = field.get_width()
                qy = field.get_length()

                l_max = (int(np.floor(np.sqrt((qx**2 + qy**2))/(rng))) - 1)

                l_max = np.min([max_recursion_level, l_max])


                if l_max < min_recursion_level:
                    l_max = min_recursion_level

                if l_max > max_recursion_level:
                    l_max = max_recursion_level


                # Recursion level cases for PGPR and POK

                if (spatial_est_type == "PGPR"):
                    # Set the hyperparameters:
                    # 1) the common length scale
                    # 2) the variance scale factor
                    spatial_estimator.set_ell(k_rng*rng)
                    spatial_estimator.set_sigma_f(k_sill*sill)
                    spatial_estimator.set_rlvl(l_max)

                elif (spatial_est_type == "POK"):

                    # Local variogram from immediate sub-field
                    E = spatial_estimator.makeEmpericalSemiVariogram(
                        sensorLog=kriging_zxy,
                        endlevel=l_max)

                    # Fit the local variogram from immediate sub-field
                    spatial_estimator.fit2EmpiricalScipy(
                        lagvec[:len(E)], E)

            #
            empirical_formed = True

        ###################################################################
        # Is it time to update the spatial estimate?
        if ((np.mod(est_updt_meas_cnt, est_update_interval) == 0)
                and (est_updt_meas_cnt_last != est_updt_meas_cnt)
                and empirical_formed):

            # Prevent repeat estimates within a single interval
            est_updt_meas_cnt_last = est_updt_meas_cnt

            if est_updt_meas_cnt > 0:

                # Gaussian Process Regression and Partitioned Gaussian
                # Process Regression
                if ((spatial_est_type == "GPR") or
                        (spatial_est_type == "PGPR")):
                    spatial_estimator.find_unobserved_yxs()

                    Zhat, V_Zhat = spatial_estimator.predict()

                # Ordinary Kriging and Partitioned Ordinary Kriging
                if ((spatial_est_type == "OK") or
                    (spatial_est_type == "IIOK") or
                        (spatial_est_type == "POK")):

                    # Ordinary Kriging (OK)
                    if spatial_est_type == "OK":

                        C = spatial_estimator.calcCovMatFromVario(
                            sensorLog=kriging_zxy)

                        # Cinv = spatial_estimator.getInvCovarianceMatrix()

                        Zhat, V_Zhat = spatial_estimator.predict(
                            C, sensorLog=kriging_zxy)

                    # Iterative Inverse Ordinary Kriging (IIOK)
                    elif (spatial_est_type == "IIOK"):

                        # maxlag = 200
                        # lagsize = 1
                        # lagvec = np.arange(0, maxlag, lagsize)
                        # lagvec = np.reshape(lagvec, (len(lagvec), 1))

                        Cinv = spatial_estimator.getInvCovarianceMatrix()

                        C = spatial_estimator.calcCovMatFromVario(
                            sensorLog=kriging_zxy)

                        # # Fit the local variogram from immediate sub-field
                        # spatial_estimator.fit2EmpiricalScipy(
                        #     lagvec[:len(E)], E)

                        Zhat, V_Zhat = spatial_estimator.predict1Step(
                            C=C, Cinv=Cinv, sensorLog=kriging_zxy,
                            field=field)

                        # print("Zhat:\n{}".format(Zhat))

                    # Partitioned Ordinary Kriging (POK)
                    elif (spatial_est_type == "POK"):

                        C = spatial_estimator.calcCovMatFromVario(
                            kriging_zxy, endlevel=l_max)

                        # Cinv = spatial_estimator.getInvCovarianceMatrix()

                        Zhat, V_Zhat = spatial_estimator.predict(
                            C=C,
                            sensorLog=kriging_zxy,
                            endlevel=l_max,
                            x_field=field.get_x_vector(),
                            y_field=field.get_y_vector())

                    ######################################################
                    ######################################################
                    ######################################################

                # new_est_flag = True
                new_est_flag_internal = True

        #######################################################################
        # Simulation Update Vehicle
        if simulation_flag:
            if (t_new - t_sim) >= dt_sim:
                t_sim = t_new

                Slug3.update(delta_angle)
                tvc_angle = Slug3.get_tvc_angle()

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

                yaw_g = (yaw*deg2rad) - (np.pi / 2.0)

                tracker.update(yaw_g, -pitch, -roll, wp_prev_en=wp_prev_en,
                               wp_next_en=wp_next_en,
                               position_en=uc_vehi_en,
                               clst_pt_en=clst_pt_en,
                               new_grid_pts=path_planner.get_wp_queue())

        #######################################################################
        # Log messages (at intervals)
        if (t_new - t_old) >= dt_log:
            t_old = t_new

            # if was_waypoint_flag == False:
            status = logger.log(msg)

            if echo_sensor or (sensor_com == "x"):
                if status == 'GPS_RAW_INT':  # log depth if we just got a GPS MAVLink message

                    echo_sensor_time = int(t_new)  # Time in milliseconds

                    if sensor_com != "x":
                        echo_data = myPing.get_distance()
                        echo_sensor_distance = echo_data["distance"]
                        echo_confidence = echo_data["confidence"]

                    # using orentation for recording cross track error for simple test
                    echo_sensor_orientation = int(cte*10)
                    if echo_sensor_orientation > 255:
                        echo_sensor_orientation = 255

                    echo_msg = mavutil.mavlink.MAVLink_distance_sensor_message(
                        echo_sensor_time,
                        echo_sensor_min,
                        echo_sensor_max,
                        echo_sensor_distance,
                        echo_sensor_type,
                        echo_sensor_id,
                        echo_sensor_orientation,
                        echo_confidence)

                msg_list = [echo_msg]

                # Written this way in case more python-side MAVLink messages
                # are to be added
                for msg in msg_list:
                    logger.log(msg)

        msg = None

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
