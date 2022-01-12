"""
.. module:: Constants.py
    :platform: MacOS, Unix, Windows,
    :synopsis: Useful default constants for the simulation
.. moduleauthor:: Pavlo Vlastos <pvlastos@ucsc.edu>
"""
import numpy as np

###############################################################################
# Timing
dt_sim = 0.01  # simulation time step in seconds
dt_uc = 0.1  # computation time step. @NOTE: should always be more than dt_sim

###############################################################################
# Spatial 
ds = 0.25   # Spatial separation between field measurements [meters]
x0 = -48.0    # The x-coordinate of the start of the gradient [meters]
y0 = -30.0    # The y-coordinate of the start of the gradient [meters]
xf = 48.0   # The x-coordinate of the end of the gradient [meters]
yf = 30.0   # The y-coordinate of the end of the gradient [meters]
x0 = np.round(x0/ds)*ds 	# Make sure it is a multiple of ds, to simply
# finding invidiual points
xf = np.round(xf/ds)*ds 	# Make sure it is a multiple of ds, to simply
# finding invidiual points
y0 = np.round(y0/ds)*ds 	# Make sure it is a multiple of ds, to simply
# finding invidiual points
yf = np.round(yf/ds)*ds 	# Make sure it is a multiple of ds, to simply
# finding invidiual points
x = np.arange(x0, xf+ds, ds)  # Vector of x coordinates
y = np.arange(y0, yf+ds, ds)  # Vector of y coordinates
x0 = x[0]
xf = x[-1]
y0 = y[0]
yf = y[-1]

totalNumPointsInField = len(x)*len(y)

xMagnitude = xf - x0
yMagnitude = yf - y0
hypotenuse = np.sqrt(xMagnitude**2 + yMagnitude**2)
xLen = len(x)
yLen = len(y)
kSize = 10  # The width and hieght dimension such that K is an [m x m] matrix
kSigma = 2.0    # The standard deviation
kN = 1          # The number of times to convolve the kernel
mMax = 10.0     # The maximum possible measurement of the field
mMin = 2.0      # The minimum possible measurement of the field

muNoise = 0.0  # The mean of the normal distrubtion for field measurement noise
# The standard deviation of the normal distrubtion for field measurement noise
sigmaNoise = 1.0

depthSigma = 1.0  # Standared deviation of field depth [meters]

###############################################################################
# Gaussian Process Regression

# Ordinary kriging
d_lag = 2*ds
tol = d_lag/2
stc=1.0 # scale distance torwards center
lag_max = np.sqrt(((xf-x0)/stc)**2 + ((yf-y0)/stc)**2)/2
# lag_max = 100
all_lags = np.transpose(np.array([np.arange(0.0, lag_max, d_lag)]))
lag_vec = np.reshape(all_lags, (len(all_lags), 1))
nLags = len(all_lags)
sill = mMin
sill0 = mMax
min_empirical_index = 5
min_empirical_lag = 4
ise = 0
sill_eff = 0
nugget = 0
range2a = 0
C = np.array([])
Zhat = np.ones((yLen, xLen))
Vzh = np.ones((yLen, xLen))#*1000
MZ = np.array([np.zeros((yLen, xLen))])
MV = np.array([np.zeros((yLen, xLen))])

# For partitioned ordinary kriging
lowestLevel = 3

###############################################################################
# State space
full_state = np.array([[xf-4.0],
                       [yf-2.0],
                       [0.0],
                       [0.0],
                       [0.0],
                       [0.0],
                       [0.0],
                       [0.0],
                       [0.0],
                       [0.0],
                       [0.0],
                       [0.0]])

point_mass_state = np.array([[xf-4.0], # x
                             [yf-2.0], # y
                             [0.0], # z
                             [0.0], # vx
                             [0.0], # vy
                             [0.0]]) # vz

solid_sphere_state = np.array([[0.0],  # roll
                               [0.0],  # pitch
                               [0.0],  # yaw
                               [0.0],  # p
                               [0.0],  # q
                               [0.0]])  # r

###############################################################################
# Physics
mass = 21.0  # kilograms

# Sphere dimensions and drag
sphere_radius = 0.914  # radius of sphere in meters
Area = np.pi*(sphere_radius**2)  # Cross sectional area of a sphere
rho = 997.0  # kg/(m^3) fluid density of water
Cd = 0.5  # Drag coefficient of a sphere
Cf = 0.25  # skin friction coefficient

# References
reference_speed = 1.2 # meters per second

# Controller
kp_rudder = 1.0
ki_rudder = 0.1
kd_rudder = 1.0

max_rudder_angle = 0.8 #np.pi/3
min_rudder_angle = -0.8 #-np.pi/3

max_thrust = 100000 # Newtons
min_thrust = 0

moment_of_inertia_sphere = (2.0/5.0)*mass*(sphere_radius**2.0)
max_torque = max_thrust*sphere_radius
max_angular_acceleration = max_torque*moment_of_inertia_sphere

# Trajectory

# Default waypoint       x,   y
defaultWP = np.array([[0.0, 0.0]])

# North vector
north_vec = np.array([[0.0, 1.0]])

# Waypoints
wp_threshold = 0.5 # meters