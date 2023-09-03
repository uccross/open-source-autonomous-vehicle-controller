#!/usr/bin/env python2.7
import rospy
from std_msgs.msg import Float64
from tf.transformations import euler_from_quaternion
from matplotlib import pyplot as plt
from gazebo_msgs.msg import LinkStates

# Variable to hold states of the links
model_state = LinkStates()

#Callback function to transfer link state information from gazebo
def link_state_callback(data):
    global model_state
    model_state = data

#Function for converting from quaterion notation to roll,pitch,yaw notation
def convert_to_rpy(pose):
    roll, pitch, yaw = euler_from_quaternion([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])
    return [roll, pitch, yaw]

#Ros topic names
link_state_topic = '/gazebo/link_states'
left_velocity_topic = '/left_motor_controller/command'
right_velocity_topic = '/right_motor_controller/command'

#Inializing ros node
rospy.init_node('custom_controller')

#objects for ros publisher and subscribers
odometry_subscriber = rospy.Subscriber(link_state_topic, LinkStates , link_state_callback)
left_velocity_publisher = rospy.Publisher(left_velocity_topic, Float64, queue_size=2)
right_velocity_publisher = rospy.Publisher(right_velocity_topic, Float64, queue_size=2)

#sleep function for waiting till callback get data from ros master
rospy.sleep(1)

'''
- The control loop for simulation is using PD controller(Used code from https://thingsdaq.org/2022/04/07/digital-pid-controller/).

- The motor take off around 40 units for propeller velocity as this depends on the aerodynamic parameters on the propeller of the bot

Where,

Input = motor reference speed
output = motor's speed correction (for left = ref speed + correction and right = ref speed - correction)
feedback = arm pivot joint angle
'''

#Variables
current_pose=convert_to_rpy(model_state.pose[2])[1] # arm pivot joint current angle in radians 
target_pose =0.0 # arm pivot joint target angle in radians

# #output variable
vel = 40.0

# PID controller
class PID:
    """
    The class to represent a discrete PID controller.
    For more information about this class go to:
    https://thingsdaq.org/2022/04/07/digital-pid-controller/
    Create a PID controller:
        >>> Ts = 0.01
        >>> kp = 0.15
        >>> ki = 0.35
        >>> kd = 0.01
        >>> mypid = PID(Ts, kp, ki, kd)
    :param Ts: The sampling period of the execution loop.
    :type Ts: float
    :param kp: The PID proportional gain.
    :type kp: float
    :param ki: The PID integral gain.
    :type ki: float
    :param kd: The PID derivative gain.
    :type kd: float
    :param umax: The upper bound of the controller output saturation.
        Defalt value is ``1``.
    :type umax: float
    :param umin: The lower bound of the controller output saturation.
        Defalt value is ``-1``.
    :type umin: float
    :param tau: The derivative term low-pass filter response time (s).
        Defalt value is ``0``.
    :type tau: float
    """
    def __init__(self, Ts, kp, ki, kd, umax=1, umin=-1, tau=0):
        """
        Class constructor.
        """
        self._Ts = Ts  # Sampling period (s)
        self._kp = kp  # Proportional gain
        self._ki = ki  # Integral gain
        self._kd = kd  # Derivative gain
        self._umax = umax  # Upper output saturation limit
        self._umin = umin  # Lower output saturation limit
        self._tau = tau  # Derivative term filter time constant (s)
        #
        self._eprev = [0, 0]  # Previous errors e[n-1], e[n-2]
        self._uprev = 0  # Previous controller output u[n-1]
        self._udfiltprev = 0  # Previous filtered value

    def control(self, xsp, x, uff=0):
        """
        Calculate PID controller output.
        :param xsp: The set point value at the time step.
        :type xsp: float
        :param x: The actual value at the time step.
        :type x: float
        :param uff: The feed-forward value at the time step.
            Default value is ``0``.
        :type uff: float
        """
        # Calculating error
        e = xsp - x

        # Calculating proportional term
        up = self._kp * (e - self._eprev[0])
        # Calculating integral term (with anti-windup)
        ui = self._ki*self._Ts * e
        if (self._uprev+uff >= self._umax) or (self._uprev+uff <= self._umin):
            ui = 0
        # Calculating derivative term
        ud = self._kd/self._Ts * (e - 2*self._eprev[0] + self._eprev[1])
        # Filtering derivative term
        udfilt = (
            self._tau/(self._tau+self._Ts)*self._udfiltprev +
            self._Ts/(self._tau+self._Ts)*ud
        )
        # Calculating PID controller output
        u = self._uprev + up - ui + udfilt + uff
        # Updating previous time step errors
        self._eprev[1] = self._eprev[0]
        self._eprev[0] = e
        # Updating previous time step output value
        self._uprev = u - uff
        # Updating previous time step derivative term filtered value
        self._udfiltprev = udfilt
        # Limiting output (just to be safe)
        if u < self._umin:
            u = self._umin
        elif u > self._umax:
            u = self._umax
        # Returning controller output at current time step
        return u

#PID controller parameters
kp = 1
ki = 0.0
kd = 1
tsample = 0.001
taupid = 0.01

#PID controller object
pid = PID(tsample, kp, ki, kd, tau=taupid)

# variable to hold error data for visualization
visualize_error = []

#loop for controlling the bot and exiting the control loop by ctrl + c
try:
    while (not rospy.is_shutdown()):
        control_action = pid.control(target_pose, current_pose)
        # print(vel,control_action,target_pose - current_pose)
        left_velocity_publisher.publish(vel+control_action)
        right_velocity_publisher.publish(vel-control_action)
        current_pose=convert_to_rpy(model_state.pose[2])[1]
        visualize_error.append((target_pose-current_pose))

except:
    print("ended")    
visualize_timesteps = [i for i in range(len(visualize_error))]
plt.plot(visualize_timesteps,visualize_error)
plt.xlabel("Simulation timesteps")
plt.ylabel("Error (in radians)")
plt.legend()
plt.show()
