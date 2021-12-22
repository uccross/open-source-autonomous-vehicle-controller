"""
.. module:: Controller.py
	:platform: MacOS, Unix, Windows,
	:synopsis: A class for discrete Proportional Integral Derivative (PID) 
    Controllers
.. moduleauthor:: Pavlo Vlastos <pvlastos@ucsc.edu>
"""
import numpy as np
from ..Constants import Constants as CNST


class PID():
    def __init__(self, dt=CNST.dt_sim, kp=CNST.kp_rudder, ki=CNST.ki_rudder,
                 kd=CNST.kd_rudder, max_act_lim=CNST.max_rudder_angle,
                 min_act_lim=CNST.min_rudder_angle):
        """
        :param dt: The sample time in seconds
        :param kp: Propertional gain
        :param ki: Integral gain
        :param kd: Derivative gain
        :return: none
        """

        self.dt = dt
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.max_act_lim = max_act_lim
        self.min_act_lim = min_act_lim
        self.prevError = 0.0
        self.accumulator = 0.0
        self.uLast = 0.0
        return

    def update(self, commanded, measured, derivative):
        """
        def update(self, commanded, measured, derivative):
        :param commanded: The commanded value for control
        :param measured: The measured value of the system
        :param derivative: The measured rate of change of the system (if 
        applicable)
        :return: u, The resulting control input
        """

        error = commanded - measured

        # Trapezoidal Integration
        self.accumulator += 0.5*self.dt*(error + self.prevError)

        u = self.kp*error + self.ki*self.accumulator + self.kd*derivative

        if u > self.max_act_lim:
            self.accumulator -= 0.5*self.dt*(error + self.prevError)
            u = self.max_act_lim

        elif u < self.min_act_lim:
            self.accumulator -= 0.5*self.dt*(error + self.prevError)
            u = self.min_act_lim

        self.prevError = error
        self.uLast = u
        return u

    def getLastU(self):
        """
        :return: The last resulting control input
        """
        return self.uLast

class PD():
    def __init__(self, dt=CNST.dt_sim, kp=CNST.kp_rudder, kd=CNST.kd_rudder,
                 max_act_lim=CNST.max_rudder_angle, 
                 min_act_lim=CNST.min_rudder_angle):
        """
        :param dt: The sample time in seconds
        :param kp: Propertional gain
        :param kd: Derivative gain
        :return: none
        """

        self.dt = dt
        self.kp = kp
        self.kd = kd
        self.max_act_lim = max_act_lim
        self.min_act_lim = min_act_lim
        self.u_last = 0.0
        return

    def update(self, commanded, measured, derivative):
        """
        def update(self, commanded, measured, derivative):
        :param commanded: The commanded value for control
        :param measured: The measured value of the system
        :param derivative: The measured rate of change of the system (if 
        applicable)
        :return: u, The resulting control input
        """

        error = commanded - measured

        u = self.kp*error + self.kd*derivative

        if u > self.max_act_lim:
            u = self.max_act_lim

        elif u < self.min_act_lim:
            u = self.min_act_lim

        self.u_last = u
        return u

    def getLastU(self):
        """
        :return: The last resulting control input
        """
        return self.u_last


class P():
    def __init__(self, dt=CNST.dt_sim, kp=CNST.kp_rudder,
                 max_act_lim=CNST.max_rudder_angle, 
                 min_act_lim=CNST.min_rudder_angle):
        """
        :param dt: The sample time in seconds
        :param kp: Propertional gain
        :param kd: Derivative gain
        :return: none
        """

        self.dt = dt
        self.kp = kp
        self.max_act_lim = max_act_lim
        self.min_act_lim = min_act_lim
        self.u_last = 0.0
        return

    def update(self, commanded, measured):
        """
        def update(self, commanded, measured):
        :param commanded: The commanded value for control
        :param measured: The measured value of the system
        :return: u, The resulting control input
        """

        error = commanded - measured

        u = self.kp*error

        if u > self.max_act_lim:
            u = self.max_act_lim

        elif u < self.min_act_lim:
            u = self.min_act_lim

        self.u_last = u
        return u

    def getLastU(self):
        """
        :return: The last resulting control input
        """
        return self.u_last
