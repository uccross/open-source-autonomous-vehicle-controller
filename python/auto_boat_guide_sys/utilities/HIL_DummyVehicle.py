"""
.. module:: HIL_DummyVehicle.py
    :platform: MacOS, Unix, Windows,
    :synopsis: A class test class for a sphere-shaped dummy vehicle 
.. moduleauthor:: Pavlo Vlastos <pvlastos@ucsc.edu>
"""

from re import S
from . import Constants as CONST
from . import Model
from . import State
from . import Force
from . import Torque
from . import Linear as LN
from . import Quaternions as QU
import numpy as np


class DualModel():
    def __init__(self, dt_sim=CONST.dt_sim, dt_uc=CONST.dt_uc, mass=CONST.mass,
                 point_mass_state=CONST.point_mass_state,
                 orientation_state=CONST.solid_sphere_state,
                 radius=CONST.sphere_radius,
                 reference_speed=CONST.reference_speed,
                 max_angle=CONST.max_rudder_angle,
                 min_angle=CONST.min_rudder_angle):

        # Computation timing
        self.dt_uc = dt_uc
        self.counter = 0
        self.dt_ratio = dt_uc/dt_sim

        # Point mass
        self.point_mass_state = State.PointMassState(point_mass_state)
        self.point_mass_model = Model.PointMassDynamics(dt_sim, mass)

        # Orientation
        self.orientation_state = State.OrientationState(orientation_state)
        self.orientation_model = Model.SolidSphere(dt_sim, mass, radius)

        # Vehicle geometry
        self.tvc_angle = 0.0
        self.delta_angle = 0.0

        # Forces
        self.F_thrust_inertial = Force.Thrust()
        self.F_thrust_body = Force.Thrust()
        self.thrust_mag_cmd = 0.0

        # Drag force
        self.F_drag_linear = Force.DragLinear()

        # Torques
        self.T_thrust = Torque.FromThrust()
        self.T_drag = Torque.FromDrag()

        self.reference_speed = reference_speed

        self.is_mission_complete = False

        self.min_angle = min_angle
        self.max_angle = max_angle

        return

    def update(self, input_rudder_angle):
        """
        :param input_rudder_angle:
        """
        #######################################################################
        # Grab models' states
        x_pm = self.point_mass_state.get_state_vector()
        x_om = self.orientation_state.get_state_vector()

        #######################################################################
        # Computation
        if np.mod(self.counter, self.dt_ratio) == 0:
            self.tvc_angle = -input_rudder_angle

            # Limit angle
            if self.tvc_angle > self.max_angle:
                self.tvc_angle = self.max_angle

            if self.tvc_angle < self.min_angle:
                self.tvc_angle = self.min_angle

        #######################################################################
        # Forces @TODO have controller for commanded angel vs measured

        # Thrust
        self.thrust_mag_cmd = self.reference_speed

        # Adjust the thrust magnitude within the body frame
        self.F_thrust_body.update(x=self.thrust_mag_cmd*np.sin(self.tvc_angle),
                                  y=-self.thrust_mag_cmd*np.cos(self.tvc_angle),
                                  z=0.0)

        # rotate the thrust vector into the inertial frame from the body frame
        self.F_thrust_inertial.vec = QU.rotateVectorWithQuaternion(
            self.F_thrust_body.vec,
            x_om[2][0],
            x_om[1][0],
            x_om[0][0])

        # Drag
        self.F_drag_linear.update(x_pm[3:])

        F_linear_sum = self.F_thrust_inertial.vec + self.F_drag_linear.vec

        #######################################################################
        # Torques
        radius = self.orientation_model.get_radius()

        # Thrust
        self.T_thrust.update(self.F_thrust_body.vec, radius)

        # Drag
        self.T_drag.update(x_om[3:])

        T_sum = self.T_thrust.vec + self.T_drag.vec

        #######################################################################
        # Model
        x_pm = self.point_mass_state.get_state_vector()
        x_pm = self.point_mass_model.update(x_pm, F_linear_sum)
        self.point_mass_state.set_state_vector(x_pm)

        x_sp = self.orientation_state.get_state_vector()
        x_sp = self.orientation_model.update(x_sp, T_sum)
        self.orientation_state.set_state_vector(x_sp)

        #######################################################################
        # Computation timing
        self.counter += 1

        return self.is_mission_complete

    #######################################################################
    # Getters
    def get_vehicle_point_state(self):
        return self.point_mass_state.get_state_vector()

    def get_vehicle_orientation_state(self):
        return self.orientation_state.get_state_vector()

    def get_vehicle_position(self):
        return np.array([[self.point_mass_state.x, self.point_mass_state.y]])

    def get_vehicle_heading(self):
        return self.orientation_state.yaw

    def get_vehicle_yaw_rate(self):
        return self.orientation_state.r

    def set_trajectory_prev_wp(self, new_wp):
        self.trajectory.setPreviousWaypoint(new_wp)
        return

    def set_trajectory_next_wp(self, new_wp):
        self.trajectory.setNextWaypoint(new_wp)
        return

    def get_trajectory_prev_wp(self):
        return self.trajectory.getPreviousWaypoint()

    def get_trajectory_next_wp(self):
        return self.trajectory.getNextWaypoint()

    def get_trajectory_path_angle(self):
        return self.trajectory.getPathAngle()

    def get_trajectory_cte(self):
        return self.trajectory.getCte()

    def get_trajectory_closest_point(self):
        return self.trajectory.getClosestPoint()

    def get_x(self):
        return self.point_mass_state.get_state_vector()[0][0]  # x

    def get_y(self):
        return self.point_mass_state.get_state_vector()[1][0]  # y

    def get_vx(self):
        return self.point_mass_state.get_state_vector()[3][0]  # vx

    def get_vy(self):
        return self.point_mass_state.get_state_vector()[4][0]  # vy

    def get_speed(self):
        return self.speed

    def get_reference_speed(self):
        return self.reference_speed

    def get_tbx(self):
        return self.F_thrust_body.vec[0][0]

    def get_tby(self):
        return self.F_thrust_body.vec[1][0]

    def get_torque_z(self):
        return self.T_thrust.vec[2][0]

    def get_tvc_angle(self):
        return self.tvc_angle

    def get_delta_angle(self):
        return self.delta_angle

    def get_thrust_mag_cmd(self):
        return self.thrust_mag_cmd

    def mission_complete(self):
        return self.is_mission_complete
