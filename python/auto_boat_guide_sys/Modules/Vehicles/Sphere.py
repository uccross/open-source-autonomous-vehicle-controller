"""
.. module:: Sphere.py
    :platform: MacOS, Unix, Windows,
    :synopsis: A class for a sphere-shaped vehicle
.. moduleauthor:: Pavlo Vlastos <pvlastos@ucsc.edu>
"""

from re import S
from ..Constants import Constants as CONST
from ..Controller import Controller as CTRL
from ..Model import Model
from ..Containers import State
from ..Containers import Force
from ..Containers import Torque
from ..Trajectories import Linear as LN
from ..Field import Field as FD
from ..Utilities import Quaternions as QU
from ..Utilities import GaussianProcessRegression as GPR
from ..Waypoints import WaypointQueue as WQ
from ..Utilities import Graph as Graph
from ..Gradients import CostMap as CMAP
import numpy as np


class DualModel():
    def __init__(self, dt_sim=CONST.dt_sim, dt_uc=CONST.dt_uc, mass=CONST.mass,
                 point_mass_state=CONST.point_mass_state,
                 orientation_state=CONST.solid_sphere_state,
                 radius=CONST.sphere_radius,
                 reference_speed=CONST.reference_speed,
                 min_thrust=CONST.min_thrust,
                 max_thrust=CONST.max_thrust,
                 threshold=CONST.wp_threshold,
                 waypoints=np.array([]),
                 spatial_estimator='OK',
                 max_wp_count=7,
                 estimator_update_case='local_point_transition',
                 variogram_update_case='once',
                 vmin=CONST.mMin,
                 vmax=CONST.mMax,
                 lag_vec=CONST.lag_vec,
                 all_lags=CONST.all_lags,
                 lag_max=CONST.lag_max,
                 field=FD.Field(),
                 final_point=np.zeros((1, 2)),
                 final_point_max_count=10,
                 initial_range_guess=11.0,
                 distance_weight=100,
                 edge_multiple=4):

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

        # Trajectory
        self.trajectory = LN.Linear()

        # References
        self.reference_speed = reference_speed
        self.reference_angle = 0.0

        # Controllers
        # self.heading_controller = CTRL.PD(dt=dt_uc, kp=1000.0, kd=0.1,
        #                                   min_act_lim=CONST.min_rudder_angle,
        #                                   max_act_lim=CONST.max_rudder_angle)

        self.cross_track_controller = CTRL.PD(
            dt=dt_uc, kp=10.0, kd=100.0,
            min_act_lim=-10000.0,
            max_act_lim=10000.0)  # enforces controller priority

        self.speed_controller = CTRL.P(dt=dt_uc, kp=5000.0,
                                       min_act_lim=min_thrust,
                                       max_act_lim=max_thrust)

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

        # Waypoints
        self.threshold = threshold
        self.wp_queue = WQ.WaypointQueue(threshold=self.threshold)

        for row in waypoints:
            row = np.array([row])
            print("adding wp: {}, shape: {}".format(row, row.shape))
            self.wp_queue.add(row)

        print("peak queue before prev:\r\n{}".format(
            self.wp_queue.peak_next_waypoint()))

        self.set_trajectory_prev_wp(self.wp_queue.getNext())

        print("peak queue after prev:\r\n{}".format(
            self.wp_queue.peak_next_waypoint()))

        # Spatial estimation
        self.spatial_estimator_type = spatial_estimator
        if spatial_estimator == 'OK':
            self.spatial_estimator = GPR.OrdinaryKriging(field=field)
        if spatial_estimator == 'IIOK':
            self.spatial_estimator = GPR.OrdinaryKriging()
        if spatial_estimator == 'POK':
            self.spatial_estimator = GPR.PartitionedKriging()

        self.field = field

        self.final_point = final_point
        self.final_point_counter = 0
        self.final_point_max_count = final_point_max_count

        iy, ix = self.field.getTrueFieldIndices(
            self.final_point[0][1],
            self.final_point[0][0])

        self.final_point_indices = [ix, iy]

        x_pm = self.point_mass_state.get_state_vector()

        vehicle_position = np.array([[x_pm[0][0], x_pm[1][0]]])

        # Local start point
        ivy, ivx = self.field.getTrueFieldIndices(
            vehicle_position[0][1],
            vehicle_position[0][0])

        self.start_point = [ivx, ivy]

        ###########################################################
        # Cost matrix
        self.ke_mat = CMAP.cost_map(self.field.get_yLen(),
                                    self.field.get_xLen(),
                                    self.start_point,
                                    self.final_point_indices,
                                    k=10000000,
                                    umin=0,
                                    umax=12)

        self.cost_matrix = - self.ke_mat

        # Compute next point
        stop_point = self.final_point_indices

        print("next stop point:\n{}".format(stop_point))
        print("next start point:\n{}".format(self.start_point))

        node_matrix = Graph.make_node_matrix(
            m=self.field.get_yLen(), n=self.field.get_xLen())

        self.distance_weight = distance_weight

        # Graphing
        self.edge_multiple = edge_multiple  # Multiples of ds is for edges in x
        # and y
        self.grapher = Graph.Graph(
            num_vertices=node_matrix.size,
            distance_weight=self.distance_weight,
            variance_matrix=self.spatial_estimator.getVarianceMatrix())

        self.arrows_x = []
        self.arrows_y = []
        self.arrows_dx = []
        self.arrows_dy = []

        self.sol_arrows_x = []
        self.sol_arrows_y = []
        self.sol_arrows_dx = []
        self.sol_arrows_dy = []

        [self.arrows_x,
         self.arrows_y,
         self.arrows_dx,
         self.arrows_dy] = self.grapher.generate_dag(stop_point,
                                                     self.start_point,
                                                     nth=self.edge_multiple)

        src_node, dest_node = self.grapher.get_src_and_dest_node(
            stop_point, self.start_point, nth=self.edge_multiple)

        [self.sol_arrows_x,
         self.sol_arrows_y,
         self.sol_arrows_dx,
         self.sol_arrows_dy] = self.grapher.generate_optimal_path(
            src_node, dest_node,
            self.arrows_x, self.arrows_y,
            self.arrows_dx, self.arrows_dy)

        for ax, ay, in zip(self.sol_arrows_x, self.sol_arrows_y):

            new_point = self.field.getTrueFieldPointCoordinate(
                ay, ax)

            self.wp_queue.add(new_point)

        self.wp_queue.add(self.final_point)

        # burn a point
        self.wp_queue.getNext()
        self.set_trajectory_next_wp(self.wp_queue.getNext())

        print("peak queue after next:\r\n{}".format(
            self.wp_queue.peak_next_waypoint()))

        self.range2a = initial_range_guess

        i_wp_next = 1
        print("i_wp_next: {}".format(i_wp_next))

        self.next_local_point = np.array([
            self.wp_queue.get_waypoints()[i_wp_next, :]])
        print("self.next_local_point:\n{}".format(self.next_local_point))
        self.local_point_counter = 0
        self.new_estimate_flag = False

        self.max_wp_count = max_wp_count

        self.estimator_update_case = estimator_update_case

        self.variogram_update_case = variogram_update_case

        self.vmin = vmin
        self.vmax = vmax
        self.all_lags = all_lags
        self.lag_vec = lag_vec
        self.lag_max = lag_max

        self.variogram = np.array([])
        self.empirical_variogram = np.array([])

        self.vehicle_position_matrix = vehicle_position

        self.is_mission_complete = False

        return

    def update(self, measurement_z_x_y_log=np.array([[0.0, 0.0, 0.0]])):
        """
        :param measurement_z_x_y_log: np.array([[z, x, y]])
        """
        #######################################################################
        # Grab models' states
        x_pm = self.point_mass_state.get_state_vector()
        x_om = self.orientation_state.get_state_vector()

        #######################################################################
        # Computation
        if np.mod(self.counter, self.dt_ratio) == 0:

            ###################################################################
            # Sensor measurements @TODO:
            vehicle_position = np.array([[x_pm[0][0], x_pm[1][0]]])

            self.speed = np.linalg.norm(x_pm[3:])

            ###################################################################
            # Trajectory
            self.trajectory.udpate(vehicle_position)

            ###################################################################
            # References
            self.reference_angle = self.trajectory.getPathAngle()

            angle_error = self.orientation_state.yaw - self.reference_angle

            # Prevent turning the wrong way by making sure the angle error is
            # wrapped to [-pi, pi]
            angle_error = (angle_error + np.pi) % (2.0 * np.pi) - np.pi

            cte = self.trajectory.getCte()

            closest_point = self.trajectory.getClosestPoint()

            ###################################################################
            # Update Spatial Estimator
            if self.estimator_update_case == 'local_point_transition':

                proximity_flag = in_proximity(closest_point,
                                              self.next_local_point,
                                              self.wp_queue.threshold)

                ###############################################################
                # Update estimate and optimal path
                if (proximity_flag):

                    ###########################################################
                    # generate empirical variogram
                    if self.variogram_update_case == 'once':
                        # self.variogram_update_case = 0

                        E = self.empirical_variogram = self.spatial_estimator.\
                            makeEmpericalSemiVariogram(
                                measurement_z_x_y_log, vmin=self.vmin,
                                vmax=self.vmax
                            )

                        # S, LPS = self.spatial_estimator.calculateSemiVariogram(
                        #     self.empirical_variogram, 40, 1)

                        sill, self.range2a, nugget = self.spatial_estimator.\
                            fit2EmpiricalScipy(
                                self.lag_vec[0:len(E)],
                                E,
                                # 1000)
                                self.lag_max*2,
                                1)

                        self.variogram = self.spatial_estimator.variogramGauss(
                            self.lag_vec)

                    print("calculating covariance matrix ...")
                    C = self.spatial_estimator.calcCovMatFromVario(
                        measurement_z_x_y_log)

                    print("estimating field ...")
                    _, Vzh = self.spatial_estimator.predict(
                        C, measurement_z_x_y_log)

                    # Normalize
                    if len(measurement_z_x_y_log) > 2:
                        Vzh -= np.min(Vzh)
                        Vzh /= np.abs(np.max(Vzh) - np.min(Vzh))
                        Vzh *= np.abs(10.0 - 0.0)  # variance, @TODO: make nice

                    self.new_estimate_flag = True

                    # Local start point
                    iy_start, ix_start = self.field.getTrueFieldIndices(
                        vehicle_position[0][1],
                        vehicle_position[0][0])

                    new_start_point = [ix_start, iy_start]

                    stop_point = self.final_point_indices

                    ###########################################################
                    # Cost matrix
                    self.ke_mat = CMAP.cost_map(self.field.get_yLen(),
                                                self.field.get_xLen(),
                                                self.start_point,
                                                stop_point,
                                                k=10000000,
                                                umin=5,
                                                umax=15)

                    self.cost_matrix = Vzh - self.ke_mat

                    Vzh = Vzh - self.ke_mat

                    ###########################################################
                    # Update final point
                    if (in_proximity(closest_point, self.final_point,
                                     self.wp_queue.threshold)):

                        iy_var, ix_var = np.where(Vzh == np.max(Vzh))

                        if len(iy_var) == 0:
                            iy_var = np.random.choice(
                                range(0, self.field.get_yLen()))

                        elif len(iy_var) > 0:
                            iy_var = iy_var[0]

                        if len(ix_var) == 0:
                            ix_var = np.random.choice(
                                range(0, self.field.get_xLen()))

                        elif len(ix_var) > 0:
                            ix_var = ix_var[0]

                        self.start_point = self.final_point_indices

                        self.final_point = self.field.\
                            getTrueFieldPointCoordinate(iy_var, ix_var)

                        self.final_point_indices = [ix_var, iy_var]

                        stop_point = self.final_point_indices

                        self.final_point_counter += 1

                        if (self.final_point_counter >=
                                self.final_point_max_count):

                            self.is_mission_complete = True

                    ###########################################################
                    # Update the optimal path
                    node_matrix = Graph.make_node_matrix(
                        m=self.field.get_yLen(), n=self.field.get_xLen())

                    # Find better way to reset graph
                    self.grapher = Graph.Graph(
                        num_vertices=node_matrix.size,
                        distance_weight=self.distance_weight,
                        variance_matrix=Vzh)

                    [self.arrows_x,
                        self.arrows_y,
                        self.arrows_dx,
                        self.arrows_dy] = self.grapher.generate_dag(
                        stop_point,
                        self.start_point,
                        nth=self.edge_multiple)

                    src_node, dest_node = self.grapher.get_src_and_dest_node(
                        stop_point, new_start_point, nth=self.edge_multiple)

                    [self.sol_arrows_x,
                        self.sol_arrows_y,
                        self.sol_arrows_dx,
                        self.sol_arrows_dy] = self.grapher.generate_optimal_path(
                        src_node, dest_node,
                        self.arrows_x, self.arrows_y,
                        self.arrows_dx, self.arrows_dy)

                    ###########################################################
                    # Reset the waypoint queue
                    self.wp_queue = WQ.WaypointQueue(threshold=self.threshold)

                    for ax, ay, in zip(self.sol_arrows_x, self.sol_arrows_y):

                        new_point = self.field.getTrueFieldPointCoordinate(
                            ay, ax)

                        self.wp_queue.add(new_point)

                    self.wp_queue.add(self.final_point)

                    i_wp_next = 4
                    # i_wp_next = int(np.floor(self.range2a /
                    #                          self.field.get_resolution()))
                    if i_wp_next >= self.wp_queue.getTotal():
                        i_wp_next = self.wp_queue.getTotal()-1
                        print("    i_wp_next maxed out")
                        print("self.wp_queue:\n{}".format(
                            self.wp_queue.get_waypoints()))

                    self.next_local_point = np.array([
                        self.wp_queue.get_waypoints()[i_wp_next, :]])

                    self.trajectory.setNextWaypoint(
                        self.wp_queue.getNext())

                    # update the global point counter mainly for graphing
                    # purposes
                    self.local_point_counter += 1

                ###############################################################
                # Update next point along path of points towards the global
                # next point

                beyond_next_flag = self.trajectory.\
                    is_closest_point_beyond_next(self.wp_queue.threshold)
                if (beyond_next_flag):

                    if self.wp_queue.getTotal() > 1:

                        self.trajectory.setPreviousWaypoint(
                            self.trajectory.getNextWaypoint())

                        self.trajectory.setNextWaypoint(
                            self.wp_queue.getNext())

                    if len(measurement_z_x_y_log) > 2:

                        print("*calculating covariance matrix ...")
                        C = self.spatial_estimator.calcCovMatFromVario(
                            measurement_z_x_y_log)

                        print("*estimating field ...")
                        self.spatial_estimator.predict(
                            C, measurement_z_x_y_log)

                        self.new_estimate_flag = True

                    self.vehicle_position_matrix = np.concatenate(
                        (self.vehicle_position_matrix, vehicle_position),
                        axis=0)

                    # Update the reference speed
                    next_wp = self.get_trajectory_next_wp()

                    iny, inx = self.field.getTrueFieldIndices(next_wp[0][1],
                                                              next_wp[0][0])

                    self.reference_speed = self.ke_mat[iny][inx]

            ###################################################################
            # Controllers
            self.tvc_angle = 0

            # Speed control
            self.thrust_mag_cmd = self.speed_controller.update(
                self.reference_speed,
                self.speed)

            # self.thrust_mag_cmd = self.reference_speed

            # Heading control
            # self.tvc_angle = self.heading_controller.update(
            #     self.reference_angle,  # negative because cw is +
            #     self.orientation_state.yaw,  # negative because cw is +
            #     self.orientation_state.r)

            # Trajectory tracking
            acc_cmd = self.cross_track_controller.update(
                0.0,  # reference
                cte,  # measured
                angle_error)  #

            if acc_cmd != 0:
                usign = np.sign(acc_cmd)
                rTemp = (self.speed**2)/np.abs(acc_cmd)

                if rTemp != 0:
                    self.delta_angle = usign*(np.sqrt(np.abs(acc_cmd)
                                                      / rTemp) * self.dt_uc)

            # Sum control input from the two simple controllers
            # self.tvc_angle = self.tvc_angle - self.delta_angle
            self.tvc_angle = -self.delta_angle

        #######################################################################
        # Forces @TODO have controller for commanded angel vs measured

        # Thrust
        # Adjust the thrust magnitude within the body frame
        self.F_thrust_body.update(x=self.thrust_mag_cmd*np.sin(-self.tvc_angle),
                                  y=self.thrust_mag_cmd*np.cos(self.tvc_angle),
                                  z=0.0)
        
        # Possible bug fix: did it work? [Yes]
        F_thrust_body_temporary = np.zeros((3,1))
        F_thrust_body_temporary[0] = -self.F_thrust_body.vec[0]
        F_thrust_body_temporary[1] = self.F_thrust_body.vec[1]
        F_thrust_body_temporary[2] = self.F_thrust_body.vec[2]

        # rotate the thrust vector into the inertial frame from the body frame
        self.F_thrust_inertial.vec = QU.rotateVectorWithQuaternion(
            F_thrust_body_temporary,
            x_om[0][0],
            x_om[1][0],
            x_om[2][0])

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

    def get_start_point(self):
        ix, iy = self.start_point
        return self.field.getTrueFieldPointCoordinate(iy, ix)

    def get_final_point(self):
        return self.final_point

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

    def get_waypoints_peak_next(self):
        return self.wp_queue.peak_next_waypoint()

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

    def get_waypoints(self):
        # return self.wp_queue.get_waypoints()
        return self.wp_queue.get_waypoints()

    def get_empirical_variogram(self):
        return self.empirical_variogram

    def get_variogram(self):
        return self.variogram

    def get_variance_matrix(self):
        return self.spatial_estimator.getVarianceMatrix()

    def get_cost_matrix(self):
        return self.cost_matrix

    def get_estimate_matrix(self):
        return self.spatial_estimator.getFieldEstimate()

    def get_arrows(self):
        """
        @TODO: get rid of this later?
        """
        return self.arrows_x, self.arrows_y, self.arrows_dx, self.arrows_dy

    def get_sol_arrows(self):
        return self.sol_arrows_x, self.sol_arrows_y, self.sol_arrows_dx, self.sol_arrows_dy

    def get_distance_weight(self):
        return self.distance_weight

    def get_local_point_count(self):
        return self.local_point_counter

    def get_next_local_point(self):
        return self.next_local_point

    def is_new_estimate(self):
        status = self.new_estimate_flag
        self.new_estimate_flag = False
        return status

    def mission_complete(self):
        return self.is_mission_complete

###############################################################################
# Helper functions


def in_proximity(point_a=np.zeros((1, 2)), point_b=np.zeros((1, 2)),
                 threshold=1.2):
    """
    :param point: Compare if two points are closet to each other
    :param point_a
    :param point_b:
    :return: True or False
    """
    return (np.linalg.norm(point_a - point_b) <= threshold)


def get_distance_along_path(position_matrix=np.zeros((3, 2)), i_start=0,
                            i_stop=-1):
    """
    :param position_matrix: An mx2 matrix of positions with rows of
    np.array([[x, y]])
    :param i_start: the starting index for the rows of the waypoint matrix
    :param i_stop: the stopping index for the rows of the waypoint matrix
    """
    total_distance = 0

    if i_stop == -1:
        i_stop, _ = position_matrix.shape

    if i_stop-i_start > 1:
        for i in range(i_start, i_stop-1):
            total_distance += np.linalg.norm(position_matrix[i, :] -
                                             position_matrix[i+1, :])

    elif i_stop-i_start == 1:
        total_distance = np.linalg.norm(position_matrix[i_start, :] -
                                        position_matrix[i_stop, :])

    else:
        print("Path Planner get_distance_along_path() error")

    return total_distance
