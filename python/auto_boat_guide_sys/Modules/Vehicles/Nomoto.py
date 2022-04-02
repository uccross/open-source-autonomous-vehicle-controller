"""
.. module:: Nomoto.py
    :platform: MacOS, Unix, Windows,
    :synopsis: A class for a sphere-shaped vehicle
.. moduleauthor:: Pavlo Vlastos <pvlastos@ucsc.edu>
"""

from ..Constants import Constants as CONST
from ..Controller import Controller as CTRL
from ..Model import Model
from ..Containers import State
from ..Containers import Force
from ..Containers import Torque
from ..Trajectories import Linear as LN
from ..Field import Field as FD
from ..Utilities import Quaternions as QU
from ..Utilities import GaussianProcess as GP
from ..Utilities import Kriging as KG
from ..Utilities import Graph as Graph
from ..Utilities import PathPlanner as PATH
import matplotlib.pyplot as plt

import time
import copy

import numpy as np


class AugmentedIdeal():
    def __init__(self, dt_sim=CONST.dt_sim, dt_uc=CONST.dt_uc, threshold=2.5,
                 path_planner_type="Zig-zag", spatial_est_type="GPR",
                 field=FD.Field(),
                 initial_state=CONST.nomoto_state,
                 sigma_w=0.0,
                 starting_position=np.zeros((1, 2)),
                 stopping_position=np.zeros((1, 2)),
                 max_recursion_level=1,
                 node_separation=5,
                 estimate_interval=3,
                 vario_interval=50,
                 correction_threshold=20.0, distance_weight=10.0,
                 k_sill=1.0,
                 k_rng=100.0,
                 dynamic_hyperparams=False,
                 extra_plot_info=False):

        self.starting_position = starting_position
        self.stopping_position = stopping_position

        # Computation timing
        self.dt_sim = dt_sim
        self.dt_uc = dt_uc
        self.cntrl_counter = 0
        self.dt_ratio = dt_uc/dt_sim

        self.state = State.NomotoAugmentedIdealState(initial_state)
        self.model = Model.AugmentedNomotoIdeal(dt_sim=dt_sim, kYaw=500)

        self.u = 0.0  # control input

        self.position = np.array([[initial_state[2][0], initial_state[3][0]]])

        self.tracking_controller = CTRL.PD(kp=0.05, kd=0.99,
                                           max_act_lim=10000.0,
                                           min_act_lim=-10000.0)

        self.k_sill = k_sill
        self.k_rng = k_rng

        self.threshold = threshold

        self.field = field
        self.path_planner_type = path_planner_type

        if path_planner_type == "Zig-zag":
            self.path_planner = PATH.Zigzag(
                field=field,
                starting_position=starting_position,
                stopping_position=stopping_position,
                node_separation=node_separation)

        elif path_planner_type == "Myopic":
            self.path_planner = PATH.Myopic(
                field=field,
                look_int_dist=15,
                starting_position=starting_position,
                stopping_position=stopping_position)

        elif path_planner_type == "Random":
            self.path_planner = PATH.RandomizedWaypoints(field=field)

        elif path_planner_type == "HV":
            self.path_planner = PATH.HighestVariance(field=field)

        elif path_planner_type == "HV-Bellman-Ford":
            self.path_planner = PATH.BellmanFord(
                field=field, distance_weight=distance_weight,
                node_separation=node_separation, threshold=threshold,
                starting_position=starting_position,
                stopping_position=stopping_position)

        else:
            raise ValueError("Nomoto.py: Invalid path planner!")

        self.sigma_w = sigma_w

        self.est_update_interval = estimate_interval  # every 'N' measurements
        self.vario_update_interval = vario_interval  # every 'N' measurements
        self.est_updt_meas_cnt = 0

        # Number of measurements and estimate update counter
        self.est_updt_meas_cnt_last = 0
        self.vario_updt_meas_cnt_last = 0

        self.Zhat = np.zeros((field.get_yLen(), field.get_xLen()))
        self.V_Zhat = np.ones((field.get_yLen(), field.get_xLen()))*100
        self.Zhat_2sigma_hi = np.zeros((field.get_yLen(), field.get_xLen()))
        self.Zhat_2sigma_lo = np.zeros((field.get_yLen(), field.get_xLen()))

        self.path_angle = 0.0
        self.clst_pt_en = np.zeros((1, 2))
        self.cte = 0.0
        self.correction_threshold = correction_threshold
        self.new_est_flag = False
        self.new_est_flag_internal = False

        self.spatial_est_type = spatial_est_type

        self.max_recursion_level = max_recursion_level
        self.min_recursion_level = 1

        self.l_max = 1  # Default max level of recursion

        # Variogram stuff
        self.maxlag = 50
        self.lagsize = 0.25
        self.lagvec = np.arange(0, self.maxlag, self.lagsize)
        self.lagvec = np.reshape(self.lagvec, (len(self.lagvec), 1))
        nLags = len(self.lagvec)

        # Gaussian Process Regression and Partitioned Gaussian Process
        # Regression
        if (spatial_est_type == "GPR") or (spatial_est_type == "PGPR"):

            self.dynamic_hyperparams = dynamic_hyperparams

            self.spatial_estimator = GP.TwoDimensional(m_field=field.get_yLen(),
                                                       n_field=field.get_xLen(),
                                                       sigma_f=1.0,
                                                       ell=5.5,
                                                       sigma_w=0.1,
                                                       rlvl=max_recursion_level,
                                                       type=spatial_est_type)

            if self.dynamic_hyperparams or (spatial_est_type == "PGPR"):
                x = starting_position[0][0]
                y = starting_position[0][1]
                self.kriging_zxy = np.array([[0.0, x, y]])
                self.variogrammer = KG.OrdinaryKriging(field=field,
                                                       tol=field.ds, 
                                                       nLags=nLags)

        # Ordinary Kriging and Partitioned Ordinary Kriging
        elif ((spatial_est_type == "OK") or (spatial_est_type == "IIOK") or
              (spatial_est_type == "POK")):

            self.dynamic_hyperparams = False  # This only applies to GPR/PGPR

            x = starting_position[0][0]
            y = starting_position[0][1]
            self.kriging_zxy = np.array([[0.0, x, y]])

            if (spatial_est_type == "OK"):
                self.spatial_estimator = KG.OrdinaryKriging(field=field,
                                                            tol=field.ds,
                                                            nLags=nLags)
            elif (spatial_est_type == "IIOK"):
                self.spatial_estimator = KG.OrdinaryKriging(field=field,
                                                            tol=field.ds,
                                                            nLags=nLags)

            elif (spatial_est_type == "POK"):
                self.spatial_estimator = KG.PartitionedKriging(
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

        [iqy, iqx] = self.field.getTrueFieldIndices(starting_position[0][1],
                                                    starting_position[0][0])

        self.trajectory = LN.Linear(vehiPT=self.position,
                                    prevWP=starting_position,
                                    nextWP=self.path_planner.get_wp_next_en(
                                        iqy, iqx,
                                        self.V_Zhat,
                                        self.new_est_flag_internal,
                                        heading_angle=self.state.get_state_psi()))
        # self.trajectory.setNextWaypoint(self.path_planner.get_wp_next_en())

        self.t_spat_est_comp_last = 0
        self.t_log_path_plan = []
        self.t_log_path_plan_num_wps = []
        self.num_wps_reached = 0
        self.t_log_spat_est = []
        self.t_log_spat_est_num_pts = []

        self.extra_plot_info = extra_plot_info

        print("self.spatial_est_type: {}".format(self.spatial_est_type))
        print("self.path_planner_type: {}".format(self.path_planner_type))

        self.empirical_formed = False

    def update(self, field=FD.Field()):

        #######################################################################
        # Computation
        if np.mod(self.cntrl_counter, self.dt_ratio) == 0:

            ###################################################################
            # 'Sensor" measurements
            self.position[0][0] = self.state.get_state_x()
            self.position[0][1] = self.state.get_state_y()

            angle_error = self.state.get_state_psi() - self.path_angle

            # Prevent turning the wrong way by making sure the angle error is
            # wrapped to [-pi, pi]
            angle_error = (angle_error + np.pi) % (2.0 * np.pi) - np.pi

            ###################################################################
            # Can a new field measurement be taken?
            y = self.position[0][1]
            x = self.position[0][0]
            [iqy, iqx] = field.getTrueFieldIndices(y, x)
            # [iqy, iqx] = field.quantizePosition(y, x)
            q = [iqy, iqx]

            # Gaussian Process Regression and Partitioned Gaussian Process
            # Regression
            if ((self.spatial_est_type == "GPR") or
                    (self.spatial_est_type == "PGPR")):

                if not (q in self.spatial_estimator.get_observed_yxs().tolist()):
                    self.spatial_estimator.update_observed_yxs(iqy, iqx)

                    z = field.getTrueFieldValue(y, x)
                    z += self.sigma_w * np.random.randn()

                    self.spatial_estimator.update_observed_zs(z)

                    self.est_updt_meas_cnt += 1

                # If we are using dynamic hyperparameters based on the
                # variogram
                if (self.dynamic_hyperparams or
                    (self.spatial_est_type == "PGPR")):

                    qy, qx = field.quantizePosition(y, x)
                    qxy = [qx, qy]

                    if not (qxy in self.kriging_zxy[:, 1:].tolist()):
                        z = field.getTrueFieldValue(y, x)
                        z += self.sigma_w * np.random.randn()

                        zxy = np.array([[z, qx, qy]])

                        self.kriging_zxy = np.concatenate(
                            (self.kriging_zxy, zxy), axis=0)

            # Ordinary Kriging, Iterative Inverse Ordinary Kriging and
            # Partitioned Ordinary Kriging
            elif ((self.spatial_est_type == "OK") or
                    (self.spatial_est_type == "IIOK") or
                    (self.spatial_est_type == "POK")):

                qy, qx = field.quantizePosition(y, x)
                qxy = [qx, qy]

                if not (qxy in self.kriging_zxy[:, 1:].tolist()):
                    z = field.getTrueFieldValue(y, x)
                    z += self.sigma_w * np.random.randn()

                    zxy = np.array([[z, qx, qy]])

                    self.kriging_zxy = np.concatenate(
                        (self.kriging_zxy, zxy), axis=0)

                    self.est_updt_meas_cnt += 1

            ###################################################################
            # Is it time to update the empirical variogram
            # Currently setup to update once, but can be changed for multiple
            # updates. Keep in mind that it is computationally expensive to
            # update the empirical variogram
            if ((np.mod(self.est_updt_meas_cnt, self.vario_update_interval) == 0)
                    and (self.vario_updt_meas_cnt_last != self.est_updt_meas_cnt)
                    and (self.empirical_formed == False)):

                # Prevent repeat variograms within a single interval
                self.vario_updt_meas_cnt_last = self.est_updt_meas_cnt

                ###############################################################
                # GPR general cases
                if ((self.spatial_est_type == "GPR") or
                        (self.spatial_est_type == "PGPR")):
                        
                    if (self.dynamic_hyperparams or
                            (self.spatial_est_type == "PGPR")):

                        E = self.variogrammer.makeEmpericalSemiVariogram(
                            self.kriging_zxy, lag_vec=self.lagvec)

                        sill, _, _ = self.variogrammer.fit2EmpiricalScipy(
                            self.lagvec[:len(E)], E)

                        f = self.variogrammer.variogramGauss(self.lagvec)
                        rng = 1
                        for lag, fl in zip(self.lagvec, f):
                            if fl >= 0.95*sill:
                                rng = lag[0]
                                break

                        # Set the hyperparameters:
                        # 1) the common length scale
                        # 2) the variance scale factor
                        self.spatial_estimator.set_ell(self.k_rng*rng)
                        self.spatial_estimator.set_sigma_f(self.k_sill*sill)

                        
                        #
                        #
                        #
                        #
                        # Get rid of this vvvvvv used for debugging
                        # f = self.spatial_estimator.variogramGauss(self.lagvec)

                        
                        # rng = 1
                        # for lag, fl in zip(self.lagvec, f):
                        #     if fl >= 0.95*sill:
                        #         rng = lag[0]
                        #         break

                        #
                        #
                        #
                        #
                        #
                        


                ###############################################################
                # OK, IIOK, and POK variogram cases
                if ((self.spatial_est_type == "OK") or
                        (self.spatial_est_type == "IIOK") or
                        (self.spatial_est_type == "POK")):

                    # Global variogram for whole field
                    if ((self.spatial_est_type == "OK") or
                            (self.spatial_est_type == "IIOK")):

                        # print(self.kriging_zxy)

                        E = self.spatial_estimator.makeEmpericalSemiVariogram(
                            self.kriging_zxy, lag_vec=self.lagvec)

                        # print(E)

                    elif (self.spatial_est_type == "POK"):
                        E = self.spatial_estimator.makeEmpericalSemiVariogram(
                            sensorLog=self.kriging_zxy,
                            endlevel=1)

                    # Fit the global empirical variogram
                    sill, _, _ = self.spatial_estimator.fit2EmpiricalScipy(
                        self.lagvec[:len(E)], E)

                ###############################################################
                # Special Cases for recursive partition of the field into
                # subfields for PGPR and POK
                if ((self.spatial_est_type == "PGPR") or
                        (self.spatial_est_type == "POK")):

                    if (self.spatial_est_type == "PGPR"):
                        f = self.variogrammer.variogramGauss(self.lagvec)

                    elif (self.spatial_est_type == "POK"):
                        f = self.spatial_estimator.variogramGauss(self.lagvec)

                    rng = 1
                    for lag, fl in zip(self.lagvec, f):
                        if fl >= 0.95*sill:
                            rng = lag[0]
                            break
                    
                    
                    #
                    #
                    #
                    #
                    #
                    # print(E)

                    # fig = plt.figure(figsize=(5, 3.5))
                    # plt.title('Fit to Variogram')
                    # plt.scatter(self.lagvec, E, label='Empirical Variogram')
                    # plt.plot(self.lagvec, f, label='LS Gaussian Fit', color='purple')
                    # plt.plot([rng, rng], [0.0, sill], linestyle='--', color='red',
                    #         label='range')
                    # plt.plot([0, self.maxlag], [sill, sill], linestyle='--', color='tab:blue',
                    #         label='sill')
                    # plt.xlabel('Lags (meters)')
                    # plt.ylabel('Variance (meters)')
                    # plt.grid()
                    # plt.legend()
                    # plt.show()
                    #
                    #
                    #
                    #
                    #

                    print("rng: {}\n".format(rng))

                    qx = self.field.get_width()
                    qy = self.field.get_length()

                    self.l_max = (
                        int(np.floor(np.sqrt((qx**2 + qy**2))/(rng))) - 1)

                    self.l_max = np.min([self.max_recursion_level, self.l_max])

                    print('    self.l_max (a) = {}'.format(self.l_max))

                    if self.l_max < self.min_recursion_level:
                        self.l_max = self.min_recursion_level
                        print('    (LOW) self.l_max={}'.format(self.l_max))

                    if self.l_max > self.max_recursion_level:
                        self.l_max = self.max_recursion_level
                        print('    (HIGH) self.l_max={}'.format(self.l_max))

                    print('    self.l_max (b) = {}'.format(self.l_max))

                    # Recursion level cases for PGPR and POK

                    if (self.spatial_est_type == "PGPR"):
                        # Set the hyperparameters:
                        # 1) the common length scale
                        # 2) the variance scale factor
                        self.spatial_estimator.set_ell(self.k_rng*rng)
                        self.spatial_estimator.set_sigma_f(self.k_sill*sill)
                        self.spatial_estimator.set_rlvl(self.l_max)

                    elif (self.spatial_est_type == "POK"):

                        # Local variogram from immediate sub-field
                        E = self.spatial_estimator.makeEmpericalSemiVariogram(
                            sensorLog=self.kriging_zxy,
                            endlevel=self.l_max)

                        # Fit the local variogram from immediate sub-field
                        self.spatial_estimator.fit2EmpiricalScipy(
                            self.lagvec[:len(E)], E)

                #
                self.empirical_formed = True

            ###################################################################
            # Is it time to update the spatial estimate?
            if ((np.mod(self.est_updt_meas_cnt, self.est_update_interval) == 0)
                    and (self.est_updt_meas_cnt_last != self.est_updt_meas_cnt)
                    and self.empirical_formed):

                # Prevent repeat estimates within a single interval
                self.est_updt_meas_cnt_last = self.est_updt_meas_cnt

                if self.est_updt_meas_cnt > 0:

                    # Gaussian Process Regression and Partitioned Gaussian
                    # Process Regression
                    if ((self.spatial_est_type == "GPR") or
                            (self.spatial_est_type == "PGPR")):

                        ######################################################
                        ######################################################
                        ######################################################
                        # RECORD THE TIME IT TAKES TO ESTIMATE
                        self.spatial_estimator.find_unobserved_yxs()

                        t_est_a = time.perf_counter()

                        self.Zhat, self.V_Zhat = self.spatial_estimator.predict()

                        t_est_b = time.perf_counter()

                        self.t_spat_est_comp_last = (copy.deepcopy(t_est_b)
                                                     - copy.deepcopy(t_est_a))

                        if self.extra_plot_info:
                            self.t_log_spat_est.append(
                                self.t_spat_est_comp_last)
                            self.t_log_spat_est_num_pts.append(
                                copy.deepcopy(self.est_updt_meas_cnt))
                        ######################################################
                        ######################################################
                        ######################################################

                    # Ordinary Kriging and Partitioned Ordinary Kriging
                    if ((self.spatial_est_type == "OK") or
                        (self.spatial_est_type == "IIOK") or
                            (self.spatial_est_type == "POK")):

                        ######################################################
                        ######################################################
                        ######################################################
                        # RECORD THE TIME IT TAKES TO ESTIMATE
                        t_est_a = time.perf_counter()

                        # Ordinary Kriging (OK)
                        if self.spatial_est_type == "OK":

                            C = self.spatial_estimator.calcCovMatFromVario(
                                sensorLog=self.kriging_zxy)

                            # Cinv = self.spatial_estimator.getInvCovarianceMatrix()

                            self.Zhat, self.V_Zhat = self.spatial_estimator.predict(
                                C, sensorLog=self.kriging_zxy)

                        # Iterative Inverse Ordinary Kriging (IIOK)
                        elif (self.spatial_est_type == "IIOK"):

                            # maxlag = 200
                            # lagsize = 1
                            # lagvec = np.arange(0, maxlag, lagsize)
                            # lagvec = np.reshape(lagvec, (len(lagvec), 1))

                            Cinv = self.spatial_estimator.getInvCovarianceMatrix()

                            C = self.spatial_estimator.calcCovMatFromVario(
                                sensorLog=self.kriging_zxy)

                            # # Fit the local variogram from immediate sub-field
                            # self.spatial_estimator.fit2EmpiricalScipy(
                            #     lagvec[:len(E)], E)

                            self.Zhat, self.V_Zhat = self.spatial_estimator.predict1Step(
                                C=C, Cinv=Cinv, sensorLog=self.kriging_zxy,
                                field=self.field)

                            # print("self.Zhat:\n{}".format(self.Zhat))

                        # Partitioned Ordinary Kriging (POK)
                        elif (self.spatial_est_type == "POK"):

                            C = self.spatial_estimator.calcCovMatFromVario(
                                self.kriging_zxy, endlevel=self.l_max)

                            # Cinv = self.spatial_estimator.getInvCovarianceMatrix()

                            self.Zhat, self.V_Zhat = self.spatial_estimator.predict(
                                C=C,
                                sensorLog=self.kriging_zxy,
                                endlevel=self.l_max,
                                x_field=self.field.get_x_vector(),
                                y_field=self.field.get_y_vector())

                        t_est_b = time.perf_counter()

                        self.t_spat_est_comp_last = (copy.deepcopy(t_est_b)
                                                     - copy.deepcopy(t_est_a))

                        if self.extra_plot_info:
                            self.t_log_spat_est.append(
                                self.t_spat_est_comp_last)
                            self.t_log_spat_est_num_pts.append(
                                copy.deepcopy(self.est_updt_meas_cnt))
                        ######################################################
                        ######################################################
                        ######################################################

                    self.new_est_flag = True
                    self.new_est_flag_internal = True

            ###################################################################
            # Is it time to update the 'previous' and 'next' waypoints?
            if (self.trajectory.is_closest_point_near_next_wp(self.threshold)
                or (self.trajectory.is_closest_point_beyond_next(
                    self.threshold))):
                wp_next_en_old = self.trajectory.getNextWaypoint()

                ######################################################
                ######################################################
                ######################################################
                # RECORD THE TIME IT TAKES TO PLAN A PATH
                t_old = time.perf_counter()
                wp_next_en = self.path_planner.get_wp_next_en(
                    iqy, iqx,
                    self.V_Zhat,
                    self.new_est_flag_internal,
                    heading_angle=self.state.get_state_psi())
                t_new = time.perf_counter()

                t_path_plan = (copy.deepcopy(t_new) - copy.deepcopy(t_old))

                if self.extra_plot_info:
                    self.t_log_path_plan.append(t_path_plan)

                    self.num_wps_reached += 1
                    self.t_log_path_plan_num_wps.append(
                        copy.deepcopy(self.num_wps_reached))
                ######################################################
                ######################################################
                ######################################################

                self.new_est_flag_internal = False

                self.trajectory.setNextWaypoint(wp_next_en)

                if (np.linalg.norm(wp_next_en-wp_next_en_old) > 1.0):
                    self.trajectory.setPreviousWaypoint(wp_next_en_old)

            # Trajectory logic
            self.cte = self.trajectory.getCte()
            # prev_wp = self.trajectory.getPreviousWaypoint()
            # next_wp = self.trajectory.getNextWaypoint()
            # if ((np.linalg.norm(self.position-prev_wp) >=
            #         self.correction_threshold) and
            #         (np.linalg.norm(self.position-next_wp) >=
            #         self.correction_threshold)):
            #     print("***corrected***")
            #     self.trajectory.setPreviousWaypoint(self.position)

            self.trajectory.udpate(self.position)

            self.path_angle = self.trajectory.getPathAngle()
            self.clst_pt_en = self.trajectory.getClosestPoint()

            ###################################################################
            # Update Controller
            acc_cmd = self.tracking_controller.update(0, self.cte, angle_error)

            usign = np.sign(acc_cmd)
            if acc_cmd != 0.0:
                r_temp = (self.state.get_state_v()**2)/np.abs(acc_cmd)
                if r_temp != 0.0:
                    self.u = usign * \
                        (np.sqrt(np.abs(acc_cmd) / r_temp) * self.dt_uc)

        #######################################################################
        # Model
        x = self.state.get_state_vector()

        x = self.model.update(x, self.u)

        self.state.set_state_vector(x)

        #######################################################################
        # Computation interval count
        self.cntrl_counter += 1

        return self.path_planner.is_at_last_waypoint(self.position,
                                                     self.threshold)

    def get_heading_angle(self):
        return self.state.get_state_psi()

    def get_path_angle(self):
        return self.path_angle

    def get_yaw_rate(self):
        return self.state.get_state_yaw_rate()

    def get_x(self):
        return self.state.get_state_x()

    def get_y(self):
        return self.state.get_state_y()

    def get_v(self):
        return self.state.get_state_v()

    def get_u(self):
        return self.u

    def get_waypoints(self):
        return self.path_planner.get_wp_queue()

    def get_prev_wp(self):
        return self.trajectory.getPreviousWaypoint()

    def get_next_wp(self):
        return self.trajectory.getNextWaypoint()

    def get_candidate_wps(self):
        return self.path_planner.get_candidate_wps()

    def get_selected_next_wp(self):
        return self.path_planner.get_slct_cnd_wp()

    def get_Zhat(self):
        return self.Zhat

    def get_V_Zhat(self):
        return self.V_Zhat

    def get_Zhat_2sigma_hi(self):
        return self.Zhat_2sigma_hi

    def get_Zhat_2sigma_lo(self):
        return self.Zhat_2sigma_lo

    def get_unobserved_yxs(self):
        return self.spatial_estimator.get_unobserved_yxs()

    def get_mu(self):
        return self.spatial_estimator.get_mu()

    def get_hi(self):
        return self.spatial_estimator.get_hi()

    def get_lo(self):
        return self.spatial_estimator.get_lo()

    def get_number_of_points_measured(self):
        return self.est_updt_meas_cnt_last

    def is_there_a_new_Zhat(self):
        status = self.new_est_flag
        self.new_est_flag = False
        return status

    def get_observations_yxs(self):
        observed_yxs = np.zeros((self.est_updt_meas_cnt, 2))
        for i in range(self.est_updt_meas_cnt):
            iy = int(self.spatial_estimator.get_observed_yxs()[i, 0])
            ix = int(self.spatial_estimator.get_observed_yxs()[i, 1])

            xy = self.field.getTrueFieldPointCoordinate(iy, ix)

            # Y
            observed_yxs[i][0] = xy[0][1]

            # x
            observed_yxs[i][1] = xy[0][0]

        return observed_yxs

    def get_path_planner_type(self):
        return self.path_planner_type

    def get_spatial_est_type(self):
        return self.spatial_est_type

    def get_t_spat_est_comp_last(self):
        return self.t_spat_est_comp_last

    def get_t_log_path_plan(self):
        return self.t_log_path_plan

    def get_t_log_path_plan_num_wps(self):
        return self.t_log_path_plan_num_wps

    def get_t_log_spat_est(self):
        return self.t_log_spat_est

    def get_t_log_spat_est_num_pts(self):
        return self.t_log_spat_est_num_pts

    def get_arrows(self):
        if self.path_planner_type == "HV-Bellman-Ford":
            return self.path_planner.get_arrows()

    def get_sol_arrows(self):
        if self.path_planner_type == "HV-Bellman-Ford":
            return self.path_planner.get_sol_arrows()
