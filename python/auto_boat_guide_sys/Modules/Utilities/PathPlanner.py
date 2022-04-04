"""
.. module:: PathPlanner.py
    :platform: MacOS, Unix, Windows,
    :synopsis: A class for path planners
.. moduleauthor:: Pavlo Vlastos <pvlastos@ucsc.edu>
"""
from re import X
from tracemalloc import stop
from ..Utilities import Graph as Graph
from ..Waypoints import WaypointQueue as WQ
from ..Field import Field as Field
import numpy as np
from numpy import unravel_index


class TemplatePlanner():
    def __init__(self, field=Field.Field(), distance_weight=0.0,
                 node_separation=3, starting_position=np.zeros((1, 2)),
                 stopping_position=np.zeros((1, 2)), threshold=2.5):
        node_matrix = Graph.make_node_matrix(m=field.get_yLen(),
                                             n=field.get_xLen())
        self.grapher = Graph.Graph(num_vertices=node_matrix,
                                   distance_weight=distance_weight)

        self.wp_queue = WQ.WaypointQueue(threshold=threshold)
    # def get_wp_next_en(self, iy=0, ix=0, V_Zhat=None, is_new_est=False):


class Myopic():
    def __init__(self, field=Field.Field(), look_int_dist=20,
                 starting_position=np.zeros((1, 2)),
                 stopping_position=np.zeros((1, 2)), threshold=2.5, npts=10,
                 va=290*np.pi/180.0):

        self.starting_position = starting_position
        self.stopping_position = stopping_position

        self.field = field
        self.V_Zhat = np.zeros((field.get_yLen(), field.get_xLen()))
        self.m, self.n = self.V_Zhat.shape

        self.threshold = threshold
        self.wp_queue = WQ.WaypointQueue(threshold=threshold)
        self.wp_queue.add(self.starting_position)

        iy, ix = self.field.getTrueFieldIndices(
            starting_position[0][1],
            starting_position[0][0])

        self.start_point = [ix, iy]
        self.past_points = [self.start_point]

        iy, ix = self.field.getTrueFieldIndices(
            stopping_position[0][1],
            stopping_position[0][0])

        self.stop_point = [ix, iy]

        self.slct_cnd_wp = self.starting_position

        self.last_wp = self.field.getTrueFieldPointCoordinate(
            self.stop_point[1], self.stop_point[0])

        self.look_int_dist = look_int_dist
        self.npts = npts
        self.va = va
        self.angle_range = np.arange(-self.va/2,
                                     self.va/2, self.va/(self.npts))
        self.candidate_wps = np.zeros((self.npts, 3))  # [x y variance]

    def get_wp_next_en(self, iy=0, ix=0, V_Zhat=None, is_new_est=False,
                       heading_angle=0):
        if is_new_est:
            self.V_Zhat = V_Zhat

        position = self.field.getTrueFieldPointCoordinate(iy, ix)

        self.candidate_wps.fill(0.0)

        i = 0
        for angle in self.angle_range:
            cpt = np.array([[0.0, self.look_int_dist]])
            delta_angle = (angle + heading_angle)
            c = np.cos(delta_angle)
            s = np.sin(delta_angle)
            R = np.array([[c, -s],
                          [s, c]])
            cpt = cpt @ R

            # Place back
            cpt += position  # Place back

            # Ensure that the candidate points are in bounds
            [y, x] = self.field.quantizePosition(cpt[0][1], cpt[0][0])

            i_y, i_x = self.field.getTrueFieldIndices(y, x)

            i_y = int(i_y)
            i_x = int(i_x)

            # print("i_y = {}".format(i_y))
            # print("i_x = {}".format(i_x))

            # Edge Case: Does the point already exist in the candidate points?
            if [x, y] not in self.candidate_wps[:, :2].tolist():

                # weight the points based on distance to stay in bounds
                # * (np.linalg.norm(cpt-vpt)
                variance = (self.V_Zhat[i_y, i_x])-np.abs(angle)
                # / self.look_int_dist))

                # if len(variance) == 0:
                #     variance = 0
                # if (np.linalg.norm(cpt) <= self.look_int_dist):
                self.candidate_wps[i, 0] = x
                self.candidate_wps[i, 1] = y
                self.candidate_wps[i, 2] = variance
                # print("delta_angle = {}".format(round(delta_angle*180.0/np.pi)))
                # print("cpt = {}".format(cpt))

                # else:
                #     self.candidate_wps[i, 2] = -10000

            # Have we already visited this point? If so, the variance is 0
            if [x, y] in self.past_points:
                variance = 0.0

                self.candidate_wps[i, 2] = variance

            i += 1

        # Find the maximum varaince point
        iv = np.argmax(self.candidate_wps[:, 2])

        next_wp = np.zeros((1, 2))

        next_wp[0][0] = self.candidate_wps[iv][0]  # x
        next_wp[0][1] = self.candidate_wps[iv][1]  # y

        self.past_points.append([next_wp[0][0], next_wp[0][1]])
        self.slct_cnd_wp = next_wp

        # This path planner does not use the queue in the same way as the others
        self.wp_queue.add(next_wp)

        return next_wp

    def is_at_last_waypoint(self, position=np.zeros((1, 2)), threshold=2.5):
        return False  # (np.linalg.norm(self.last_wp-position) <= threshold)

    def get_wp_queue(self):
        return self.wp_queue.get_waypoints()

    def get_candidate_wps(self):
        return self.candidate_wps[:, :2]

    def get_slct_cnd_wp(self):
        return self.slct_cnd_wp

    def move_wp(self, ox, oy, angle):
        self.wp_queue.move_waypoints(ox, oy, angle)


class RandomizedWaypoints():
    def __init__(self, field=Field.Field()):
        np.random.seed(42)

        self.field = field
        self.previously_visited = []

        return

    def get_wp_next_en(self, iy=0, ix=0, V_Zhat=None, is_new_est=False,
                       heading_angle=0):
        irx = np.random.choice(range(self.field.get_xLen()))
        iry = np.random.choice(range(self.field.get_yLen()))

        while ([irx, iry] in self.previously_visited):
            irx = np.random.choice(range(self.field.get_xLen()))
            iry = np.random.choice(range(self.field.get_yLen()))

        self.previously_visited.append([irx, iry])

        next_wp = self.field.getTrueFieldPointCoordinate(iry, irx)

        return next_wp

    def is_at_last_waypoint(self, position=np.zeros((1, 2)), threshold=2.5):
        return False  # (np.linalg.norm(self.last_wp-position) <= threshold)

    def get_wp_queue(self):
        return self.wp_queue.get_waypoints()

    def move_wp(self, ox, oy, angle):
        self.wp_queue.move_waypoints(ox, oy, angle)


class HighestVariance():
    def __init__(self, field=Field.Field()):

        self.field = field
        self.to_visit_points = []

        return

    def get_wp_next_en(self, iy=0, ix=0, V_Zhat=None, is_new_est=False,
                       heading_angle=0):

        m, n = V_Zhat.shape

        imx = np.random.choice(range(self.field.get_xLen()))
        imy = np.random.choice(range(self.field.get_yLen()))
        v_max = 0

        if is_new_est:
            for i in range(m):
                for j in range(n):
                    if ((V_Zhat[i][j] > v_max) and
                            (not ([imx, imy] in self.to_visit_points))):
                        imy = i
                        imx = j
                        v_max = V_Zhat[i][j]

            self.to_visit_points.append([imx, imy])

        next_wp = self.field.getTrueFieldPointCoordinate(imy, imx)

        return next_wp

    def is_at_last_waypoint(self, position=np.zeros((1, 2)), threshold=2.5):
        return False  # (np.linalg.norm(self.last_wp-position) <= threshold)

    def get_wp_queue(self):
        return self.wp_queue.get_waypoints()

    def move_wp(self, ox, oy, angle):
        self.wp_queue.move_waypoints(ox, oy, angle)


class Zigzag():
    def __init__(self, field=Field.Field(), distance_weight=0.0,
                 node_separation=3, starting_position=np.zeros((1, 2)),
                 stopping_position=np.zeros((1, 2)), threshold=2.5):

        self.starting_position = starting_position
        self.stopping_position = stopping_position

        # print("self.starting_position: {}".format(self.starting_position))
        # print("self.stopping_position: {}".format(self.stopping_position))

        self.field = field

        self.wp_queue = WQ.WaypointQueue(threshold=threshold)

        node_matrix = Graph.make_node_matrix(m=field.get_yLen(),
                                             n=field.get_xLen())

        # print("node_matrix.size: {}".format(node_matrix.size))

        self.grapher = Graph.Graph(
            num_vertices=node_matrix.size,
            distance_weight=distance_weight,
            variance_matrix=np.ones((field.get_yLen(), field.get_xLen())))

        iy, ix = self.field.getTrueFieldIndices(
            starting_position[0][1],
            starting_position[0][0])

        self.start_point = [ix, iy]
        # print("self.start_point: {}".format(self.start_point))

        iy, ix = self.field.getTrueFieldIndices(
            stopping_position[0][1],
            stopping_position[0][0])

        self.stop_point = [ix, iy]

        # print("self.stop_point: {}".format(self.stop_point))

        [self.arrows_x,
         self.arrows_y,
         self.arrows_dx,
         self.arrows_dy] = self.grapher.generate_dag(self.stop_point,  self.start_point,
                                                     nth=node_separation)

        src_node, dest_node = self.grapher.get_src_and_dest_node(
            self.stop_point, self.start_point, nth=node_separation)

        [self.sol_arrows_x,
         self.sol_arrows_y,
         self.sol_arrows_dx,
         self.sol_arrows_dy] = self.grapher.generate_optimal_path(
            src_node, dest_node,
            self.arrows_x, self.arrows_y,
            self.arrows_dx, self.arrows_dy)

        for ax, ay, in zip(self.sol_arrows_x, self.sol_arrows_y):
            new_point = self.field.getTrueFieldPointCoordinate(ay, ax)
            self.wp_queue.add(new_point)

        self.last_wp = self.field.getTrueFieldPointCoordinate(
            self.stop_point[1], self.stop_point[0])

        self.wp_queue.add(self.last_wp)

        # print("self.wp_queue.get_waypoints() = {}".format(
        #     self.wp_queue.get_waypoints()))

    def get_wp_next_en(self, iy=0, ix=0, V_Zhat=None, is_new_est=False,
                       heading_angle=0):
        """
        NOTE: extra arguments added on purpose
        """
        return self.wp_queue.getNext()

    def is_at_last_waypoint(self, position=np.zeros((1, 2)), threshold=2.5):
        return (np.linalg.norm(self.last_wp-position) <= threshold)

    def get_wp_queue(self):
        return self.wp_queue.get_waypoints()

    def move_wp(self, ox, oy, angle):
        self.wp_queue.move_waypoints(ox, oy, angle)

class BellmanFord():
    def __init__(self, field=Field.Field(), distance_weight=1.0,
                 node_separation=3, starting_position=np.zeros((1, 2)),
                 stopping_position=np.zeros((1, 2)), threshold=2.5,
                 index_proximity=5):

        self.starting_position = starting_position
        self.stopping_position = stopping_position

        self.field = field

        self.threshold = threshold

        self.wp_queue = WQ.WaypointQueue(threshold=threshold)

        node_matrix = Graph.make_node_matrix(m=field.get_yLen(),
                                             n=field.get_xLen())

        self.distance_weight = distance_weight

        self.grapher = Graph.Graph(
            num_vertices=node_matrix.size,
            distance_weight=distance_weight,
            variance_matrix=np.ones((field.get_yLen(), field.get_xLen())))

        iy, ix = self.field.getTrueFieldIndices(
            starting_position[0][1],
            starting_position[0][0])

        self.start_point = [ix, iy]

        iy, ix = self.field.getTrueFieldIndices(
            stopping_position[0][1],
            stopping_position[0][0])

        self.stop_point = [ix, iy]

        self.node_separation = node_separation

        [self.arrows_x,
         self.arrows_y,
         self.arrows_dx,
         self.arrows_dy] = self.grapher.generate_dag(self.stop_point,  self.start_point,
                                                     nth=node_separation)

        src_node, dest_node = self.grapher.get_src_and_dest_node(
            self.stop_point, self.start_point, nth=node_separation)

        [self.sol_arrows_x,
         self.sol_arrows_y,
         self.sol_arrows_dx,
         self.sol_arrows_dy] = self.grapher.generate_optimal_path(
            src_node, dest_node,
            self.arrows_x, self.arrows_y,
            self.arrows_dx, self.arrows_dy)

        self.wp_queue.add(self.starting_position)

        # print("**********************")
        i = 0
        for ax, ay, in zip(self.sol_arrows_x, self.sol_arrows_y):
            new_wp = self.field.getTrueFieldPointCoordinate(ay, ax)
            self.wp_queue.add(new_wp)
            if i == 1:
                self.new_wp = new_wp
            i += 1
        #     print("        new_wp: {}".format(new_wp))
        # print("**********************")

        # self.last_wp = self.field.getTrueFieldPointCoordinate(
        #     self.stop_point[1], self.stop_point[0])

        self.wp_queue.add(self.stopping_position)
        # self.wp_queue.getNext()
        # self.wp_queue.getNext()
        # self.wp_queue.getNext()
        # self.new_wp = self.wp_queue.getNext()
        # print("new_wp: {}".format(self.new_wp))
        # print("peak: {}".format(self.wp_queue.peak_next_waypoint()))

        self.index_proximity = index_proximity

        return

    def get_wp_next_en(self, iy=0, ix=0, V_Zhat=None, is_new_est=False,
                       heading_angle=0):
        """
        NOTE: extra arguments added on purpose
        """

        # print("*\n*\n*\n*\n")

        # if is_new_est:
        # if ((self.start_point[0] != ix) and (self.start_point[1] != iy)):

        current_position = self.field.getTrueFieldPointCoordinate(iy, ix)

        # if ((np.linalg.norm(current_position-self.stopping_position) <=
        #         self.threshold) or is_new_est):
        if (np.linalg.norm(current_position-self.stopping_position) <=
                self.threshold):

            self.start_point = [ix, iy]

            ix0 = ix - self.index_proximity
            if ix0 < 0:
                ix0 = 0

            ixf = ix + self.index_proximity
            if ixf >= self.field.get_xLen():
                ixf = self.field.get_xLen()-1

            iy0 = iy - self.index_proximity
            if iy0 < 0:
                iy0 = 0

            iyf = iy + self.index_proximity
            if iyf >= self.field.get_yLen():
                iyf = self.field.get_yLen()-1

            for iyi in range(iy0, iyf):
                for ixi in range(ix0, ixf):
                    V_Zhat[iyi][ixi] = 0.0

            iy_stop, ix_stop = np.where(V_Zhat == np.max(V_Zhat))

            if len(iy_stop) > 0:
                # print("iy_stop: {}".format(iy_stop))
                iy_stop = iy_stop[0]  # np.random.choice(iy_stop)

            if len(ix_stop) > 0:
                # print("ix_stop: {}".format(ix_stop))
                ix_stop = ix_stop[0]  # np.random.choice(ix_stop)

            if (ix == ix_stop) and (iy == iy_stop):

                print("---------")
                print("TOO CLSOE")
                print("---------")

                V_Zhat[iy_stop][ix_stop] = 0.0
                iy_stop, ix_stop = np.where(V_Zhat == np.max(V_Zhat))

                if len(iy_stop) > 0:
                    # print("iy_stop: {}".format(iy_stop))
                    iy_stop = iy_stop[0]  # np.random.choice(iy_stop)

                if len(ix_stop) > 0:
                    # print("ix_stop: {}".format(ix_stop))
                    ix_stop = ix_stop[0]  # np.random.choice(ix_stop)

            # u = np.array([[ix_stop, iy_stop]])
            # v = np.array([[ix, iy]])
            self.stop_point = [ix_stop, iy_stop]

            self.stopping_position = self.field.getTrueFieldPointCoordinate(
                iy_stop, ix_stop)

            # print("    self.start_point: {}".format(self.start_point))
            # print("    self.stop_point: {}".format(self.stop_point))
            # print("    np.max(V_Zhat): {}".format(np.max(V_Zhat)))
            # print("    V_Zhat[{}, {}]: {}".format(
            #     iy_stop, ix_stop, V_Zhat[iy_stop, ix_stop]))

            self.wp_queue = WQ.WaypointQueue(threshold=self.threshold)

            m = self.field.get_yLen()
            n = self.field.get_xLen()

            node_matrix = Graph.make_node_matrix(m=m, n=n)

            self.grapher = Graph.Graph(
                num_vertices=node_matrix.size,
                distance_weight=self.distance_weight,
                variance_matrix=V_Zhat)

            [self.arrows_x,
             self.arrows_y,
             self.arrows_dx,
             self.arrows_dy] = self.grapher.generate_dag(self.stop_point, self.start_point,
                                                         nth=self.node_separation)

            src_node, dest_node = self.grapher.get_src_and_dest_node(
                self.stop_point, self.start_point, nth=self.node_separation)

            [self.sol_arrows_x,
             self.sol_arrows_y,
             self.sol_arrows_dx,
             self.sol_arrows_dy] = self.grapher.generate_optimal_path(
                src_node, dest_node,
                self.arrows_x, self.arrows_y,
                self.arrows_dx, self.arrows_dy)

            self.new_wp = np.zeros((1, 2))
            for ax, ay, in zip(self.sol_arrows_x, self.sol_arrows_y):
                new_wp = self.field.getTrueFieldPointCoordinate(ay, ax)
                # print("        new_wp: {}".format(new_wp))
                self.wp_queue.add(new_wp)

            self.wp_queue.add(self.stopping_position)
            # self.wp_queue.getNext()
            # print("    PathPlanner.py:BellmanFord: wp queue lenth: {}".format(
            #     self.wp_queue.getTotal()))
            # print("PathPlanner.py:BellmanFord: get_wp_next_en()")
            # print("***peak: {}".format(self.wp_queue.peak_next_waypoint()))
            self.wp_queue.getNext()  # Don't use the first point
            # self.wp_queue.getNext()
            # self.wp_queue.getNext()
            # self.wp_queue.getNext()
            # self.new_wp = self.wp_queue.getNext()
            # print("***new_wp: {}".format(self.new_wp))
            # print("***peak: {}".format(self.wp_queue.peak_next_waypoint()))

        return self.wp_queue.getNext()

    def is_at_last_waypoint(self, position=np.zeros((1, 2)), threshold=2.5):
        # status = False
        # if (np.linalg.norm(self.stopping_position-position) <= threshold):
        #     print("PathPlanner.py:BellmanFord: last wp condition")
        #     status = True
        return False

    def get_wp_queue(self):
        return self.wp_queue.get_waypoints()

    def get_arrows(self):
        """
        @TODO: get rid of this later?
        """
        return self.arrows_x, self.arrows_y, self.arrows_dx, self.arrows_dy

    def get_sol_arrows(self):
        return self.sol_arrows_x, self.sol_arrows_y, self.sol_arrows_dx, self.sol_arrows_dy

    def move_wp(self, ox, oy, angle):
        self.wp_queue.move_waypoints(ox, oy, angle)
