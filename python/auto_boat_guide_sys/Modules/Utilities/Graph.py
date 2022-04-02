"""
.. module:: Graph.py
    :platform: MacOS, Unix, Windows,
    :synopsis: Graph with Bellman-Ford algorithm based on
    https://favtutor.com/blogs/bellman-ford-python and
    https://www.techiedelight.com/single-source-shortest-paths-bellman-ford-algorithm/
.. moduleauthor:: Pavlo Vlastos <pvlastos@ucsc.edu>
"""

from os import stat
import numpy as np


class Graph:

    def __init__(self, num_vertices, distance_weight=1.0,
                 variance_matrix=np.zeros((2, 2))):
        """
        :param num_vertices:
        :param distance_weight:
        :param var_matrix: variance matrix
        """
        self.V_num = num_vertices
        self.graph = []  # array of edges with weights
        self.edges = []  # array of *just* edges
        self.dist = [np.inf] * self.V_num
        self.parent = [-1] * self.V_num
        self.distance_weight = distance_weight
        self.delta_hyp = np.sqrt(distance_weight)

        self.m, self.n = variance_matrix.shape
        self.var_matrix = variance_matrix

        self.node_matrix = make_node_matrix(self.m, self.n)
        self.node_sub_matrix = make_node_matrix(self.m, self.n)

        return

    def add_edge(self, a, b, c):
        self.graph.append([a, b, c])
        self.edges.append([a, b])
        return

    def print_solution(self):
        print("Vertex distance from source")
        for k in range(self.V_num):
            print("{0}\t\t{1}".format(k, self.dist[k]))
        return

    def print_path(self, destination_vertex):
        print("path: [ ", end='')
        print_helper(self.parent, destination_vertex)
        print("]")
        return

    def bellman_ford(self, src):
        self.dist[src] = 0

        for _ in range(self.V_num - 1):
            for (a, b, c) in self.graph:
                if ((self.dist[a] != np.inf) and
                        ((self.dist[a] + c) < self.dist[b])):

                    self.dist[b] = self.dist[a] + c

                    # Set b's parent as a
                    self.parent[b] = a

        for (a, b, c) in self.graph:
            if (self.dist[a] != np.inf) and ((self.dist[a] + c) < self.dist[b]):
                print("BellmanFord: Graph contains negative cycle")
                return

        return self.dist

    def get_optimal_path(self, destination_vertex):
        path_list = []
        get_path_helper(self.parent, destination_vertex, path_list)
        return path_list

    def set_distance_weight(self, new_weight):
        self.distance_weight = new_weight

    def generate_dag(self, stop_point_indices, start_point_indices, nth=1):
        """
        :param stop_point_indices: A list with the x and y indices: [ix, iy] 
        associated with an element of a variance matrix
        :param start_point_indices: A list with the x and y indices: [ix, iy]
        associated with an element of a variance matrix
        :param nth: How many indices between points are skipped.Only make
        graphs between every nth index. NOTE: must be an odd number with 
        current implementation. TODO: find a more elegant implementation with 
        adjacency matrix
        """
        if (nth & 0x1) == 0x0:
            if nth == 0:
                nth = 1
            else:
                nth += 1
        if (nth > self.m) or (nth > self.n):
            print("Graph error! nth is too large")

        self.node_sub_matrix = get_sub_matrix(stop_point_indices,
                                         start_point_indices,
                                         self.node_matrix, nth)

        i_dir_x, i_dir_y = get_directions(stop_point_indices,
                                          start_point_indices)

        i_dir_y *= nth
        i_dir_x *= nth

        # if (i_dir_y == 0):

        # if (i_dir_x == 0):

        arrows_x = []
        arrows_y = []
        arrows_dx = []
        arrows_dy = []

        # Start from the stop point
        y_indices = list(range(0, self.m))[::nth]
        x_indices = list(range(0, self.n))[::nth]
        for iy in y_indices:
            for ix in x_indices:

                curr_node = self.node_matrix[iy, ix]

                if curr_node in self.node_sub_matrix:

                    ###########################################################
                    # Left or right
                    if ((0 <= (ix + i_dir_x)) and ((ix + i_dir_x) < self.n)
                            and (i_dir_x != 0)):
                        potential_node_x = self.node_matrix[iy, ix + i_dir_x]

                        if potential_node_x in self.node_sub_matrix:
                            potential_var = 0
                            # Sum variance along edge
                            if nth > 1:
                                sign_x = np.sign(i_dir_x)
                                xs = np.arange(ix, ix + i_dir_x + sign_x, sign_x)
                                for ix_mid in xs:
                                    potential_var += self.var_matrix[iy, ix_mid]
                            else:
                                potential_var = self.var_matrix[iy, ix + i_dir_x]
                            # print("    Added edge from {} to {}".format(
                            #     curr_node, potential_node_x))

                            # edge      from: ____, to: ____________, edge weight
                            self.add_edge(curr_node, potential_node_x,
                                          -potential_var+self.distance_weight)

                            arrows_x.append(ix)
                            arrows_y.append(iy)
                            arrows_dx.append(i_dir_x)
                            arrows_dy.append(0)

                    ###########################################################
                    # Up or down
                    if ((0 <= (iy + i_dir_y)) and ((iy + i_dir_y) < self.m)
                            and (i_dir_y != 0)):
                        potential_node_y = self.node_matrix[iy + i_dir_y, ix]

                        if potential_node_y in self.node_sub_matrix:
                            potential_var = 0
                            # Sum variance along edge
                            if nth > 1:
                                sign_y = np.sign(i_dir_y)
                                ys = np.arange(iy, iy + i_dir_y + sign_y, sign_y)
                                for iy_mid in ys:
                                    potential_var += self.var_matrix[iy_mid, ix]
                            else:
                                potential_var = self.var_matrix[iy + i_dir_y, ix]
                            # print("    Added edge from {} to {}".format(
                            #     curr_node, potential_node_y))

                            # edge      from: ____, to: ____________, edge weight
                            self.add_edge(curr_node, potential_node_y,
                                          -potential_var+self.distance_weight)

                            arrows_x.append(ix)
                            arrows_y.append(iy)
                            arrows_dx.append(0)
                            arrows_dy.append(-i_dir_y)

                    ###########################################################
                    # Diagonal towards stop point
                    if (((0 <= (ix + i_dir_x)) and ((ix + i_dir_x) < self.n)
                         and (i_dir_x != 0))
                            and ((0 <= (iy + i_dir_y)) and
                                 ((iy + i_dir_y) < self.m)
                                 and (i_dir_y != 0))):
                        potential_node_d = self.node_matrix[iy + i_dir_y,
                                                            ix + i_dir_x]

                        if potential_node_d in self.node_sub_matrix:
                            potential_var = 0
                            # Sum variance along edge
                            if nth > 1:
                                if iy < iy + i_dir_y:
                                    sign_y = 1
                                else:
                                    sign_y = -1
                                if ix < ix + i_dir_x:
                                    sign_x = 1
                                else:
                                    sign_x = -1
                                ys = np.arange(iy, iy + i_dir_y + sign_y, sign_y)
                                xs = np.arange(ix, ix + i_dir_x + sign_x, sign_x)
                                for iy_mid, ix_mid in zip(ys, xs):
                                    potential_var += self.var_matrix[iy_mid,
                                                                     ix_mid]
                            else:
                                potential_var = self.var_matrix[iy + i_dir_y,
                                                                ix + i_dir_x]
                            # print("    Added edge from {} to {}".format(
                            #     curr_node, potential_node_d))

                            # edge      from: ____, to: ____________, edge weight
                            self.add_edge(curr_node, potential_node_d,
                                          -potential_var+self.distance_weight)

                            arrows_x.append(ix)
                            arrows_y.append(iy)
                            arrows_dx.append(i_dir_x)
                            arrows_dy.append(-i_dir_y)

                    ###########################################################
                    # Unidirectional cross diagonals that flip with no cycles
                    if ((iy & 0x1) == 0x1):

                        if ((ix & 0x1) == 0x1):
                            delta_x = - i_dir_x
                            delta_y = i_dir_y
                        else:
                            delta_x = i_dir_x
                            delta_y = - i_dir_y

                    else:

                        if ((ix & 0x1) == 0x1):
                            delta_x = i_dir_x
                            delta_y = - i_dir_y
                        else:
                            delta_x = - i_dir_x
                            delta_y = i_dir_y

                    cross_diag_x = ix + delta_x
                    cross_diag_y = iy + delta_y

                    if (((i_dir_x != 0) and ((i_dir_y != 0))) and
                        ((0 <= cross_diag_x) and (cross_diag_x < self.n)) and
                            ((0 <= cross_diag_y) and (cross_diag_y < self.m))):
                        potential_node_d = self.node_matrix[cross_diag_y,
                                                            cross_diag_x]

                        if potential_node_d in self.node_sub_matrix:
                            potential_var = 0
                            # Sum variance along edge
                            if nth > 1:
                                if iy < cross_diag_y:
                                    sign_y = 1
                                else:
                                    sign_y = -1
                                if ix < cross_diag_x:
                                    sign_x = 1
                                else:
                                    sign_x = -1
                                ys = np.arange(iy, cross_diag_y + sign_y, sign_y)
                                xs = np.arange(ix, cross_diag_x + sign_x, sign_x)
                                for iy_mid, ix_mid in zip(ys, xs):
                                    potential_var += self.var_matrix[iy_mid, 
                                                                     ix_mid]
                            else:
                                potential_var = self.var_matrix[cross_diag_y,
                                                                cross_diag_x]
                            # print("    Added edge from {} to {}".format(
                            #     curr_node, potential_node_d))

                            edge = [curr_node, potential_node_d]

                            if not (edge in self.edges):

                                # edge      from: ____, to: ____________, edge
                                # weight
                                self.add_edge(curr_node, potential_node_d,
                                              -potential_var +
                                              (self.delta_hyp *
                                               self.distance_weight))

                                arrows_x.append(ix)
                                arrows_y.append(iy)
                                arrows_dx.append(delta_x)
                                arrows_dy.append(-delta_y)

        return arrows_x, arrows_y, arrows_dx, arrows_dy

    def set_variance_matrix(self, new_variance_matrix):
        m_cur, n_cur = self.var_matrix.shape
        m_new, n_new = new_variance_matrix.shape
        if m_cur == m_new and n_cur == n_new:
            self.var_matrix = new_variance_matrix
        else:
            print("************ GRAPH ERROR:set_variance_matrix ************")

    def get_src_and_dest_node(self, stop_point_indices, start_point_indices, nth=1):
        """
        :param stop_point_indices: A list with the x and y indices: [ix, iy] 
        associated with an element of a variance matrix
        :param start_point_indices: A list with the x and y indices: [ix, iy]
        associated with an element of a variance matrix
        :param nth: How many indices between points are skipped. Only make
        graphs between every nth index
        """
        if (nth > self.m) or (nth > self.n):
            print("Graph error! nth is too large")

        stop_x, stop_y, start_x, start_y = get_end_point_indices(
            stop_point_indices, start_point_indices, nth)

        # Source node
        if start_y >= self.m:
            start_y = self.m-1
        if start_x >= self.n:
            start_x = self.n-1
        src_node = self.node_matrix[start_y][start_x]

        # Destination node
        if stop_y >= self.m:
            stop_y = self.m-1
        if stop_x >= self.n:
            stop_x = self.n-1
        dest_node = self.node_matrix[stop_y][stop_x]

        return src_node, dest_node

    def generate_optimal_path(self, src_node, dest_node, arrows_x, arrows_y,
                              arrows_dx, arrows_dy):

        self.bellman_ford(src_node)

        optimal_path = self.get_optimal_path(dest_node)

        sol_arrows_x = []
        sol_arrows_y = []
        sol_arrows_dx = []
        sol_arrows_dy = []

        last_node = -1
        for curr_node in optimal_path:

            if (curr_node != src_node):

                i = 0
                for row in self.edges:

                    src = row[0]
                    dest = row[1]

                    if (last_node == src) and (curr_node == dest):
                        sol_arrows_x.append(arrows_x[i])
                        sol_arrows_y.append(arrows_y[i])
                        sol_arrows_dx.append(arrows_dx[i])
                        sol_arrows_dy.append(arrows_dy[i])

                    i += 1

            last_node = curr_node

        return sol_arrows_x, sol_arrows_y, sol_arrows_dx, sol_arrows_dy

###############################################################################
# Helper functions


def print_helper(parent, vertex):
    if vertex < 0:
        return

    print_helper(parent, parent[vertex])
    print(vertex, end=' ')


def get_path_helper(parent, vertex, path):
    if vertex < 0:
        return

    get_path_helper(parent, parent[vertex], path)
    path.append(vertex)


def make_node_matrix(m, n):
    """
    :param m: the length (number of rows) of the variance matrix
    :param n: the width (number of columns) of the variance matrix
    """
    node_matrix = np.arange(m*n)
    node_matrix.shape = (m, n)

    return node_matrix


def get_sub_matrix(stop_point_indices, start_point_indices, matrix, nth=1):

    stop_x, stop_y, start_x, start_y = get_end_point_indices(
        stop_point_indices, start_point_indices, nth)

    i_y_min = np.min([stop_y, start_y])
    i_x_min = np.min([stop_x, start_x])

    i_y_max = np.max([stop_y, start_y])
    i_x_max = np.max([stop_x, start_x])

    # Make submatrix
    sub_matrix = matrix[i_y_min:i_y_max+1, i_x_min:i_x_max+1]

    return sub_matrix


def get_directions(stop_point_indices, start_point_indices):

    stop_x, stop_y, start_x, start_y = get_end_point_indices(
        stop_point_indices, start_point_indices)

    # Direction
    y_dif = stop_y-start_y
    x_dif = stop_x-start_x

    if y_dif != 0:
        i_dir_y = int(np.sign(y_dif))  # up or down
    else:
        i_dir_y = 0

    if x_dif != 0:
        i_dir_x = int(np.sign(x_dif))  # left or right
    else:
        i_dir_x = 0

    return i_dir_x, i_dir_y


def get_end_point_indices(stop_point_indices, start_point_indices, nth=1):
    
    # Stop towards start
    if np.mod(stop_point_indices[0], nth) != 0:
        if stop_point_indices[0] < start_point_indices[0]:
            stop_point_indices[0]=(
                int(np.ceil(stop_point_indices[0]/nth))*nth)
        else:
            stop_point_indices[0]=(
                int(np.floor(stop_point_indices[0]/nth))*nth)

    if np.mod(stop_point_indices[1], nth) != 0:
        if stop_point_indices[1] < start_point_indices[1]:
            stop_point_indices[1]=(
                int(np.ceil(stop_point_indices[1]/nth))*nth)
        else:
            stop_point_indices[1]=(
                int(np.floor(stop_point_indices[1]/nth))*nth)

    # Start towards stop
    if np.mod(start_point_indices[0], nth) != 0:
        if start_point_indices[0] < stop_point_indices[0]:
            start_point_indices[0]=(
                int(np.ceil(start_point_indices[0]/nth))*nth)
        else:
            start_point_indices[0]=(
                int(np.floor(start_point_indices[0]/nth))*nth)

    if np.mod(start_point_indices[1], nth) != 0:
        if start_point_indices[1] < stop_point_indices[1]:
            start_point_indices[1]=(
                int(np.ceil(start_point_indices[1]/nth))*nth)
        else:
            start_point_indices[1]=(
                    int(np.floor(start_point_indices[1]/nth))*nth)

    stop_x = stop_point_indices[0]
    stop_y = stop_point_indices[1]

    start_x = start_point_indices[0]
    start_y = start_point_indices[1]
        
    return stop_x, stop_y, start_x, start_y

