
"""
.. module:: Partition.py
    :platform: MacOS, Unix, Windows,
    :synopsis: A library for partitioning fields for ordinary kriging
.. moduleauthor:: Pavlo Vlastos <pvlastos@ucsc.edu>
"""
from ..Field.Constants import GaussianRandomFieldConstants as GRFC
from ..Field.Gradients import GaussianRandomField as GRF
from ..Vehicle.Constants import FieldEstimatorConstants as FEC

import numpy as np

def isEstimatable(self, positions, endlevel=FEC.lowestLevel):
    """
    :param positions: np.array([east:, north:]) a Mx2
    matrix
    :param endlevel: the recursion-end level to stop recurring at (0 is
    the top-most level, and 1, 2, 3, ... m, represent  the subsequent lower
    levels)
    :return: True or False
    """
    count = 0
    firstOb = positions[0, :]

    # print("self.getQuadrant({}, {}) = {}".format(firstOb, endlevel,
    #                                              self.getQuadrant(firstOb, endlevel)))

    Q0 = self.getQuadrant(firstOb, endlevel)

    positions = positions[1:]

    for obs in positions:

        # print("self.getQuadrant({}, {}) = {}".format(obs, endlevel,
        #                                              self.getQuadrant(obs, endlevel)))

        Qi = self.getQuadrant(obs, endlevel)

        subcount = 0

        for a, b in zip(Q0, Qi):
            if a == b:
                subcount += 1

        # TODO: 4 bc quadrants, but still a magic number, so rename
        if subcount == 4:
            count += 1

    if count == len(positions):
        return True

    return False

def isPositionInQuadrant(self, position, endlevel, x0, xf, y0, yf):
    """
    :param position: np.array([[east , north]]) a position vector 1x2
    :param endlevel: the recursion-end level to stop recurring at (0 is
    the top-most level, and 1, 2, 3, ... m, represent  the subsequent lower
    levels)
    :param x0:
    :param xf:
    :param y0:
    :param yf:
    """

    if ((x0 <= position[0]) and (position[0] <= xf) and (y0 <= position[1])
            and (position[1] <= yf)):
        return True

    return False

def partition24(self, x, y):
    """
    Partition a field or sub-field into four sub-fields
    :param x: Mx1 vector of x-coordinates of a field or sub-field
    :param y: Mx1 vector of y-coordinates of a field or sub-field
    :return: 4x4 numpy matrix (array):
            Q = np.array([[q0x0, q0xf, q0y0, q0yf]
                            [q1x0, q1xf, q1y0, q1yf]
                            [q2x0, q2xf, q2y0, q2yf]
                            [q3x0, q3xf, q3y0, q3yf]])

            quadrants in physical space:
            q3 | q2
            --- ---
            q0 | q1
    """
    xn = len(x)-1
    ym = len(y)-1

    ixhalf = int(np.floor(xn/2))
    iyhalf = int(np.floor(ym/2))

    Q = np.zeros((4, 4))

    # Quadrant 0
    Q[0][0] = x[0][0]  # q0x0
    Q[0][1] = x[ixhalf]  # q0xf
    Q[0][2] = y[0][0]  # q0y0
    Q[0][3] = y[iyhalf]  # q0yf

    # Quadrant 1
    Q[1][0] = x[ixhalf+1][0]  # q1x0
    Q[1][1] = x[-1][0]  # q1xf
    Q[1][2] = y[0][0]  # q1y0
    Q[1][3] = y[iyhalf]  # q1yf

    # Quadrant 2
    Q[2][0] = x[ixhalf+1][0]  # q2x0
    Q[2][1] = x[-1]  # q2xf
    Q[2][2] = y[iyhalf+1][0]  # q2y0
    Q[2][3] = y[-1]  # q2yf

    # Quadrant 3
    Q[3][0] = x[0][0]  # q3x0
    Q[3][1] = x[ixhalf][0]  # q3xf
    Q[3][2] = y[iyhalf+1][0]  # q3y0
    Q[3][3] = y[-1][0]  # q3yf

    return Q

def getQuadrant(self, position, endlevel=FEC.lowestLevel):
    """
    :param position: np.array([[east , north]]) a position vector 1x2
    :param endlevel: the recursion-end level to stop recurring at (0 is
    the top-most level, and 1, 2, 3, ... m, represent  the subsequent lower
    levels)
    :return: The x and y coordinates of the quadrant
    """

    # binary search in x for minimum x partition coordinate
    x0, xf = self.binarySearch(position[0], 0,
                                len(self.xdivs)-1,
                                self.xdivs, 0, endlevel)

    # binary search in y for minimum y partition coordinate
    y0, yf = self.binarySearch(position[1], 0,
                                len(self.ydivs)-1,
                                self.ydivs, 0, endlevel)

    return [x0, xf, y0, yf]

def binarySearch(self, v, il, ir, divs, n, endlevel=FEC.lowestLevel):
    """
    binary search
    :param v: value to be searched with
    :param il: left index
    :param ir: right index
    :param divs: An np.array([x0, x1, x2, ..., xN-1]) representing the
    spatial divisions of a field in either the x or y-dimension.
    :param n: the current level of recursion
    :param endlevel: the recursion-end level to stop recurring at (0 is
    the top-most level, and 1, 2, 3, ... m, represent  the subsequent lower
    levels)
    :return: divs[il], divs[ir]
    """

    # Termination, or "Bottom-out" case
    if n >= endlevel:
        return [divs[il], divs[ir]]

    i = round((il+ir)/2)

    # bias towards left
    if v <= divs[i]:
        ir = i

    else:
        il = i

    return self.binarySearch(v, il, ir, divs, n+1, endlevel)