"""
.. module:: Grid.py
    :platform: MacOS, Unix, Windows,
    :synopsis: A grid class for a local tangent plane (LTP) with resepct to a
    desired initial starting position and orientation of an autonomous vehicle.
    Forms a grid with a static number of waypoints to travel to
    Dimensions are in meters.
.. moduleauthor:: Pavlo Vlastos <pvlastos@ucsc.edu>
"""

import numpy as np


class Grid():
    def __init__(self, ds=5.0, width=60.0, length=80.0, x0=-30.0, y0=10.0):
        """
        Note that note grid points will reside on the perimeter of the grid
        :param ds: Spatial separation distance between grid points in meters
        :param width: The width of the grid in meters
        :param length: The length of the grid in meters
        """
        self.ds = ds

        self.width = width
        self.length = length

        ylen = int(np.ceil(self.length/self.ds))
        xlen = int(np.ceil(self.width/self.ds))
        self.N = ylen*xlen

        self.x0 = x0
        self.y0 = y0

        self.points = np.zeros((self.N, 2))

        return None

    def form_grid(self, ds, angle):
        """
        Note that note grid points will reside on the perimeter of the grid
        :param ds: Spatial separation distance between grid points in meters
        :param angle: The orientation of the grid with respect to the LTP
        """

        xs = np.arange(0, self.width, ds)
        ys = np.arange(0, self.length, ds)

        # Form grid at origin
        i = 0
        for y in ys:
            for x in xs:
                self.points[i][0] = x
                self.points[i][1] = y
                i += 1

        # Rotate grid
        R = np.array([[np.cos(angle), -np.sin(angle)],
                      [np.sin(angle), np.cos(angle)]])

        self.points = self.points @ R  

        # Shift in x and y
        for i in range(self.N):
            self.points[i][0] += self.x0
            self.points[i][1] += self.y0

    def get_points(self):
        """
        :return: an 'm' by 2 matrix of grid points
        """
        return self.points


###############################################################################
# MODULE TEST EXAMPLE
###############################################################################
if __name__ == '__main__':
    g = Grid()
    g.form_grid(5.0, 32.0*np.pi/180.0)

    print("g.points:")
    for point in g.points:
        print("    {}".format(point))
