"""
.. module:: Field.py
	:platform: MacOS, Unix, Windows,
	:synopsis: A class for a Field
.. moduleauthor:: Pavlo Vlastos <pvlastos@ucsc.edu>
"""

import numpy as np
from ..Constants import Constants as CNST
from ..Gradients import Gradients as GD


class Field():
    def __init__(self, ds=CNST.ds, x0=CNST.x0, y0=CNST.y0, xf=CNST.xf,
                 yf=CNST.yf, mMin=CNST.mMin, mMax=CNST.mMax, kSize=CNST.kSize,
                 alpha=CNST.alpha):
        """
        :param ds: Spatial separation between surface measurements [meters]
        :param x0: The x-coordinate of the start of the gradient [meters]
        :param y0: The y-coordinate of the start of the gradient [meters]
        :param xf: The x-coordinate of the end of the gradient [meters]
        :param yf: The y-coordinate of the end of the gradient [meters]
        :return: none
        """
        self.ds = ds
        self.x0 = x0
        self.y0 = y0
        self.xf = xf
        self.yf = yf

        # Set initial bounds
        self.x0 = np.round(self.x0/self.ds)*self.ds
        self.xf = np.round(self.xf/self.ds)*self.ds
        self.y0 = np.round(self.y0/self.ds)*self.ds
        self.yf = np.round(self.yf/self.ds)*self.ds

        # Vector of x coordinates
        self.x = np.arange(self.x0, self.xf+self.ds, self.ds)
        # Vector of y coordinates
        self.y = np.arange(self.y0, self.yf+self.ds, self.ds)

        # Update bounds to ensure they are multiples of the resolution
        self.x0 = self.x[0]
        self.xf = self.x[-1]
        self.y0 = self.y[0]
        self.yf = self.y[-1]

        self.width = self.xf - self.x0
        self.length = self.yf - self.y0

        self.xLen = len(self.x)
        self.yLen = len(self.y)

        # print("    Field.py: self.xLen: {}".format(self.xLen))
        # print("    Field.py: self.yLen: {}".format(self.yLen))

        self.totalNumPointsInField = self.xLen*self.yLen
        # print("    Field.py: self.totalNumPointsInField: {}".format(self.totalNumPointsInField))

        self.gradient = GD.GaussianRandomField(kSize=kSize, mMax=mMax,
                                               mMin=mMin, xLen=self.xLen,
                                               yLen=self.yLen, alpha=alpha,
                                               x=self.x, y=self.y,
                                               x0=self.x0,
                                               xf=self.xf,
                                               y0=self.y0,
                                               yf=self.yf)

    def finilize(self, kN=CNST.kN):
        self.gradient.applyConvolutions(kN)
        return

    def get_max(self):
        return self.gradient.get_max()

    def get_min(self):
        return self.gradient.get_min()

    def get_width(self):
        return self.width

    def get_length(self):
        return self.length

    def get_resolution(self):
        return self.ds

    def get_x0(self):
        return self.x0

    def get_xf(self):
        return self.xf

    def get_y0(self):
        return self.y0

    def get_yf(self):
        return self.yf

    def get_xLen(self):
        return self.xLen

    def get_yLen(self):
        return self.yLen

    def get_x_vector(self):
        return self.x

    def get_y_vector(self):
        return self.y

    def get_number_of_points(self):
        return self.totalNumPointsInField

    def is_point_in_bounds(self, y, x):
        return ((self.x0 <= x) and (x <= self.xf) and
                (self.y0 <= y) and (y <= self.yf))

    def quantizePosition(self, y, x):
        """
        def quantizePosition(self, y, x):
        :param y: The y-coordinate of the field
        :param x: The x-coordinate of the field
        :return: The quatized position y, x
        """

        # Round x and y to nearest resolution if coordinates isn't an exact
        # multiple. Clip x and/or y if beyond field bounds

        # X
        if (self.x0 <= x) and (x <= self.xf):
            if np.mod(x, self.ds) != 0:
                x = np.round(x/self.ds)*self.ds
                # x = np.round(np.round(x/self.ds)*self.ds, 2)

        if x < self.x0:
            x = self.x0

        if x > self.xf:
            x = self.xf

        # Y
        if (self.y0 <= y) and (y <= self.yf):
            if np.mod(y, self.ds) != 0:
                y = np.round(y/self.ds)*self.ds

        if y < self.y0:
            y = self.y0

        if y > self.yf:
            y = self.yf

        return [y, x]

    def getTrueFieldValue(self, y, x):
        """
        def getTrueFieldValue(self, y, x):
        :param y: The y-coordinate of the field
        :param x: The x-coordinate of the field
        :return: The value of the field at Z[iy, ix]
        """

        # Round ix and iy to nearest resolution if coordinates isn't an exact
        # multiple. Clip ix and/or iy if beyond field bounds
        qy, qx = self.quantizePosition(y, x)

        # print('qx = {}'.format(qx))
        # print('qy = {}'.format(qy))

        ix = np.where(self.x == qx)[0][0]
        iy = np.where(self.y == qy)[0][0]

        return self.gradient.Z[iy, ix]

    def getTrueFieldIndices(self, y, x):
        """
        def getTrueFieldValue(self, y, x):
        :param y: The y-coordinate of the field
        :param x: The x-coordinate of the field
        :return: The indices of the field corresponding to the coordinates
        """
        qy, qx = self.quantizePosition(y, x)

        ix = np.where(self.x == qx)[0][0]
        iy = np.where(self.y == qy)[0][0]

        return iy, ix

    def getTrueFieldPointCoordinate(self, iy, ix):
        """
        :param iy: The y-coordinate index of the field
        :param ix: The x-coordinate index of the field
        :return: The coordinates the field at the indices: [iy, ix]
        """
        return np.array([[self.x[ix], self.y[iy]]])

    def getNoisyFieldValue(self, iy, ix):
        """
        def getNoisyFieldValue(self, iy, ix):
        get the value of the field at a location with noise
        :param iy: The y-coordinate of the field
        :param ix: The x-coordinate of the field
        :return: The measured value of the field at Z[iy, ix]
        """

        return (self.getTrueFieldValue(iy, ix)
                + np.random.normal(0.0, CNST.depthSigma))

    def setTrueFieldValue(self, iy, ix, value):
        """
        :param iy: The INDEX of the y-coordinate vector of the field
        :param ix: The INDEX of the x-coordinate vector of the field
        :param value: The true value of the field at Z[iy, ix]
        :return: None
        """

        if iy not in range(0, self.yLen):
            raise ValueError("iy: invalid index in the y-coordinate vector")

        if ix not in range(0, self.xLen):
            raise ValueError("ix: invalid index in the x-coordinate vector")

        self.gradient.Z[iy, ix] = value

        return

    def getZ(self):
        """
        :return: The matrix representing the field values
        """
        return self.gradient.getZ()

    def getX(self):
        return self.gradient.getX()

    def getY(self):
        return self.gradient.getY()

    def setZ(self, Z):
        """
        :param Z: The matrix representing the field values
        :return: None:
        """
        self.gradient.Z = Z
