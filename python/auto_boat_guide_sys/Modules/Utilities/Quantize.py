"""
.. module:: Quantize.py
    :platform: MacOS, Unix, Windows,
    :synopsis: A library for quantizing spatial measurements
.. moduleauthor:: Pavlo Vlastos <pvlastos@ucsc.edu>
"""
import numpy as np
from ..Field import Field as FD

def quantizePosition(point=np.zeros((1,2)), Field=FD.Field):
    """
    :param point: A point with East and North coordinates
    :param Field: A field object
    :return: The quantized position y, x
    """

    # Round x and y to nearest resolution if coordinates isn't an exact
    # multiple. Clip x and/or y if beyond field bounds

    x = point[0][0]
    y = point[0][1]

    # X
    if (Field.x0 <= x) and (x <= Field.xf):
        if np.mod(x, Field.ds) != 0:
            x = np.round(x/Field.ds)*Field.ds

    if x < Field.x0:
        x = Field.x0

    if x > Field.xf:
        x = Field.xf

    # Y
    if (Field.y0 <= y) and (y <= Field.yf):
        if np.mod(y, Field.ds) != 0:
            y = np.round(y/Field.ds)*Field.ds

    if y < Field.y0:
        y = Field.y0

    if y > Field.yf:
        y = Field.yf

    return np.array([[x, y]])


def calculateMatrixIndicesAtPoint(point, Field=FD.Field):
    """
    :param point: a 1x2 vector as an np.array([[]])
    :param Field: a Field object
    :return: The indices of the field x and y coordinate vectors that are 
    closest to the given point 
    """

    q_point = quantizePosition(point, Field)

    iy = int(np.where(Field.y == q_point[0][1])[0])
    ix = int(np.where(Field.x == q_point[0][0])[0])

    return [iy, ix]

