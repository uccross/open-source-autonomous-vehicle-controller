"""
.. module:: Force.py
    :platform: MacOS, Unix, Windows,
    :synopsis: 
.. moduleauthor:: Pavlo Vlastos <pvlastos@ucsc.edu>
"""

import numpy as np
import copy
from ..Constants import Constants as CNST

class Thrust():
    def __init__(self):
        self.vec = np.zeros((3,1))
        return

    def update(self, x=0.0, y=0.0, z=0.0):
        """
        :return: Thrust vector 3x1
        """

        self.vec[0][0] = x
        self.vec[1][0] = y
        self.vec[2][0] = z

        return self.vec

class DragLinear():
    def __init__(self):
        self.vec = np.zeros((3,1))
        return

    def update(self, velocity):
        """
        Update the drag force based on 
        :param velocity: Velocity vector 3x1
        :return: The drag force vector
        """

        sign_x = -np.sign(velocity[0][0])
        sign_y = -np.sign(velocity[1][0])
        sign_z = -np.sign(velocity[2][0])

        self.vec[0][0] = sign_x*0.5*CNST.rho*(velocity[0][0]**2)*CNST.Cd*CNST.Area
        self.vec[1][0] = sign_y*0.5*CNST.rho*(velocity[1][0]**2)*CNST.Cd*CNST.Area
        self.vec[2][0] = sign_z*0.5*CNST.rho*(velocity[2][0]**2)*CNST.Cd*CNST.Area

        return self.vec



