"""
.. module:: Torque.py
    :platform: MacOS, Unix, Windows,
    :synopsis: 
.. moduleauthor:: Pavlo Vlastos <pvlastos@ucsc.edu>
"""
import numpy as np

class FromThrust():
    def __init__(self):
        self.r_unit_vec = np.array([[0.0, -1.0, 0.0]])
        self.vec = np.zeros((3,1))
        self.Fb_x = np.zeros((1,3))
        self.Torque = np.zeros((1,3))
        return

    def update(self, F_thrust, radius):
        """
        :param F_thrust: The input thrust force vector
        """
        # Calculate the tangential force for yaw from the rudder
        self.Fb_x[0][0] = F_thrust[0][0]
        self.Torque = np.cross(self.Fb_x, self.r_unit_vec*radius)

        self.vec[0][0] = self.Torque[0][0]
        self.vec[1][0] = self.Torque[0][1]
        self.vec[2][0] = self.Torque[0][2]
        
        return self.vec


class FromDrag():
    def __init__(self):
        self.drag_coeff = 0.0015
        self.vec = np.zeros((3,1))
        return

    def update(self, angular_velocity):
        """
        Update the drag torque based on a rough approximation from angular 
        velocity
        :param angular_velocity: Angular velocity vector 3x1
        :return: The drag force vector
        """

        self.vec[0][0] = -self.drag_coeff*(angular_velocity[0][0]**2.0)
        self.vec[1][0] = -self.drag_coeff*(angular_velocity[1][0]**2.0)
        self.vec[2][0] = -self.drag_coeff*(angular_velocity[2][0]**2.0)

        return self.vec