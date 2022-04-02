"""
.. module:: Linear.py
	:platform: MacOS, Unix, Windows,
	:synopsis: Linear trajectory generation, connecting waypoints for Autonomous
    an Surface Vehicle (ASV)
.. moduleauthor:: Pavlo Vlastos <pvlastos@ucsc.edu>
"""
import numpy as np
from ..Constants import Constants as CONST


class Linear():
    def __init__(self, vehiPT=CONST.defaultWP, prevWP=CONST.defaultWP,
                 nextWP=CONST.defaultWP, closPT=CONST.defaultWP,
                 pathAngle=0.0):
        """
        :param vehiPT: Vehicle location within Local Tangent Plane (LTP)
        :param prevWP: Previous Waypoint within Local Tangent Plane (LTP)
        :param nextWP: Next Waypoint within Local Tangent Plane (LTP)
        :return: none
        """

        self.vehiPT = vehiPT
        self.prevWP = prevWP
        self.nextWP = nextWP
        self.closPT = closPT
        self.pathAngle = pathAngle
        self.cte = 0.0

        return

    def udpate(self, newPosition):
        """
        def udpate(self, newPosition):
        :param newPositon:
        :return: cte, The cross-track error
        """
        self.vehiPT = newPosition
        self.closPT = self.project()
        self.pathAngle = self.calculatePathAngle()
        self.cte = self.calculateCte()
        return self.cte

    def calculateCte(self):
        """
        def calculateCte(self):
        :return: cte, The cross-track error [meters]
        """

        # Move segment to origin, basically creating Serret-Frenet (SF) frame
        vehiclePositionSF = self.vehiPT - self.prevWP
        closestPTonLine = self.closPT - self.prevWP

        # Rotate all points to be in the SF frame
        c = np.cos(self.pathAngle)
        s = np.sin(self.pathAngle)
        R = np.array([[c, -s],
                      [s, c]])

        # Finish making the SF frame using rotation
        vehiclePositionSF = np.matmul(R, np.transpose(vehiclePositionSF))
        closestPTonLine = np.matmul(R, np.transpose(closestPTonLine))
        cte = (closestPTonLine[0] - vehiclePositionSF[0])

        return cte[0]

    def project(self):
        """
        def project(self):
        :return: closPT, The closest point on the line connecting the previous 
        and next waypoints, [1x2] in meters
        """

        a = self.prevWP
        b = self.nextWP
        p = self.vehiPT

        # Move the vector to the origin and project onto path segment
        bv = b - a
        av = p - a

        # Check for zero-case
        if bv[0, 0] == 0.0 and bv[0, 1] == 0.0:
            return self.closPT

        bvhat = bv/np.linalg.norm(bv)
        a1s = np.dot(av, np.transpose(bvhat))
        a1v = a1s*bvhat
        #a2v = av - a1v

        # Calculate closest point and move back from origin
        return a1v + a

    def project_look_point(self, look_point):
        """
        :param look_point: A 1x2 vector
        """
        a = self.prevWP
        b = self.nextWP

        # Move the point to the origin and project onto path segment
        bv = b - a

        lv = look_point - a

        # Check for zero-case
        if bv[0, 0] == 0.0 and bv[0, 1] == 0.0:
            return self.closPT

        bvhat = bv/np.linalg.norm(bv)
        l1s = np.dot(lv, np.transpose(bvhat))
        l1v = l1s*bvhat

        return l1v + a

    def create_look_vector(self, heading_angle, look_distance):
        """
        :param heading_angle: 
        :param look_distance: A scalar
        """

        look_vector = look_distance*np.array(
            [[np.sin(heading_angle), np.cos(heading_angle)]])

        return look_vector

    def calculatePathAngle(self):
        """
        def calculatePathAngle(self):
        :return: pathAngle, The angle of the path in radians [-pi, pi]
        """

        # Move vector to origin
        p1v = self.nextWP - self.prevWP

        self.pathAngle = np.arctan2(CONST.north_vec[0, 1],
                                    CONST.north_vec[0, 0]) \
            - np.arctan2(p1v[0, 1], p1v[0, 0])

        # Make sure the heading angle is wrapped to [-pi, pi]
        self.pathAngle = (self.pathAngle + np.pi) % (2.0 * np.pi) - np.pi

        return self.pathAngle

    def is_closest_point_beyond_next(self, threshold=2.5):
        is_beyond = False
        c0 = self.closPT-self.prevWP
        t0 = self.nextWP-self.prevWP
        
        if np.linalg.norm(c0)+(threshold) >= np.linalg.norm(t0):
            is_beyond = True

        return is_beyond

    def is_closest_point_near_next_wp(self, threshold):
        return (np.linalg.norm(self.closPT - self.nextWP) <= threshold)

    def getClosestPoint(self):
        """
        :return: closPT, The closest point on the line connecting the previous 
        and next waypoints, [1x2] in meters
        """
        return self.closPT

    def getCte(self):
        """
        :return: cte, The cross-track error [meters]
        """
        return self.cte

    def getPathAngle(self):
        """
        :return: cte, The cross-track error [meters]
        """
        return self.pathAngle

    def getPreviousWaypoint(self):
        """
        Get the previous waypoint
        :return: The previous waypoint
        """
        return self.prevWP

    def getNextWaypoint(self):
        """
        Get the next waypoint
        :return: The next waypoint
        """
        return self.nextWP

    def setPreviousWaypoint(self, newPrevWP):
        """
        Set the previous waypoint
        :param newPrevWP: The new previous wayopint
        """
        self.prevWP = newPrevWP

    def setNextWaypoint(self, newNextWP):
        """
        Set the next waypoint
        :param newPrevWP: The new next wayopint
        """
        self.nextWP = newNextWP
