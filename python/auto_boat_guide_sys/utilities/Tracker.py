"""
.. module:: Tracker.py
    :platform: MacOS, Unix, Windows,
    :synopsis:
.. moduleauthor:: Pavlo Vlastos <pvlastos@ucsc.edu>
"""

import pymavlink
from pymavlink import mavutil
import numpy as np
from utilities import Quaternions as Qu
import matplotlib.pyplot as plt
from matplotlib import gridspec


class Tracker():
    def __init__(self, graphInterval=400, x_bound=50, y_bound=50, z_bound=20):
        """
        :param graphInterval: number of main loop cycles before next dynamic
        graph evaluation
        :param x_bound: The x bound for the position subplot
        :param y_bound: The y bound for the position subplot
        :param z_bound: The z bound for the position subplot
        :return: None
        """

        self.graphInterval = graphInterval

        self.yaw = 0.0
        self.pitch = 0.0
        self.roll = 0.0

        #######################################################################
        # Setup Dynamic Graphing
        self.fig = plt.figure(figsize=(16, 8))
        # self.fig = plt.figure()

        gs = gridspec.GridSpec(nrows=4, ncols=6)

        # Attitude
        self.ax0 = self.fig.add_subplot(gs[0:2, 0:2], projection='3d')

        # Position
        self.ax1 = self.fig.add_subplot(gs[0:4, 2:6], projection='3d')

        self.fig.tight_layout(pad=4.0)

        plt.show(block=False)

        #######################################################################
        # Full Attitude Estimate
        self.ax0.set_title('Orientation')
        self.ax0.set_xlim(-1.0, 1.0)
        self.ax0.set_ylim(-1.0, 1.0)
        self.ax0.set_zlim(-1.0, 1.0)
        self.ax0.view_init(30, 30)
        self.ax0.set_xlabel("X")
        self.ax0.set_ylabel("Y")
        self.ax0.set_zlabel("Z")

        # Unit circles
        cSpace = np.arange(0.0, 2.0*np.pi, 0.1)
        ca = np.cos(cSpace)
        cb = np.sin(cSpace)
        cc = np.zeros(len(cSpace))

        self.offset = 1.0

        self.projxz = self.ax0.plot([], [], [], lw=2, color='blue')[0]
        self.projyz = self.ax0.plot([], [], [], lw=2, color='orange')[0]
        self.projzx = self.ax0.plot([], [], [], lw=2, color='red')[0]

        self.ax0.plot(cb, cc-self.offset, ca, lw=2, color='blue')[0]
        self.ax0.plot(cc-self.offset, ca, cb, lw=2, color='orange')[0]
        self.ax0.plot(ca, cb, cc-self.offset, lw=2, color='red')[0]

        # A real vector in R^3 and the origin
        self.attVecX = np.array([[1.0],
                                 [0.0],
                                 [0.0]])
        self.attVecY = np.array([[0.0],
                                 [1.0],
                                 [0.0]])
        self.attVecZ = np.array([[0.0],
                                 [0.0],
                                 [1.0]])
        self.og = np.array([[0.0],
                            [0.0],
                            [0.0]])

        self.lineAttX = self.ax0.plot([], [], [], lw=4, color='purple')[0]
        self.lineAttY = self.ax0.plot([], [], [], lw=2, color='cyan')[0]
        self.lineAttZ = self.ax0.plot([], [], [], lw=2, color='magenta')[0]
        self.dotAcc = self.ax0.plot(self.og[0, 0], self.og[1, 0],
                                    self.og[2, 0], 'o', lw=2,
                                    color='purple')[0]

        self.ax0.grid()

        #######################################################################
        # Position relative to the local tangent plane (LTP)
        self.ax1.set_title('Position')
        self.ax1.set_xlim(-x_bound, x_bound)
        self.ax1.set_ylim(-y_bound, y_bound)
        self.ax1.set_ylim(-z_bound, z_bound)
        self.ax0.view_init(30, 30)
        self.ax0.set_xlabel("East (meters)")
        self.ax0.set_ylabel("North (meters)")
        self.ax0.set_zlabel("Up (meters)")

        # Vector tangent to the current linear trajectory segment vector
        self.norm = self.ax1.plot([], [], lw=2, color='lime',
                                  label='Cross-Track Error')[0]

        # Current linear trajectory segment vector
        self.lin_tra = self.ax1.plot([], [], lw=2, color='tab:purple', 
            label='Linear Trajectory')[0]

        # Previous Waypoint
        self.lin_prev = self.ax1.scatter([], [], lw=2, edgecolors='tab:blue',
                                         color='None', label='Prev WP')
        # Next Waypoint
        self.lin_next = self.ax1.scatter([], [], lw=2, edgecolors='tab:orange',
                                         color='None', label='Next WP')

        # Heading Vector
        self.heading_vec = self.ax1.plot([], [], lw=2, color='tab:purple')[0]

        self.ax1legend = self.ax1.legend()

        #######################################################################
        # Final setup steps
        self.fig.canvas.draw()

        self.background0 = self.fig.canvas.copy_from_bbox(self.ax0.bbox)
        self.background1 = self.fig.canvas.copy_from_bbox(self.ax1.bbox)

        self.i = 0  # Index for graphing udpate
        self.j = 0  # Index for graphing

    def update(self, msg, wp_prev_en=np.zeros((1, 2)),
               wp_next_en=np.zeros((1, 2)), clst_pt_en=np.zeros((1, 2))):
        """
        :param msg: A MAVLink 'ATTITUDE' message
        :param wp_prev_en: The prev waypoint (meters) 1x2 vector: East, North
        :param wp_next_en: The next waypoint (meters) 1x2 vector: East, North
        :param clst_pt_en: The closest point (meters) 1x2 vector: East, North
        :return: None
        """
        if (msg.get_type() != 'ATTITUDE'):
            print("ERROR: Tracker: Wrong message: {}".format(
                msg))
            return

        # Update graphs only at specified intervals to help with speed of
        # graphing
        if np.mod(self.i, self.graphInterval) == 0:

            if msg.get_type() == 'ATTITUDE':
                nav_msg = msg.to_dict()
                self.yaw = nav_msg['yaw']
                self.pitch = nav_msg['pitch']
                self.roll = nav_msg['roll']

            # Attitude
            attQuatX = Qu.rotateVectorWithQuaternion(
                self.attVecX,
                self.yaw,
                self.pitch,
                self.roll)

            attQuatY = Qu.rotateVectorWithQuaternion(
                self.attVecY,
                self.yaw,
                self.pitch,
                self.roll)

            attQuatZ = Qu.rotateVectorWithQuaternion(
                self.attVecZ,
                self.yaw,
                self.pitch,
                self.roll)

            # Projecting the x-vector onto the three different planes
            self.projxz.set_data(np.array([self.og[0, 0], attQuatX[0, 0]]),
                                 np.array([-self.offset, -self.offset]))
            self.projxz.set_3d_properties(np.array([self.og[2, 0],
                                                    attQuatX[2, 0]]))

            self.projyz.set_data(np.array([-self.offset, -self.offset]),
                                 np.array([self.og[1, 0], attQuatX[1, 0]]))
            self.projyz.set_3d_properties(
                np.array([self.og[2, 0], attQuatX[2, 0]]))

            self.projzx.set_data(np.array([self.og[0, 0], attQuatX[0, 0]]),
                                 np.array([self.og[1, 0], attQuatX[1, 0]]))
            self.projzx.set_3d_properties(
                np.array([-self.offset, -self.offset]))

            # Drawing the body reference-frame
            self.lineAttX.set_data(np.array([self.og[0, 0], attQuatX[0, 0]]),
                                   np.array([self.og[1, 0], attQuatX[1, 0]]))
            self.lineAttX.set_3d_properties(np.array([self.og[2, 0],
                                                      attQuatX[2, 0]]))

            self.lineAttY.set_data(np.array([self.og[0, 0], attQuatY[0, 0]]),
                                   np.array([self.og[1, 0], attQuatY[1, 0]]))
            self.lineAttY.set_3d_properties(np.array([self.og[2, 0],
                                                      attQuatY[2, 0]]))

            self.lineAttZ.set_data(np.array([self.og[0, 0], attQuatZ[0, 0]]),
                                   np.array([self.og[1, 0], attQuatZ[1, 0]]))
            self.lineAttZ.set_3d_properties(np.array([self.og[2, 0],
                                                      attQuatZ[2, 0]]))

            ###################################################################
            # Draw

            # ax0
            self.fig.canvas.restore_region(self.background0)
            self.background0 = self.fig.canvas.copy_from_bbox(self.ax0.bbox)
            self.ax0.draw_artist(self.projxz)
            self.ax0.draw_artist(self.projyz)
            self.ax0.draw_artist(self.projzx)
            self.ax0.draw_artist(self.lineAttX)
            self.ax0.draw_artist(self.lineAttY)
            self.ax0.draw_artist(self.lineAttZ)

            self.fig.canvas.blit(self.ax0.bbox)

            self.fig.canvas.flush_events()

            # ax1
            self.fig.canvas.restore_region(self.background1)
            self.ax1.draw_artist(self.norm)
            self.ax1.draw_artist(self.lin_tra)
            self.ax1.draw_artist(self.lin_prev)
            self.ax1.draw_artist(self.lin_next)
            self.ax1.draw_artist(self.heading_vec)

            self.fig.canvas.blit(self.ax1.bbox)

            self.fig.canvas.flush_events()

            self.j += 1

        self.i += 1

        return
