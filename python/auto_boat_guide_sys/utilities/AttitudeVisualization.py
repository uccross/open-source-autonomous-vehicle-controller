"""
.. module:: AttitudeVisualization.py
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


class AttitudeVizualizer():
    def __init__(self, debugFlag=False, graphInterval=400, wt0=0.0, wtf=200.0,
                 wdt=0.1):
        """
        :param debugFlag: Activates a majority of the print statements
        diagnostics mode, Hardware-in-the-loop (HIL) mode, or experimental mode,
        as it runs
        :param graphInterval: number of main loop cycles before next dynamic
        graph evaluation
        :param mavlinkMode: If using MAVLink
        :param wt0: The start time of a dynamic graphing window
        :param wtf: The final time of a dynamic graphing window
        :param wdt: The sample-time of a dynamic graphing window
        :return: None
        """

        self.debugFlag = debugFlag
        self.graphInterval = graphInterval
        self.wt0 = wt0
        self.wtf = wtf
        self.wdt = wdt
        self.wt = np.arange(wt0, wtf, wdt)
        self.wn = len(self.wt)

        self.accel_x = 0.0
        self.accel_y = 0.0
        self.accel_z = 0.0

        self.mag_x = 0.0
        self.mag_y = 0.0
        self.mag_z = 0.0

        self.gyro_x = 0.0
        self.gyro_y = 0.0
        self.gyro_z = 0.0

        self.yaw = 0.0
        self.pitch = 0.0
        self.roll = 0.0

        self.accelX = np.zeros((self.wn))
        self.accelY = np.zeros((self.wn))
        self.accelZ = np.zeros((self.wn))

        self.accelXtoGraph = np.zeros((self.wn))
        self.accelYtoGraph = np.zeros((self.wn))
        self.accelZtoGraph = np.zeros((self.wn))

        self.magX = np.zeros((self.wn))
        self.magY = np.zeros((self.wn))
        self.magZ = np.zeros((self.wn))

        self.magXtoGraph = np.zeros((self.wn))
        self.magYtoGraph = np.zeros((self.wn))
        self.magZtoGraph = np.zeros((self.wn))

        self.gyroX = np.zeros((self.wn))
        self.gyroY = np.zeros((self.wn))
        self.gyroZ = np.zeros((self.wn))

        self.gyroXtoGraph = np.zeros((self.wn))
        self.gyroYtoGraph = np.zeros((self.wn))
        self.gyroZtoGraph = np.zeros((self.wn))

        self.gyroIntXtoGraph = np.zeros((self.wn))
        self.gyroIntYtoGraph = np.zeros((self.wn))
        self.gyroIntZtoGraph = np.zeros((self.wn))

        self.gyroIntXtoGraph = np.zeros((self.wn))
        self.gyroIntYtoGraph = np.zeros((self.wn))
        self.gyroIntZtoGraph = np.zeros((self.wn))

        #######################################################################
        # Setup Dynamic Graphing
        self.fig = plt.figure(figsize=(16, 8))
        # self.fig = plt.figure()

        gs = gridspec.GridSpec(nrows=4, ncols=6)

        # Top Row
        self.ax0 = self.fig.add_subplot(
            gs[0:2, 2:4], projection='3d')  # ACC
        # self.ax1 = self.fig.add_subplot(gs[0:2, 2:4])
        # self.ax2 = self.fig.add_subplot(gs[0:2, 4:6])
        # self.ax3 = self.fig.add_subplot(gs[0:2, 6:8])

        # Bottom Row
        self.ax1 = self.fig.add_subplot(gs[2:4, 0:2])  # ACC
        self.ax2 = self.fig.add_subplot(gs[2:4, 2:4])  # MAG
        self.ax3 = self.fig.add_subplot(gs[2:4, 4:6])  # GYRO

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
        # Accelerometers Raw
        self.ax1.set_title('Accelerometers')
        self.ax1.set_xlim(self.wt0, self.wtf)
        self.ax1.set_ylim(-12.0, 12.0)
        self.ax1.set_xlabel("Time (seconds)")
        self.ax1.set_ylabel("Acceleration (m/s^2)")
        self.ax1.grid()

        self.points4accelX = self.ax1.plot(np.nan, np.nan, color='blue',
                                           label='x')[0]
        self.points4accelY = self.ax1.plot(np.nan, np.nan, color='orange',
                                           label='y')[0]
        self.points4accelZ = self.ax1.plot(np.nan, np.nan, color='red',
                                           label='z')[0]

        self.points4legend = self.ax1.legend(
            handles=[self.points4accelX, self.points4accelY,
                     self.points4accelZ], ncol=3)

        #######################################################################
        # Magnetometers
        self.ax2.set_title('Magnetometers')
        self.ax2.set_xlim(self.wt0, self.wtf)
        self.ax2.set_ylim(-1.0, 1.0)
        self.ax2.set_xlabel("Time (seconds)")
        self.ax2.set_ylabel("Magnetic Field (uT)")
        self.ax2.grid()

        self.points5magX = self.ax2.plot(np.nan, np.nan, color='blue',
                                         label='x')[0]
        self.points5magY = self.ax2.plot(np.nan, np.nan, color='orange',
                                         label='y')[0]
        self.points5magZ = self.ax2.plot(np.nan, np.nan, color='red',
                                         label='z')[0]

        self.points5legend = self.ax2.legend(
            handles=[self.points5magX, self.points5magY,
                     self.points5magZ], ncol=3)

        #######################################################################
        # Gyroscopes
        self.ax3.set_title('Gyroscopes')
        self.ax3.set_xlim(self.wt0, self.wtf)
        self.ax3.set_ylim(-30.0, 30.0)
        self.ax3.set_xlabel("Time (seconds)")
        self.ax3.set_ylabel("Degrees/second")
        self.ax3.grid()

        self.points6gyroX = self.ax3.plot(np.nan, np.nan, color='orange',
                                          label='x')[0]
        self.points6gyroY = self.ax3.plot(np.nan, np.nan, color='blue',
                                          label='y')[0]
        self.points6gyroZ = self.ax3.plot(np.nan, np.nan, color='red',
                                          label='z')[0]

        self.points6legend = self.ax3.legend(
            handles=[self.points6gyroX, self.points6gyroY,
                     self.points6gyroZ], ncol=3)

        # Final Setup Steps
        self.fig.canvas.draw()

        self.background0 = self.fig.canvas.copy_from_bbox(self.ax0.bbox)
        #
        #
        #
        self.background1 = self.fig.canvas.copy_from_bbox(self.ax1.bbox)
        self.background2 = self.fig.canvas.copy_from_bbox(self.ax2.bbox)
        self.background3 = self.fig.canvas.copy_from_bbox(self.ax3.bbox)

        self.i = 0  # Index for graphing udpate
        self.j = 0  # Index for graphing

        return

    def update(self, msg):
        """
        :param msg: A MAVLink 'ATTITUDE' message
        :return: None
        """
        if ((msg.get_type() != 'ATTITUDE') and
                (msg.get_type() != 'HIGHRES_IMU')):
            print("ERROR: Attitude visualization: Wrong message: {}".format(
                msg))
            return

        # Update graphs only at specified intervals to help with speed of
        # graphing
        if np.mod(self.i, self.graphInterval) == 0:

            if msg.get_type() == 'HIGHRES_IMU':
                nav_msg = msg.to_dict()
                self.accel_x = nav_msg['xacc']
                self.accel_y = nav_msg['yacc']
                self.accel_z = nav_msg['zacc']

                self.mag_x = nav_msg['xmag']
                self.mag_y = nav_msg['ymag']
                self.mag_z = nav_msg['zmag']

                self.gyro_x = nav_msg['xgyro']
                self.gyro_y = nav_msg['ygyro']
                self.gyro_z = nav_msg['zgyro']

            if msg.get_type() == 'ATTITUDE':
                nav_msg = msg.to_dict()
                self.yaw = nav_msg['yaw']
                self.pitch = nav_msg['pitch']
                self.roll = nav_msg['roll']

            # For a horizontally moving graph without expensive appending
            wi = np.mod(self.j, self.wn)

            self.accelX[wi] = self.accel_x
            self.accelY[wi] = self.accel_y
            self.accelZ[wi] = self.accel_z

            self.magX[wi] = self.mag_x
            self.magY[wi] = self.mag_y
            self.magZ[wi] = self.mag_z

            self.gyroX[wi] = self.gyro_x
            self.gyroY[wi] = self.gyro_y
            self.gyroZ[wi] = self.gyro_z

            ###################################################################
            # Accelerometers 3D Axes
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
            # Accelerometers Raw
            self.accelXtoGraph[0:(self.wn - wi-1)] = \
                self.accelX[(wi+1):self.wn]
            self.accelXtoGraph[(self.wn - wi):self.wn] = \
                self.accelX[0:wi]

            self.accelYtoGraph[0:(self.wn - wi-1)] = \
                self.accelY[(wi+1):self.wn]
            self.accelYtoGraph[(self.wn - wi):self.wn] = self.accelY[0:wi]

            self.accelZtoGraph[0:(self.wn - wi-1)] = \
                self.accelZ[(wi+1):self.wn]
            self.accelZtoGraph[(self.wn - wi):self.wn] = self.accelZ[0:wi]

            self.ax1.set_xlim(self.wt0, self.wtf)
            self.ax1.set_ylim(-12.0, 12.0)

            self.points4accelX.set_data(self.wt, self.accelXtoGraph)
            self.points4accelY.set_data(self.wt, self.accelYtoGraph)
            self.points4accelZ.set_data(self.wt, self.accelZtoGraph)

            # TODO: Make this vvvvvvvvvvvvvvvvvvvvvv change
            # self.ax1.set_xlim(self.wt0, self.wtf)
            #                 ^^^^^^^^^^^^^^^^^^^^^^

            ###################################################################
            # Magnetometers Raw
            self.magXtoGraph[0:(self.wn - wi-1)] = \
                self.magX[(wi+1):self.wn]
            self.magXtoGraph[(self.wn - wi):self.wn] = \
                self.magX[0:wi]

            self.magYtoGraph[0:(self.wn - wi-1)] = \
                self.magY[(wi+1):self.wn]
            self.magYtoGraph[(self.wn - wi):self.wn] = self.magY[0:wi]

            self.magZtoGraph[0:(self.wn - wi-1)] = \
                self.magZ[(wi+1):self.wn]
            self.magZtoGraph[(self.wn - wi):self.wn] = self.magZ[0:wi]

            self.ax2.set_xlim(self.wt0, self.wtf)
            self.ax2.set_ylim(-1.0, 1.0)

            self.points5magX.set_data(self.wt, self.magXtoGraph)
            self.points5magY.set_data(self.wt, self.magYtoGraph)
            self.points5magZ.set_data(self.wt, self.magZtoGraph)

            # TODO: Make this vvvvvvvvvvvvvvvvvvvvvv change
            # self.ax2.set_xlim(self.wt0, self.wtf)
            #                 ^^^^^^^^^^^^^^^^^^^^^^

            ###################################################################
            # Gyroscopes Raw
            self.gyroXtoGraph[0:(self.wn - wi-1)] = \
                self.gyroX[(wi+1):self.wn]
            self.gyroXtoGraph[(self.wn - wi):self.wn] = \
                self.gyroX[0:wi]

            self.gyroYtoGraph[0:(self.wn - wi-1)] = \
                self.gyroY[(wi+1):self.wn]
            self.gyroYtoGraph[(self.wn - wi):self.wn] = self.gyroY[0:wi]

            self.gyroZtoGraph[0:(self.wn - wi-1)] = \
                self.gyroZ[(wi+1):self.wn]
            self.gyroZtoGraph[(self.wn - wi):self.wn] = self.gyroZ[0:wi]

            self.ax3.set_xlim(self.wt0, self.wtf)
            self.ax3.set_ylim(-15.0, 15.0)

            self.points6gyroX.set_data(self.wt, self.gyroXtoGraph)
            self.points6gyroY.set_data(self.wt, self.gyroYtoGraph)
            self.points6gyroZ.set_data(self.wt, self.gyroZtoGraph)

            # TODO: Make this vvvvvvvvvvvvvvvvvvvvvv change
            # self.ax3.set_xlim(self.wt0, self.wtf)
            #                 ^^^^^^^^^^^^^^^^^^^^^^

            ###################################################################
            # Gyroscopes Integrated

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
            self.ax1.draw_artist(self.points4accelX)
            self.ax1.draw_artist(self.points4accelY)
            self.ax1.draw_artist(self.points4accelZ)

            self.fig.canvas.blit(self.ax1.bbox)

            self.fig.canvas.flush_events()

            # ax2
            self.fig.canvas.restore_region(self.background2)
            self.ax2.draw_artist(self.points5magX)
            self.ax2.draw_artist(self.points5magY)
            self.ax2.draw_artist(self.points5magZ)

            self.fig.canvas.blit(self.ax2.bbox)

            self.fig.canvas.flush_events()

            # ax3
            self.fig.canvas.restore_region(self.background3)
            self.ax3.draw_artist(self.points6gyroX)
            self.ax3.draw_artist(self.points6gyroY)
            self.ax3.draw_artist(self.points6gyroZ)

            self.fig.canvas.blit(self.ax3.bbox)

            self.fig.canvas.flush_events()

            self.j += 1

        self.i += 1

        return
