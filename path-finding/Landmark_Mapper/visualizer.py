#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# Author: Pranay Mathur

import cv2
import numpy as np

class visualize:
    def __init__(self, positions, map_name):
        self.positions = positions
        self.map_name = map_name
        self.height = 512
        self.width = 512
        self.map = np.zeros((self.height, self.width, 3), dtype="uint8")

        # max value for depth
        max_x = 5.3
        # max value for horizontal FOV
        max_y = 0.007

        self.scale_cx = self.height / max_x
        self.scale_cy = self.width / max_y
        self.visualize_cones()

    def visualize_cones(self):
        """
        Visualizes Landmark position in the local frame
        """
        # Modify classes and colors as required
        for pt in self.positions:
            cx = int(pt[1] * self.scale_cx)
            cy = self.height - int(pt[0] * self.scale_cy)
            if pt[2] == 0:
                cv2.circle(self.map, (cx, cy), 3, (0, 60, 255), 3)
            elif pt[2] == 1:
                cv2.circle(self.map, (cx, cy), 3, (0, 255, 0), 3)
            elif pt[2] == 2:
                cv2.circle(self.map, (cx, cy), 3, (255, 255, 255), 3)
            elif pt[2] == 3:
                cv2.circle(self.map, (cx, cy), 3, (255, 0, 0), 3)
            elif pt[2] == 4:
                cv2.circle(self.map, (cx, cy), 3, (255, 255, 255), 3)
            elif pt[2] == 5:
                cv2.circle(self.map, (cx, cy), 3, (0, 255, 255), 3)
        cv2.imshow(self.map_name, self.map)
        cv2.waitKey(1)