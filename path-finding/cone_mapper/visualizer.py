import cv2
import numpy as np

class visualize:
    def __init__(self, positions):
        self.positions = positions
        self.height = 512
        self.width = 512
        self.map = np.zeros((self.height, self.width, 3), dtype="uint8")
        self.visualize_cones()

    def visualize_cones(self):
        for pt in self.positions:
            cx = int(pt[1] * 200)
            cy = self.height - int(pt[0] * 100000)

            if pt[2] == 0:
                cv2.circle(self.map, (cx, cy), 3, (0, 70, 255), 3)
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
        cv2.imshow("Map", self.map)
        cv2.waitKey(1)