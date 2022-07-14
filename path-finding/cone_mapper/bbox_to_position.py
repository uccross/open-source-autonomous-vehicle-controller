import sys
import cv2
import numpy as np
from .visualizer import visualize

class bbox:
    def __init__(self, detection_result):
        """
        Enable the debug flag to print bounding-box coordinates
        Enable the visualizer flag to visualize the cone map
        """
        self.DEBUG = False
        self.ENABLE_VISUALIZER = True

        self.detection_result = detection_result
        self.cone_height = 0.7
        self.position2d = []

        # Intrinsic Camera Params
        self.camera_fx = 0.2
        self.camera_fy = 0.2
        self.camera_cx = 0.0
        self.camera_cy = 0.0

    def get_bbox_coordinates(self):
        """
        Get bounding box coordinates 
        """
        for detection in self.detection_result.detections:
            bbox = detection.bounding_box
            class_ = detection.classes[0]
            self.get_position(bbox, class_)
            if (self.DEBUG):
                print("top_l:{},{},top_r:{},{},bottom_l:{},{},bottom_r:{},{}".
                      format(bbox.origin_x, bbox.origin_y,
                             bbox.origin_x + bbox.width, bbox.origin_y,
                             bbox.origin_x, bbox.origin_y + bbox.height,
                             bbox.origin_x + bbox.width,
                             bbox.origin_y + bbox.height))
        if(self.ENABLE_VISUALIZER):
            viz = visualize(self.position2d)

    def get_position(self, bbox, class_):
        """
        depth_cone = cone_height * f_camera
                    ----------------------
                    bounding_box_height
        """
        depth_cone = (self.cone_height * self.camera_fx) / bbox.height
        x = depth_cone
        y = ((bbox.origin_x + bbox.width) / 2 -
             self.camera_cx) * depth_cone / self.camera_fx
        z = 0  # Assume constant 0 height for simplicity
        self.position2d.append((x, y, class_.index))
