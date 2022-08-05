#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# Author: Pranay Mathur

import sys
import cv2
import yaml
import numpy as np
from yaml.loader import SafeLoader
import numpy as np
from .visualizer import visualize


class bbox:
    def __init__(self, detection_result, enable_edgeTPU):
        """
        Initialize and read yaml file
        Enable the debug flag to print bounding-box coordinates
        Enable the visualizer flag to visualize the cone map
        """

        self.yaml_path = "config.yaml"
        self.read_yaml()

        self.DEBUG = self.data['DEBUG']
        self.ENABLE_VISUALIZER = self.data['ENABLE_VISUALIZER']

        self.detection_result = detection_result
        self.landmark_height = self.data['height']
        self.landmark_width = self.data['width']
        self.position2d = []

        # Intrinsic Camera Params
        self.camera_fx = self.data['fx']
        self.camera_fy = self.data['fy']
        self.camera_cx = self.data['cx']
        self.camera_cy = self.data['cy']
        self.get_bbox_coordinates(enable_edgeTPU)

    def read_yaml(self):
        with open(self.yaml_path) as f:
            self.data = yaml.load(f, Loader=SafeLoader)

    def get_bbox_coordinates(self, edge_processing):
        """
        Get bounding box coordinates 
        """
        if (edge_processing):
            for obj in self.detection_result:
                class_ = obj.id
                bbox = obj.bbox
                self.get_position(bbox, class_, edge_processing)
                if (self.DEBUG):
                    print(
                        "top_l:{},{},top_r:{},{},bottom_l:{},{},bottom_r:{},{}"
                        .format(bbox.xmin, bbox.ymin, bbox.xmax, bbox.ymin,
                                bbox.xmin, bbox.ymax, bbox.xmax, bbox.ymax))
        else:
            for detection in self.detection_result.detections:
                bbox = detection.bounding_box
                class_ = detection.classes[0]
                self.get_position(bbox, class_, edge_processing)
                if (self.DEBUG):
                    print(
                        "top_l:{},{},top_r:{},{},bottom_l:{},{},bottom_r:{},{}"
                        .format(bbox.origin_x, bbox.origin_y,
                                bbox.origin_x + bbox.width, bbox.origin_y,
                                bbox.origin_x, bbox.origin_y + bbox.height,
                                bbox.origin_x + bbox.width,
                                bbox.origin_y + bbox.height))

            if (self.ENABLE_VISUALIZER):
                viz = visualize(self.position2d)

    def get_variance(self, dh, dw):
        mean = (dh + dw) / 2
        self.variance = (dh - mean)**2 + (dw - mean)**2
        if (self.DEBUG):
            print(self.variance)

    def get_position(self, bbox, class_, enable_edgetpu):
        """
        depth_landmark = landmark_height * f_camera
                    ----------------------
                      bounding_box_height
        """
        if (enable_edgetpu):
            depth_landmark = (self.landmark_height *
                              self.camera_fx) / (bbox.ymax - bbox.ymin)
            x = depth_landmark
            y = ((bbox.xmin + bbox.xmax) / 2 -
                 self.camera_cx) * depth_landmark / self.camera_fx
            z = 0  # Assume constant 0 height for simplicity

            depth_landmark_w = (self.landmark_width *
                                self.camera_fy) / (bbox.xmax - bbox.xmin)
            self.get_variance(depth_landmark, depth_landmark_w)
            self.position2d.append((x, y, int(class_)))
        else:
            depth_landmark = (self.landmark_height *
                              self.camera_fx) / bbox.height
            x = depth_landmark
            y = ((bbox.origin_x + (bbox.width / 2)) -
                 self.camera_cx) * depth_landmark / self.camera_fx
            z = 0  # Assume constant 0 height for simplicity
            depth_landmark_w = (self.landmark_width *
                                self.camera_fy) / bbox.width
            self.get_variance(depth_landmark, depth_landmark_w)
            self.position2d.append((x, y, class_.index))