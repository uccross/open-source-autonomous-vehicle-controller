#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# Author: Pranay Mathur

import argparse
from curses.ascii import isdigit
import sys
import os
import time
import cv2
from charset_normalizer import detect

from tflite_support.task import core
from tflite_support.task import processor
from tflite_support.task import vision
import utils

from Landmark_Mapper import bbox_to_position
from class_detect import LandmarkDetector

landmarkdetector = LandmarkDetector(model="cone_detection.tflite",
                                    source="output.avi",
                                    width=640,
                                    height=480,
                                    num_threads=4,
                                    enable_edgetpu=False,
                                    labels="cone_labels.txt",
                                    top_k=8,
                                    threshold=0.2)

while True:
    landmarkdetector.tflite_run()
    landmark_positions = landmarkdetector.return_pose()
    print(landmark_positions)
    
"""
# Example code for testing with Coral Edge TPU

landmarkdetector = LandmarkDetector(model="cone_detection_edgetpu.tflite",
                                    source="output.avi",
                                    width=640,
                                    height=480,
                                    num_threads=4,
                                    enable_edgetpu=True,
                                    labels="cone_labels.txt",
                                    top_k=8,
                                    threshold=0.2)

while True:
    landmarkdetector.coral_run()
    landmark_positions = landmarkdetector.return_pose()
    print(landmark_positions)
"""