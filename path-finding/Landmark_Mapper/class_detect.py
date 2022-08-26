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

from pycoral.adapters.common import input_size
from pycoral.adapters.detect import get_objects
from pycoral.utils.dataset import read_label_file
from pycoral.utils.edgetpu import make_interpreter
from pycoral.utils.edgetpu import run_inference


class LandmarkDetector:
    def __init__(self, model, source, width, height, num_threads,
                 enable_edgetpu, labels, top_k, threshold):
        try:
            self.cap = cv2.VideoCapture(int(source))
        except:
            self.cap = cv2.VideoCapture(source)

        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)

        self.positions = None

        if not (bool(enable_edgetpu)):
            self.tflite_run(model, source, int(width), int(height),
                            int(num_threads), bool(enable_edgetpu))
        else:
            self.coral_run(model, labels, threshold, top_k, enable_edgetpu)

    def coral_run(self, model, labels, threshold, top_k, enable_edgetpu):
        print('Loading {} with {} labels.'.format(model, labels))
        interpreter = make_interpreter(model)
        interpreter.allocate_tensors()
        labels = read_label_file(labels)
        inference_size = input_size(interpreter)

        ret, frame = self.cap.read()
        cv2_im = frame

        cv2_im_rgb = cv2.cvtColor(cv2_im, cv2.COLOR_BGR2RGB)
        cv2_im_rgb = cv2.resize(cv2_im_rgb, inference_size)
        start = time.perf_counter()
        run_inference(interpreter, cv2_im_rgb.tobytes())
        inference_time = time.perf_counter() - start
        cv2_im = cv2.putText(cv2_im, "FPS:%.2f" % (1 / inference_time),
                             (20, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                             (255, 0, 0), 2)

        objs = get_objects(interpreter, threshold)[:top_k]
        cone = bbox_to_position.bbox(objs, enable_edgetpu)
        self.positions = cone.get_pose()
        cv2_im = self.append_objs_to_img(cv2_im, inference_size, objs, labels)

        #cv2.imshow('frame', cv2_im)
        #cv2.waitKey(0)

    def append_objs_to_img(self, cv2_im, inference_size, objs, labels):
        height, width, channels = cv2_im.shape
        scale_x, scale_y = width / inference_size[0], height / inference_size[1]
        for obj in objs:
            bbox = obj.bbox.scale(scale_x, scale_y)
            x0, y0 = int(bbox.xmin), int(bbox.ymin)
            x1, y1 = int(bbox.xmax), int(bbox.ymax)

            percent = int(100 * obj.score)
            label = '{}% {}'.format(percent, labels.get(obj.id, obj.id))

            cv2_im = cv2.rectangle(cv2_im, (x0, y0), (x1, y1), (0, 0, 255), 2)
            cv2_im = cv2.putText(cv2_im, label, (x0, y0 + 30),
                                 cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 0, 0), 2)
        return cv2_im

    def tflite_run(self, model: str, source: str, width: int, height: int,
                   num_threads: int, enable_edgetpu: bool) -> None:
        """
            Continuously run inference on images acquired from the camera.
            Args:
            model: Name of the TFLite object detection model.
            source: The camera id or video source to be passed to OpenCV.
            width: The width of the frame captured from the camera.
            height: The height of the frame captured from the camera.
            num_threads: The number of CPU threads to run the model.
            enable_edgetpu: True/False whether the model is a EdgeTPU model.
        """

        # Variables to calculate FPS
        counter = 0
        fps = 0
        start_time = time.time()

        # Visualization parameters
        row_size = 20  # pixels
        left_margin = 24  # pixels
        text_color = (0, 0, 255)  # red
        font_size = 1
        font_thickness = 1
        fps_avg_frame_count = 10

        # Initialize the object detection model
        base_options = core.BaseOptions(file_name=model,
                                        use_coral=enable_edgetpu,
                                        num_threads=num_threads)
        detection_options = processor.DetectionOptions(max_results=10,
                                                       score_threshold=0.2)
        options = vision.ObjectDetectorOptions(
            base_options=base_options, detection_options=detection_options)
        detector = vision.ObjectDetector.create_from_options(options)

        # capture image from the camera and run inference
        success, image = self.cap.read()
        if not success:
            sys.exit(
                'ERROR: Unable to read from webcam. Please verify your webcam settings.'
            )

        image = cv2.resize(image, (224, 224), cv2.INTER_AREA)

        # Convert the image from BGR to RGB as required by the TFLite model.
        rgb_image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

        # Create a TensorImage object from the RGB image.
        input_tensor = vision.TensorImage.create_from_array(rgb_image)

        # Run object detection estimation using the model.
        detection_result = detector.detect(input_tensor)

        # Draw keypoints and edges on input image
        image = utils.visualize(image, detection_result)

        # call cone_mapper
        cone = bbox_to_position.bbox(detection_result, enable_edgetpu)
        self.positions = cone.get_pose()

        #cv2.imshow('object_detector', image)
        #cv2.waitKey(0)

    def return_pose(self):
        return self.positions