#!/bin/bash
pip install pyyaml
# change directory to tflite examples raspi 
cp -r Landmark_Mapper examples/lite/examples/object_detection/raspberry_pi/
cd examples/lite/examples/object_detection/raspberry_pi/

# Demo
cp Landmark_Mapper/landmark_detect_map.py .
cp Landmark_Mapper/config.yaml .
cp Landmark_Mapper/class_detect.py .
cp Landmark_Mapper/call_detect.py .