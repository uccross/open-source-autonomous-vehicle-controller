#!/bin/bash
# Install dependencies
pip install gdown
sudo apt-get install libportaudio2

# Install TensorFlow Lite for Python
/usr/bin/python3 -m pip install tflite-runtime

# clone the Tensorflow examples repository and install tflite
git clone https://github.com/tensorflow/examples --depth 1
cd examples/lite/examples/object_detection/raspberry_pi
. ./setup.sh

# Download the test video
gdown 1O8sTOCbTI0bmJTaZhbOr40dgPJSWVNYz

# Download the quantized model
gdown 1-3ZxeGXyJhshmpE7Vrc8jxo_zkE0GKCB

# Install PyCoral
/usr/bin/python3 -m pip install --extra-index-url https://google-coral.github.io/py-repo/ pycoral~=2.0

# Demo
cp detect.py cone_detect.py
sed -i 's/(camera_id)/("output.avi")/g' cone_detect.py
