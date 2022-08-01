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

# Download Weights for edge tpu
gdown 1E7vIXdZ15Wyz0uZrLxLXmwp1l_k_aee_

# Download the cone_labels.txt
gdown 1ZRk9k9IMz4lQCi7LrGTR0UYv8qjJ50K_

# Install PyCoral
pip install --extra-index-url https://google-coral.github.io/py-repo/ pycoral~=2.0

# Demo
cp detect.py cone_detect.py
sed -i 's/(camera_id)/("output.avi")/g' cone_detect.py

# Install pycoral
git clone https://github.com/google-coral/pycoral
cd pycoral/examples
sudo apt-get install libedgetpu1-std

# Download Weights for edge tpu
gdown 1E7vIXdZ15Wyz0uZrLxLXmwp1l_k_aee_

# Download the test video
gdown 1O8sTOCbTI0bmJTaZhbOr40dgPJSWVNYz

# Download the cone_labels.txt
gdown 1ZRk9k9IMz4lQCi7LrGTR0UYv8qjJ50K_

# Copy the detection script
cp ../../../../../../../scripts/inference/detect_image.py .

cp detect_image.py detect_camera.py
sed -i 's/("output.avi")/(camera_id)/g' detect_camera.py