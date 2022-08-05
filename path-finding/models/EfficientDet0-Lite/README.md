## <div align="center">Cone Detection on Raspberry Pi using EfficientDet0-Lite</div>
Code for detecting cones on Raspberry Pi using the [EfficientDet-Lite0](https://www.tensorflow.org/lite/models/modify/model_maker/object_detection) algorithm. <br><br>
**Note:** <br>
- This model can be optionally accelerated using a Coral Edge TPU Accelerator.
- The same instructions can be used to deploy EfficientDet-Lite0 on an x84 Linux based system

### Prerequisites
- Raspberry Pi OS 11 (bullseye) 64bit
- \>=Python 3.8
- \>=virtualenv 20.12.1
- numpy
- pyyaml

### Installation
Run the following from the root of the cloned repository. Give permission and execute the environment setup and installation scripts.
```
cd <PATH_TO_REPO>/open-source-autonomous-vehicle-controller/path-finding
chmod u+x scripts/install/env.sh scripts/install/install.sh
source scripts/install/env.sh
source scripts/install/install.sh
```
#### (Optional) Install Cone Mapper
```
cd <PATH_TO_REPO>/open-source-autonomous-vehicle-controller/path-finding
source scripts/install/install_landmark_mapper.sh
```

### Inference
#### Perform Inference on Raspberry Pi with Coral Edge TPU enabled:<br>
```
cd <PATH_TO_REPO>/open-source-autonomous-vehicle-controller/path-finding
source osavc_path_finding/bin/activate
cd examples/lite/examples/object_detection/raspberry_pi/pycoral/examples
```
Video Stream from Camera
```
python3 detect_camera.py --model cone_detection_edgetpu.tflite --labels cone_labels.txt
```
Using recorded demo video
```
python3 detect_image.py --model cone_detection_edgetpu.tflite --labels cone_labels.txt
```

#### Perform Inference on Raspberry Pi without the Coral Edge TPU:<br>
```
cd <PATH_TO_REPO>/open-source-autonomous-vehicle-controller/path-finding
source osavc_path_finding/bin/activate
cd examples/lite/examples/object_detection/raspberry_pi/
```
Video Stream from Camera
```
python3 detect.py --model cone_detection.tflite 
```
Using recorded demo video
```
python3 cone_detect.py --model cone_detection.tflite 
```
#### For Inference and Mapping
Run an example using the following commands. Detailed Instructions are available [here](../../Landmark_Mapper/README.md)
```
cd <PATH_TO_REPO>/open-source-autonomous-vehicle-controller/path-finding
source osavc_path_finding/bin/activate
cd examples/lite/examples/object_detection/raspberry_pi/
python landmark_detect_map.py --model cone_detection_edgetpu.tflite --source output.avi --enableEdgeTPU --labels cone_labels.txt
```