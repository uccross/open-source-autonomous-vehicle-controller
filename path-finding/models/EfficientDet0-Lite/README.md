## <div align="center">Cone Detection on Raspberry Pi using EfficientDet0-Lite</div>
Code for detecting cones on Raspberry Pi using the [EfficientDet-Lite0](https://www.tensorflow.org/lite/models/modify/model_maker/object_detection) algorithm. <br><br>
**Note:** <br>
- This model can be optionally accelerated using a Coral Edge TPU Accelerator.
- The same instructions can be used to deploy EfficientDet-Lite0 on an x84 Linux based system

### Prerequisites
- Raspberry Pi OS 11 (bullseye) 64bit
- \>=Python 3.8
- \>=virtualenv 20.12.1

### Installation
Run the following from the root of the cloned repository. Give permission and execute the environment setup and installation scripts 
```
cd <PATH_TO_REPO>/open-source-autonomous-vehicle-controller/path-finding
chmod u+x scripts/install/env.sh scripts/install/install.sh
source scripts/install/env.sh
source scripts/install/install.sh
```
Perform Inference on Raspberry Pi with Coral Edge TPU disabled:<br>
Video Stream from Camera
```
python3 detect.py --model cone_detection.tflite 
```
Using recorded demo video
```
python3 cone_detect.py --model cone_detection.tflite 
```
Perform Inference on Raspberry Pi with Coral Edge TPU enabled:<br>
Video Stream from Camera
```
python3 detect.py --model cone_detection.tflite --enableEdgeTPU
```
Using recorded demo video
```
python3 cone_detect.py --model cone_detection.tflite --enableEdgeTPU
```