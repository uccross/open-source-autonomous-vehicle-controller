## <div align="center">Landmark_Mapper</div>
Code for detecting and mapping landmarks on Raspberry Pi using the [EfficientDet-Lite0](https://www.tensorflow.org/lite/models/modify/model_maker/object_detection) algorithm. <br><br>
**Note:** <br>
- This model can be optionally accelerated using a Coral Edge TPU Accelerator.
- The same instructions can be used to deploy EfficientDet-Lite0 on an x84 Linux based system

### Prerequisites
- Raspberry Pi OS 11 (bullseye) 64-bit
- \>=Python 3.8
- \>=virtualenv 20.12.1

### Installation
Run the following from the root of the cloned repository. Give permission and execute the environment setup and installation scripts 
```
cd <PATH_TO_REPO>/open-source-autonomous-vehicle-controller/path-finding
chmod u+x scripts/install/env.sh scripts/install/install.sh scripts/install/install_landmark_mapper.sh
source scripts/install/env.sh
source scripts/install/install.sh
source scripts/install/install_landmark_mapper.sh
```

### Inference and Mapping
Execute landmark mapper:<br>
```
cd <PATH_TO_REPO>/open-source-autonomous-vehicle-controller/path-finding
source osavc_path_finding/bin/activate
cd examples/lite/examples/object_detection/raspberry_pi/
python3 landmark_detect_map.py --model cone_detection.tflite 
```