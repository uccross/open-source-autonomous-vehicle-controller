## <div align="center">Cone Mapper</div>
Code for detecting and mapping cones on Raspberry Pi using the [EfficientDet-Lite0](https://www.tensorflow.org/lite/models/modify/model_maker/object_detection) algorithm. <br><br>
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
chmod u+x scripts/install/env.sh scripts/install/install.sh scripts/install/install_cone_mapper.sh
source scripts/install/env.sh
source scripts/install/install.sh
source scripts/install/install_cone_mapper.sh
```
Execute cone mapper:<br>
```
python3 cone_detect_map.py --model cone_detection.tflite 
```