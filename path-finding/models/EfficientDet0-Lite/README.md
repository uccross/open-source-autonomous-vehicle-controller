## <div align="center">Cone Detection on Raspberry Pi using EfficientDet0-Lite</div>
Code for detecting cones on Raspberry Pi using the [EfficientDet-Lite0](https://www.tensorflow.org/lite/models/modify/model_maker/object_detection) algorithm. <br>

### Prerequisites
- \>=Python 3.7
- \>=virtualenv 20.12.1

### Installation
#### TFlite Installation on Raspberry Pi with EdgeTPU
Run the following from the root of the cloned repository. Give permission and execute the environment setup and installation scripts 
```
cd <PATH_TO_REPO>/open-source-autonomous-vehicle-controller/path-finding
chmod u+x scripts/env.sh scripts/install.sh
source env.sh
source install.sh
```
