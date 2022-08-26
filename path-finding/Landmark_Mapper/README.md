## <div align="center">Landmark_Mapper</div>
Code for detecting and mapping landmarks on Raspberry Pi using the [EfficientDet-Lite0](https://www.tensorflow.org/lite/models/modify/model_maker/object_detection) algorithm. <br><br>
**Note:** <br>
- This model can be optionally accelerated using a Coral Edge TPU Accelerator.
- The same instructions can be used to deploy EfficientDet-Lite0 on an x84 Linux based system

### Prerequisites
- Raspberry Pi OS 11 (bullseye) 64-bit
- \>=Python 3.8
- \>=virtualenv 20.12.1
- numpy 
- pyyaml

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
python landmark_detect_map.py --model cone_detection.tflite --source output.avi
```
Execute landmark mapper on Raspberry Pi with Coral Edge TPU:<br>
```
cd <PATH_TO_REPO>/open-source-autonomous-vehicle-controller/path-finding
source osavc_path_finding/bin/activate
cd examples/lite/examples/object_detection/raspberry_pi/
python landmark_detect_map.py --model cone_detection_edgetpu.tflite --source output.avi --enableEdgeTPU --labels cone_labels.txt
```
### Config
Adapt configuration for your use case by changing data in the yaml file:<br>
#### Runtime Arguments
```
--model: Name of the TFLite object detection model.
            --source: The camera id(int) or video source to be passed to OpenCV.
            --width: The width of the frame captured from the camera.(default=auto)
            --height: The height of the frame captured from the camera.(default=auto)
            --num_threads: The number of CPU threads to run the model.
            --enable_edgetpu: True/False whether the model is a EdgeTPU model.
    Example:
 python landmark_detect_map.py --model cone_detection.tflite --source 0 --num_threads 4 --enableEdgeTPU --labels cone_labels.txt
```

#### Debug
```
DEBUG: Prints bounding box-coordinates and inference times (default=False)
ENABLE_VISUALIZER: Visualizes cone map and detections (default=True)
```

#### camera intrinsics
```
fx: focal length x
fy: focal length y
cx: optical center x
cy: optical center y
```

#### Landmark Params
```
height: height of landmark
width: maximum base width of landmark
```

### Event-Triggered asynchronous call to Detector
Parameters to call the detection are the same as above. An example file called ```call_detect.py``` has been added to demonstrate this functionality. The class is implemented in ```class_detect.py```
```
python call_detect.py

```