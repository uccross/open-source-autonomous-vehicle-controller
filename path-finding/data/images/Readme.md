# Dataset
Each of the folders above have two types of files<br>
- images(.jpg)
- Annotations(.xml)

The dataset has been pre-processed and augmented using the following steps

**PREPROCESSING**
- Resize: Stretch to 224x224<br>

**AUGMENTATIONS**
- Outputs per training example: 3
- Crop: 0% Minimum Zoom, 34% Maximum Zoom
- Rotation: Between -8° and +8°
- Shear: ±3° Horizontal, ±5° Vertical
- Saturation: Between -25% and +25%
- Bounding Box: Crop: 0% Minimum Zoom, 20% Maximum Zoom
<br>
Annotations have been generated using [Roboflow](https://roboflow.com/). TensorFlow uses ```tfrecords```(a simple format for storing a sequence of binary records)
to train an ML model. Using the images and annotaions ```tfrecords``` will be generated.

Legacy Annotations:<br>
Annotations have been generated using [LabelImg](https://github.com/tzutalin/labelImg) tool. <br>


