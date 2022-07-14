## <div align="center">Cone Detection on Raspberry Pi using Yolov5</div>
Code for detecting cones on Raspberry Pi using the [Yolov5](https://github.com/ultralytics/yolov5) algorithm. <br>
Getting the virtual environment's pip package
```bash
sudo pip install virtualenv
```
Now, proceed further by creating a folder and installing yolov5 and other requirements in virual environment.<br>
```bash
mkdir test
cd test
git clone https://github.com/zldrobit/yolov5.git
cd yolov5
git checkout tf-android

```
Now clone the ```open-source-autonomous-vehicle-controller``` repository. And copy the ```path-finding/yolo-v5``` folder from the ```dev_path-finding``` branch into the ```test``` folder.
```bash
$ git clone https://github.com/uccross/open-source-autonomous-vehicle-controller
$ git checkout dev_path-finding
```

Creating virtual environment and installing the dependencies. First move into the test/yolo-v5 folder & then run the follwing commands <br>
```bash
python -m venv tflite-env
source activate tflite-env/bin/activate
sudo apt-get -y install libjpeg-dev libtiff5-dev libjasper-dev libpng12-dev
sudo apt-get -y install libavcodec-dev libavformat-dev libswscale-dev libv4l-dev
sudo apt-get -y install libxvidcore-dev libx264-dev
sudo apt-get -y install qt4-dev-tools libatlas-base-dev

pip install -r requirements.txt
```
Let's install tflite interpreter and pytorch on the device<br>
```bash
$ pip install https://dl.google.com/coral/python/tflite_runtime-2.1.0.post1-cp37-cp37m-linux_armv7l.whl
```
First install dependencies of pytorch and then install it from using wheel files
```bash
sudo apt install libopenblas-dev libblas-dev m4 cmake cython python3-dev python3-yaml python3-setuptools
$ pip install https://github.com/Kashu7100/pytorch-armv7l/raw/main/torch-1.7.0a0-cp37-cp37m-linux_armv7l.whl
$ pip install https://github.com/Kashu7100/pytorch-armv7l/raw/main/torchvision-0.8.0a0%2B45f960c-cp37-cp37m-linux_armv7l.whl
```
Test if pytorch is installed on the device by running following command on terminal
```bash
$ python
>>> import torch
>>> import torchvision
```
Now, yolov5 model is installed in the device. First let's test by running the pytorch model on the device from the ```test/yolo-v5```.<br>
I have trained two different models, one with the lower(256x320) and one with higher(320x416) resolution.<br>
Running the inference on a image.

```bash
python detect.py --weights [weight-path] --conf 0.25 --img 320 --source [path-to-yolo-v5]/data/images/test.jpg
```

Running the inference on a video.
```bash
python detect.py --weights [weight-path] --conf 0.25 --img 320 --source [path-to-yolo-v5]/data/videos/test.mp4
```
Now, for changing the source to webcam, just give 0 or 1 for the ```--source``` in the command.
```bash
python detect.py --weights [weight-path] --conf 0.25 --img 320 --source 0
```

The output will get saved in exp folder in ```./yolo-v5/runs/detect/```.

Let's move on to test the tflite model. <br>
Copy the tflite folder from the yolo-v5 folder into the yolov5 folder.
Then we have to do some configuration. Copy the ```./yolo-v5/data/coco128.yaml```'s content into ```./yolov5/data/coco.yaml``` and ```./yolov5/data/coco128.yaml```. <br>
Now, change the parameter ```nc``` to 1 in the file ```yolov5/models/yolo5s.yaml```.
<br>
**Update** Getting FPS: Add following snippet in ```detect.py``` at Line number 192.
```bash
fps = 1 / (t2 - t1)
im0 = cv2.putText(im0, "Time: {:.2f}FPS".format(fps), (0, 30), cv2.FONT_HERSHEY_COMPLEX_SMALL, 1, (0, 0, 255), 2)
```
Now run the following command for inference on image
```bash
python detect.py --weight [path-to-yolov5]/yolov5/tflite/best-fp16.tflite --img 320 --source [path-to-yolo-v5]/data/images/test.jpg
```
Running the inference on video.
```bash
python detect.py --weight [path-to-yolov5]/yolov5/tflite/best-fp16.tflite --img 320 --source [path-to-yolo-v5]/data/images/test.mp4
```
Let's change the source to the webcam.
```bash
python detect.py --weight [path-to-yolov5]/yolov5/best-fp16.tflite --img 320 --source 0
```
The output will get saved in ```./yolov5/runs/detect```.<br>
I have added a tf model as well in the tflite folder. To test them just replace best-fp16.tflite with best.pb in the above commands.
