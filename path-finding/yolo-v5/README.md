## <div align="center">Cone Detection on Raspberry Pi using Yolov5</div>
Code for detecting cones on Raspberry Pi using the [Yolov5](https://github.com/ultralytics/yolov5) algorithm. <br>
For starters, we will begin by yolov5-pytorch inference on the Raspberry-Pi<br>

```bash
$ git clone https://github.com/uccross/open-source-autonomous-vehicle-controller
$ git checkout dev_path-finding
$ pip install -r requirements.txt
```
From ```path-finding``` folder copy the yolov5 folder on the desired location.<br>

```bash
$ conda create --name test python=3.7
$ conda activate test
$ pip install -r requirements.txt
```
Now, yolov5 model is installed in the device. We are ready for inference.<br>
I have trained two different models, one with the lower(256x320) and one with higher(320x416) resolution.<br>
Running the inference on a image.

```bash
$ python detect.py --weights [weight-path] --conf 0.25 --img 320 --source [path-to-yolo-v5]/data/images/test.jpg
```

Running the inference on a video.
```bash
$ python detect.py --weights [weight-path] --conf 0.25 --img 320 --source [path-to-yolo-v5]/data/videos/test.mp4
```
Now, for changing the source to webcam, just give 0 or 1 for the ```--source``` in the command.
```bash
$ python detect.py --weights [weight-path] --conf 0.25 --img 320 --source 0
```

The output will get saved in exp folder in ```./yolo-v5/runs/detect/```.
