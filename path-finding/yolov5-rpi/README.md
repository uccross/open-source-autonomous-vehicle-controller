## <div align="center">Instructions</div>
## Inferencing on Raspberry Pi using tflite
This method is used to run the tflite model without using tensorflow and pytorch dependencies.<br>
- (Optional) Create a virtual environment.
```bash
chmod +x run.sh
./run.sh
```
Now let's test the model. I have added the trained tflite weights in this folder and created separated scripts for video, image and webcam inference. <br>

Image inference <br>
```bash
python3 yolov5_tflite_image_inference.py --weights yolov5s-fp16.tflite -i [path-to-image] --img_size 320
```
Video Inference <br>
```bash
python3 yolov5_tflite_video_inference.py --weights yolov5s-fp16.tflite -v [path-to-video] --img_size 320
```


## Inferencing on coral edge
```bash
git clone https://github.com/bogdannedelcu/yolov5/
cd yolov5
git checkout tf-edgetpu
!pip install -r requirements.txt
!pip install tensorflow==2.4.1
```
Now change ```data/coco128.yaml``` as [coco128.yaml](https://github.com/uccross/open-source-autonomous-vehicle-controller/blob/dev_path-finding/path-finding/yolo-v5/data/coco128.yaml) and ```models/yolo5s.yaml``` as [yolo5s.yaml](https://github.com/uccross/open-source-autonomous-vehicle-controller/blob/dev_path-finding/path-finding/yolo-v5/models/yolov5s.yaml).  
Move the ```yolov5-rpi/yolov5s-int8_edgetpu.tflite``` and ```yolov5-rpi/yolov5s-int8.tflite``` into yolov5 folder.<br>
Test the INT8 model:
```bash
python3 detect.py --weight yolov5s-int8.tflite --img 320 --tfl-int8 --tfl-detect --source [path-to-image/video]
```
Inference
```bash
detect.py --weight yolov5s-int8_edgetpu.tflite --img 320 --tfl-int8 --tfl-detect --source [path-to-image/video] --edgetpu
```
