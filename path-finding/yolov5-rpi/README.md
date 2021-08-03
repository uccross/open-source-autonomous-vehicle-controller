## <div align="center">Instructions</div>
This method is used to run the tflite model without using tensorflow and pytorch dependencies.<br>
- (Optional) Create a virtual environment.
```bash
chmod +x run.sh
./run.sh
```
Now let's test the model. I have added the trained tflite weights in this folder and created separated scripts for video, image and webcam inference. <br>

Image inference <br>
```bash
python3 yolov5_tflite_image_inference.py -weights yolov5s-fp16.tflite -i test/bus.jpg --img_size 320
```
Video Inference <br>
```bash
python3 yolov5_tflite_video_inference.py -weights yolov5s-fp16.tflite -i [path-to-video] --img_size 320
```
