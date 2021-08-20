## <div align="center">Running on Coral using tflite-runtime</div>

* Installing the tflite-runtime
```bash
echo "deb https://packages.cloud.google.com/apt coral-edgetpu-stable main" | sudo tee /etc/apt/sources.list.d/coral-edgetpu.list
curl https://packages.cloud.google.com/apt/doc/apt-key.gpg | sudo apt-key add -
sudo apt-get update
sudo apt-get install python3-tflite-runtime
```

Image inference

```bash
python3 yolov5_tflite_image_inference.py --weights yolov5s-fp16_edge.tflite -i [path-to-image] --img_size 320
```
Video Inference

```bash
python3 yolov5_tflite_video_inference.py --weights yolov5s-fp16_edge.tflite -v [path-to-video] --img_size 320
```