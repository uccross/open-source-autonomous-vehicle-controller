## <div align="center">Cone Detection on Raspberry Pi using Yolov5</div>
You can try running the model in Google Colaboratory [here](data/Notebook/EfficientDet_0.ipynb)
Run the following from the root of the cloned repository.<br>
Install [Tensorflow](https://www.tensorflow.org/install) and the [Object Detection API](https://tensorflow-object-detection-api-tutorial.readthedocs.io/en/latest/) (including protobuf) following the instructions. To train your own model follow the steps below:<br>
To use our pre-existing workspace download the [folder](https://drive.google.com/drive/folders/15Ox6XZGKtxiFpEPpOfzffWUlkTCKG4vx?usp=sharing) and skip to performing Inference. To train your custom model download your data:
```
cd <PATH_TO_REPO>/open-source-autonomous-vehicle-controller/path-finding
chmod u+x scripts/install/env.sh
source scripts/install/env.sh

cp data/images/train_images/annotations/* data/images/train_images/images/
cp data/images/test_images/annotations/* data/images/test_images/images/
python scripts/training/xml_to_csv.py xml data/images/train_images/images/ data/annotations/train_labels.csv
python scripts/training/xml_to_csv.py xml data/images/test_images/images/ data/annotations/test_labels.csv

python scripts/training/generate_tfrecord.py -x data/images/train_images/images/ -l data/annotations/label_map.pbtxt -o annotations/train.record
python scripts/training/generate_tfrecord.py -x data/images/test_images/images/ -l data/annotations/label_map.pbtxt -o annotations/test.record
```
Download model extract and give path in pipeline.config
Make an empty folder where trained model checkpoints will be stored
Give these folder and pipeline config paths in the command below
```
cd models/EfficientDet0
mkdir annotations
mkdir models
cp ../../data/annotations/* annotations/
wget http://download.tensorflow.org/models/object_detection/tf2/20200711/efficientdet_d0_coco17_tpu-32.tar.gz
tar -xvf efficientdet_d0_coco17_tpu-32.tar.gz
mkdir models/efficientdet_d0_coco17_tpu-32 
cp efficientdet_d0_coco17_tpu-32/pipeline.config models/efficientdet_d0_coco17_tpu-32/
python model_main_tf2.py --model_dir=models/efficientdet_d0_coco17_tpu-32 --pipeline_config_path=models/efficientdet_d0_coco17_tpu-32/pipeline.config

python exporter_main_v2.py --input_type image_tensor --pipeline_config_path models/efficientdet_d0_coco17_tpu-32/pipeline.config --trained_checkpoint_dir models/efficientdet_d0_coco17_tpu-32/ --output_directory final_model/
python video_inference.py  --model final_model --labels annotations/label_map.pbtxt 

```