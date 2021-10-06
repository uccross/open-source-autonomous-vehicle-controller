TensorFlow uses tfrecords(a simple format for storing a sequence of binary records)
to train an ML model. Using ```generate_tfrecord.py``` and annotations, ```train.record``` and ```test.record``` are generated.
Before running ```generate_tfrecord.py``` following steps are done:
- Clone [models](https://github.com/tensorflow/models) in ```~/path-finding```.
- Install ```models/research/object_detection``` in the path envrionment. This includes compiling protobuf files(Protobuf files are used by TensorFlow to configure model and training parameters).

For generating ```tfrecord```:<br>
```!python generate_tfrecord.py -x [PATH_TO_TRAIN_IMAGES] -l [PATH_TO_LABELMAP] -o [PATH_TO_DATA]/train.record```<br>
```!python generate_tfrecord.py -x [PATH_TO_TEST_IMAGES] -l [PATH_TO_LABELMAP] -o [PATH_TO_DATA]/test.record```<br>

