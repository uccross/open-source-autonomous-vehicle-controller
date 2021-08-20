Above folders will have two types of files
- images(.jpg)
- Annotations(.xml)

I have generated the annotations using [LabelImg](https://github.com/tzutalin/labelImg) tool. <br>
TensorFlow uses ```tfrecords```(a simple format for storing a sequence of binary records)
to train an ML model. Using the images and annotaions ```tfrecords``` will be generated.