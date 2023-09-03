# Self-balancing bot simulation

This is a self-balancing bot simulation using Gazebo and ros for control. The bot is controlled using a PD controller with arm joint angle as feedback.

This simulation uses ROS melodic and matplotlib

Please find instructions for installing ROS. [Here](http://wiki.ros.org/melodic)

After installing ROS. you can copy the balancing bot file into your src folder in your ros workspace and build it.

then run the commands for running gazebo and control ros node.

```
roslaunch balancing_bot gazebo.launch 
```
You should be able to see the self-balancing bot simulation in gazebo like in the image below

![gazebo](gazebo_sample.png)

In another terminal run

```
rosrun balancing_bot example_pd_controller.py 
```

After running the last command, you should be able to balance the arm link.

<video src="example_pd.mp4" controls="controls" style="max-width: 730px;">
</video>

With the error decreasing over time like this,

![error](sample.png)

The PD controller block diagram here explains the control loop, where the error of angle difference is fed into the controller. Motor speed command is generated for the left and right motors individually using the controller output and target motor speed.

![pid](blk_diag.png)



