# rostolabelimg

ROS package classifies objects from image topic inside rosbag and generates labelimg file structure

## roslaunch files

To run the software make sure roscore is running.

```
roscore
```

Run the launch file and supply the rosbag location via the parameter 

```
roslaunch rostolabelimg annotate_bag.launch bag_location:=/home/tomos/ros/wheelchair/catkin_ws/src/wheelchair_dump/dump/1DLF.bag
```
