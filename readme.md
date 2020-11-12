[![License: MIT](https://img.shields.io/badge/License-MIT-blue.svg)](https://opensource.org/licenses/MIT)
---
# Beginner Tutorials ROS Package for ROS Services, Logging and Launch files

## Overview and Description

This repository contains Publisher/Subscriber ROS package which publishes simple string message. Also creation of ROS Services, Logging and Launch files

## License

MIT License

## Author

Karan Sutradhar

## Dependencies/Assumptions

This ROS Melodic package was created and tested on ubuntu 18.04 (Linux).
C++ 11
The catkin_make is used for building this package.

## Tutorials to create and build a package with Publisher and Subscriber
http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28c%2B%2B%29

## Steps to run this package

Open new terminal window and type the following:
```
mkdir catkin_ws/src
cd catkin_ws/src
git clone https://github.com/Karansutradhar/beginner_tutorials.git
cd ..
source /opt/ros/melodic/setup.bash    # Replace melodic with your distro name
catkin_make
source ~/catkin_ws/devel/setup.bash
roscore
```
Open a new terminal window and type:
```
cd catkin_ws
source devel/setup.bash
rosrun beginner_tutorials talker
```
Open a new terminal window and type:
```
cd catkin_ws
source devel/setup.bash
rosrun beginner_tutorials listener
``` 
To end the process, type ctrl+C on all the terminal windows one by one.

## ROS Service Instructions

After executioon, open a new terminal and type:
```
cd catkin_ws
catkin_make
source devel/setup.bash
rosservice call /UpdateString "Enter the new string here"
**NOTE: rosservice call /UpdateString "Working on ROS"
```
As soon as you enter the new text, there would be a change in the message output.

### ROS Launch Instructions
```
cd catkin_ws
catkin_make
source devel/setup.bash
roslaunch beginner_tutorials stringOutput.launch 
```

### Variation in Loop Frequency

Change the frequency parameter at the end after executing the following commands:
```
cd catkin_ws
source devel/setup.bash
roslaunch beginner_tutorials stringOutput.launch frequency: "integer above zero"
**NOTE: roslaunch beginner_tutorials stringOutput.launch frequency:=10
```
### Tf frames Running & Inspection Instructions
In one terminal
```
cd catkin_ws
source devel/setup.bash
rosrun beginner_tutorials talker
```
In second terminal
```
cd catkin_ws
source devel/setup.bash
rosrun tf tf_echo /world /talk
```
PDF file generation
```
cd catkin_ws
source devel/setup.bash
rosrun tf view_frames
```
To inspect
```
cd catkin_ws
source devel/setup.bash
rosrun rqt_tf_tree rqt_tf_tree

```
### Instructions for running ROStest
You will see test passing
```
cd catkin_ws
source devel/setup.bash
catkin_make run_tests_beginner_tutorials

```
### Instructions for recording ROSbag
In terminal one
```
cd catkin_ws
source devel/setup.bash
roslaunch beginner_tutorials beginner_tutorials.launch frequency:=10 record:=true
```
### Instructions for running ROSbag file
In terminal one
```
roscore
```
In terminal two
```
cd catkin_ws
source devel/setup.bash
rosbag play src/beginner_tutorials/screenshot/my_rosbag_file.bag
```
In terminal third
```
cd catkin_ws
source devel/setup.bash
rosrun beginner_tutorials listener
```
### Instructions for Inspection of ROSbag file
In terminal 1
```
cd catkin_ws/src/beginner_tutorials/results
rosbag play my_rosbag_file.bag

```
In terminal 2
```
cd catkin_ws
source devel/setup.bash
rosrun beginner_tutorials listener

```
## Cpplint check
```
cd  <path to repository>
cpplint $( find . -name \*.hpp -or -name \*.cpp | grep -vE -e "^./build/" -e "^./vendor/" -e "^./docs/" -e "^./results" )

```

## Cppcheck check
```
cd <path to repository>
cppcheck --enable=all --std=c++11 -I include/ --suppress=missingIncludeSystem $( find . -name *.cpp | grep -vE -e "^./build/" -e "^./vendor/" )

```
