# Beginner Tutorials ROS Package for simple Publisher/Subscriber

## Overview and Description

This repository contains Publisher/Subscriber ROS package which publishes simple string message.

## License

MIT License

## Author

Karan Sutradhar

## Dependencies/Assumptions

This ROS Melodic package was created and tested on ubuntu 18.04 (Linux).
The catkin_make is used for building this package.
No other dependencies required.

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
rosrun beginner_tutorials talker
```
Open a new terminal window and type:
```
cd catkin_ws
rosrun beginner_tutorials listener
``` 
To end the process, type ctrl+C on all the terminal windows one by one.

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
