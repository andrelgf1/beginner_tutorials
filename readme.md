# ROS Beginner tutorial 

---

## Overview
This projec is a brefly introduction to some ROS concepts and core components.
The program creates two nodes, where one is the publisher while the other one 
is the subscriber.The goal of this work is to show the behavior and the 
interaction between this two nodes.
The publisher is the node talker, which will continually broadcast a message in to the  topic and
the subscriber listener will recieve this message.  

## Dependencies




### Install ROS kinetic

In order to Install ROS kinect follow the following ROS.org [link](http://wiki.ros.org/kinetic/Installation/Ubuntu)

### Install catkin
In order to Install catkin follow the following ROS.org [link](http://wiki.ros.org/catkin#Installing_catkin)


## How to build

Create and build a catkin workspace 
```
 mkdir -p ~/catkin_ws/src
 cd ~/catkin_ws/
 catkin_make
 
```
Build Project
```
 cd ~/catkin_ws/
 source devel/setup.bash
 cd src
 git clone https://github.com/andrelgf1/beginner_tutorials.git
 cd ~/catkin_ws/
 catkin_make
```

## How to run

once your environment is set
open the terminal and initialize the master
 
```
roscore

```
In a new terminal, lets run the publisher talker

```
 cd ~/catkin_ws/
 source devel/setup.bash
 rosrun beginner_tutorials talker

```
In a new terminal, lets run the Subscriber listener
```
 cd ~/catkin_ws/
 source devel/setup.bash
 rosrun beginner_tutorials listener

```



