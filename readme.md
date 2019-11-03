[![License](https://img.shields.io/badge/License-BSD%202--Clause-orange.svg)](https://opensource.org/licenses/BSD-2-Clause)
# ROS Beginner tutorial 

---

## Overview
This projec is a brefly introduction to some ROS concepts and core components.
The program creates two nodes, where one is the publisher while the other one 
is the subscriber.The goal of this work is to show the behavior and the 
interaction between this two nodes.
The publisher is the node talker, which will continually broadcast a message in to the  topic and
the subscriber listener will recieve this message. 
This projec creates a service inside talker(publishing) node to change the string being published  at the topic. 

## License

BSD License
Copyright 2019 Andre Ferreira

```
Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

```


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
Build Project in Catkin Workspace
```
 cd ~/catkin_ws/
 source devel/setup.bash
 cd src
 git clone https://github.com/andrelgf1/beginner_tutorials.git
 cd ~/catkin_ws/
 catkin_make
```

## How to run each node separatly

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
## How to run nodes using launch file

Inside catkin workspace

```
source devel/setup.bash
roslaunch beginner_tutorials launch2Nodes.launch

```
### Running nodes changing the publishing frequency

```
source devel/setup.bash
roslaunch beginner_tutorials launch2Nodes.launch frequency:=<frequency_argument_here>

```
### Calling service to Change Base String
With both nodes Running

Open new Terminal

```
cd ~/catkin_ws/
source devel/setup.bash
rosservice call /changeString "stringInput: '<New_desired_string_here>'"

```



