[![License](https://img.shields.io/badge/License-BSD%202--Clause-orange.svg)](https://opensource.org/licenses/BSD-2-Clause)
# ROS Beginner tutorial 

---

## Overview
This project is a briefly introduction to some ROS concepts and core components.
The program creates two nodes, where one is the publisher while the other one 
is the subscriber.The goal of this work is to show the behavior and the 
interaction between this two nodes.
The publisher is the node talker, which will continually broadcast a message in to the  topic and
the subscriber listener will receive this message. 
This project also creates a service inside talker(publishing) node to change the string being published  at the topic.
The final task made by this work is creating a broadcast tf frame in the talker node, where it's frames are called /talk and /World. 

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

## How to run each node separately

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

In order to Run both nodes at the same time, it is possible to use
the launch file.

There is no need to initialize the master.

Inside catkin workspace

```
source devel/setup.bash
roslaunch beginner_tutorials launch2Nodes.launch

```
### Running nodes changing the publishing frequency

If the user desires to change the publishing rate in the topic,
Run the nodes using the launch file passing as argument the desired frequency

```
source devel/setup.bash
roslaunch beginner_tutorials launch2Nodes.launch frequency:=<frequency_argument_here>

```
## Calling service to Change Base String

This service changes the base string which is being published by the talker node.
 
With both nodes Running

Open new Terminal

```
cd ~/catkin_ws/
source devel/setup.bash
rosservice call /changeString "stringInput: '<New_desired_string_here>'"

```
## Inspecting TF frames

The TF broadcaster, inside talker node, is broascasting the transformation between the World frame to talkWithFrame frame. The orientation of the of talkWithFrame with respect to the world frames varies with the time , however the translation is kept the same.
In order to check the transformation we can use the "tf_echo" tool.
Before running the command this tool, is necessary to run the talker node and for that you can use any of the methods previously mentioned in this file.

In a new terminal 

```
rosrun tf tf_echo /world /talk

```

To visualize the two frames in a graph form:

```
rosrun rqt_tf_tree rqt_tf_tree

```

To generate a pdf file with a similar tree graph

```
rosrun tf view_frames

```

In order to check this generated pdf  


```
evince frames.pdf

```

## running rostest

There is one test in this project, which consists in verifying if the service is being called properly and that the the stringOutput matchs the stringInput.

To build the code and run the test at the same time 

```
cd ~/catkin_ws
catkin_make run_tests_beginner_tutorials

```

If the code is already compiled , it is possible to run the test independly

```
cd ~/catkin_ws
source devel/setup.bash
rostest beginner_tutorials test.launch

```

Another way of running the test 

The talker node needs to be running

```
cd ~/catkin_ws
source devel/setup.bash
rosrun beginner_tutorials testTalkerNode

```

## Recording bag files with the launch file

This section uses the launch file to activate the recording of  the topic data from a running ROS system, and will accumulate the data in a bag file.
The bag file called recorded.bag will be saved at the "results" folder.
In order to record the topics of the system, run the launch file passing the argument "enable"

```
cd ~/catkin_ws
source devel/setup.bash
roslaunch beginner_tutorials launch2Nodes.launch record:=enable

```

If it is not desired recording the topics just Run the launch file without the argument record

```
cd ~/catkin_ws
source devel/setup.bash
roslaunch beginner_tutorials launch2Nodes.launch 

```
or 

pass "disable" to the argument record 

```
cd ~/catkin_ws
source devel/setup.bash
roslaunch beginner_tutorials launch2Nodes.launch record:=disable

```

## Inspecting the bag file

To inspect the recorded.bag , to check the details of what was recorded.

```
cd ~/catkin_ws
cd src/beginner_tutorials/results/
rosbag info recorded.bag

```

## Playing the recorded bag

To play the recorded bag and visualize part of what was recorded , we need first run the listener node.

Make sure talker node is not running 

```
cd ~/catkin_ws
source devel/setup.bash
rosrun beginner_tutorials listener

```

After running listener node, play the recorded bag

```
cd ~/catkin_ws
cd src/beginner_tutorials/results/
rosbag play recorded.bag

```
Open the terminal where the listener node is running and it will be possible to visualize all the messages recorded at the topic /chatter.








