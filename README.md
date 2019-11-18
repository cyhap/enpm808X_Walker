# enpm808X_Walker
The walker node for enpm808x HW12. Commands a Turtlebot to navigate with motion similar to a roomba.
## Overview:
This ROS Package contains source code to create two rosnodes titled talker and listener. One node simply publishes a string message at a 10 Hz rate to a chatter topic. The other node subscribes the chatter topic and prints "I heard:" followed by the string message that it observed being published to the chatter topic.

## Dependencies:
This ros package was developed in ros-kinetic. No other ros distributions are guaranteed to support this package. Note the source code expects to be built using the c++11 standard. (Outlined in the CMakelist.txt file)

## Build instructions:
In order to use the rosnodes provided by this package they must first be built in a catkin workspace. For the instructions the Catkin Workspace directory shall be referred to as <Catkin Workspace> and should be replaced with the users full path to their Catkin Workspace.
- Navigate to your catkin_ws src directory.
```
cd <Catkin Workspace>/src
```

- Pull the package from github.
```
git clone https://github.com/cyhap/enpm808x_walker.git
```
- The enpm808x_walker directory should now be in your <Catkin_Workspace>/src folder.

- Change directories to <Catkin_Workspace> and run catkin_make install
```
cd <Catkin_Workspace>
catkin_make install
```
- Note the install is only necessary the first time the package is built to ensure that the services are generated and installed.
- Make sure that your catkin workspace is sourced in order to run this package in another terminal. 
```
source <Catkin_Workspace>/devel/setup.bash
```

## Run instructions:
- Open a terminal and being running roscore
```
roscore
```
- Open a new terminal and make sure the <Catkin_Workspace>/devel/setup.bash has been sourced. In order to find the newly built packages.
```
source <Catkin_Workspace>/devel/setup.bash
```
### Talker Node:
- Begin Running the walker Node. Note this node expects the /scan topic to be present. Make sure that is before running this node.
```
rosrun enpm808x_walker walker
```
- The messages will be published to cmd_vel. Note some remapping may be necessary depending on your required usage.

## Both Nodes Simultaneously (USING ROS LAUNCH):
- Open a new terminal and again make sure that the <Catkin_Workspace>/devel/setup.bash has been sourced.
```
source <Catkin_Workspace>/devel/setup.bash
```
- In order to observe the roomba like behaviour of the turtlebot. Simply use the following command in your terminal:
```
roslaunch enpm808x_walker hw-12.launch
```
- There is one optional argument for the roslaunch commandline input. It allows the user to record all active topics (except camera/rgb/* topics) when this launched. Be wary that if other nodes are running their published topics will be captured in the bag file as well. An example using the argument is provided below:
```
roslaunch enpm808x_walker hw-12.launch recordChatter:=1
```
The recordChatter argument allows you to record a bag file titled walkerAll.bag.
Note the rosbag file will be located under the ~/.ros directory.

## Bag File Playback
To inspect the contents of the bagfile run the following command:
```
rosbag info <Path to bag File in Results Folder>
```

One can start playing the bagfile.
```
rosbag play <Path to bag File in Results Folder>
```
Then one can run the walker node as specified in the instructions above. The output twist messages may be observed by monitoring the cmd_vel topic

Saved bag file can be found using this link:
https://drive.google.com/file/d/1OH68giXii9qfq6x2_J-zk5q0zkfTzu27/view?usp=sharing
