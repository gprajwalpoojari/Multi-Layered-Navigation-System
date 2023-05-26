# Multi Layered Navigation System for Non-Holonomic Vehicles

## Results
Example animation of the result:

![motion-planning](https://github.com/gprajwalpoojari/Multi_Layer_Motion_Planning/assets/53962958/5c0f571f-0287-4ff1-ab72-b2269cc7db24)

## Environment
ROS Melodic on Ubuntu 18.04

## Instructions
1. Setup package in ROS environment, or setup packas as your ROS environment
   1. Instructions for setting up [ROS Environment](http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment)
2. Make sure the packages under "Main Dependent ROS packages" are installed
3. cd into your environment and Build the environment with `catkin_make`
4. Source your environment with `source devel/setup.bash`
5. Run the ROS package with `roslaunch vgraph_environment vgraph_environment.launch`

## Dependent ROS packages for "vgraph_environment"
- rospy
- roscpp
- roslib
- catkin
- visualization_msgs
- turtlebot3_fake


## Brief explanation of the approach

- Obstacle Avoidance using Visibility Graph Construction
- Global Planner - A-Star
- Local Planner - Artificial Potential Field

https://user-images.githubusercontent.com/53962958/216720721-55aac6e2-175d-442e-ae4a-84a20a199fdc.mp4


