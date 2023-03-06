#!/usr/bin/env bash 

#setup the ROS environment
LD_LIBRARY_PATH=~/catkin_ws/devel/lib/parrot_arsdk:$LD_LIBRARY_PATH &&source ~/catkin_ws/devel/setup.bash
#launch roscore in separate thread
coproc(roscore)
#launch the main program
rosrun bebop2_controller bebop2_controller_gui