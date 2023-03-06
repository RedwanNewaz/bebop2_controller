#!/usr/bin/env bash 

LD_LIBRARY_PATH=~/catkin_ws/devel/lib/parrot_arsdk:$LD_LIBRARY_PATH &&source ~/catkin_ws/devel/setup.bash

rosrun bebop2_controller bebop2_controller_gui