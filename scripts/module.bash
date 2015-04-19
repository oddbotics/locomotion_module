#!/bin/bash
source /opt/ros/indigo/setup.bash
source /home/oddbot/catkin_ws/devel/setup.bash

export ROS_MASTER_URI=http://locom:11311
export ROS_HOSTNAME=locom

sleep 2;

roscore &

#roslaunch locomotion_module locomotion.launch 
