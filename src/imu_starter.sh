#!/bin/sh

export ROS_MASTER_URI="http://10.0.0.1:11311"
export ROS_IP="10.0.0.2"

/home/yonder/catkin_ws/src/2018_URC/src/onboard_imu.py
