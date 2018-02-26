#!/bin/bash

### BEGIN INIT INFO
# Provides:          imu_starter
# Required-Start:    $network $syslog
# Required-Stop:     $network $syslog
# Default-Start:
# Default-Stop:
# Short-Description: Start up imu ROS node
# Description:       Start up IMU ROS node. May be expanded later.
### END INIT INFO

# run command below to add to init lists (not sure when necessary)
# sudo update-rc.d imu_starter.sh defaults

source /opt/ros/kinetic/setup.bash
source /home/yonder/catkin_ws/devel/setup.bash

export ROS_MASTER_URI="http://10.0.0.1:11311"
export ROS_IP="10.0.0.2"

echo $ROS_MASTER_URI
echo $ROS_IP

case "$1" in
  start)
    echo "Starting ROS nodes"
    # run application you want to start
    /home/yonder/catkin_ws/src/2018_URC/src/onboard_imu.py 2>> /home/yonder/STARTUP_ERR_LOG.log
    ;;
  *)
    echo "Usage: /etc/init.d/imu_starter.sh start"
    exit 1
    ;;
esac


