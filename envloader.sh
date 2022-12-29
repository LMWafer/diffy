#!/bin/bash

source /opt/ros/noetic/setup.bash
source ~/diffy_ws/devel/setup.bash

export ROS_MASTER_URI=http://<HOST_IP>:11311
export ROS_HOSTNAME=<MY_IP>

exec "$@"
