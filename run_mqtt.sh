#!/bin/bash
source /opt/ros/melodic/setup.bash
export PYTHONPATH=/opt/ros/melodic/lib/python3/dist-packages:$PYTHONPATH

export ROS_MASTER_URI=http://localhost:11311
export ROS_PACKAGE_PATH=/opt/ros/melodic/share

exec python3 /root/scout-files/mqtt_client.py
