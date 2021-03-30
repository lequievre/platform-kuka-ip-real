#!/bin/bash

# Laurent LEQUIEVRE
# laurent.lequievre@uca.fr
# UMR 6602 - Institut Pascal
# Used for the FT Sensor n° 13855

cd /home/kuka/git_projects/platform-kuka-ip-real
source devel/setup.bash
export ROS_MASTER_URI=http://kuka-Precision-WorkStation-T7500:11311
roslaunch force_torque_sensor FTsensor_13855.launch
