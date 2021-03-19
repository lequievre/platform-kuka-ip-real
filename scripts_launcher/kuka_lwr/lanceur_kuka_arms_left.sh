#!/bin/bash

# Laurent LEQUIEVRE
# laurent.lequievre@uca.fr
# UMR 6602 - Institut Pascal

export ROS_MASTER_URI=http://kuka-Precision-WorkStation-T7500:11311
cd /home/kuka/git_project/platform-kuka-ip-real
source devel/setup.bash
roslaunch double_lwr_robot double_arms.launch use_right_arm:=false
