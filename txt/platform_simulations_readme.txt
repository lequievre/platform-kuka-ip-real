Laurent LEQUIEVRE
Research Ingineer CNRS, France
Institut Pascal UMR6602
laurent.lequievre@uca.fr

Platform simulation with gazebo (arms only) :
===========================================

source devel/setup.bash
roslaunch platform_gazebo platform_gazebo.launch use_pantilt:=true

source devel/setup.bash
rqt
Plugins/Platform Sigma Plugins/Controller Manager (to start arms controller -> kuka_group_command_controller_fri)
Plugins/Platform Sigma Plugins/Joint Position     (to move arms joints)

How to move pantilt :
-------------------
source devel/setup.bash
rostopic pub -1 /pantilt/pantilt_group_position_controller/command std_msgs/Float64MultiArray "data: [1.0,-0.5,0.0,0.0]"

How to get images from 2 cameras :
--------------------------------
source devel/setup.bash
rosrun image_view image_view image:=/pantilt/camera1/image_raw
rosrun image_view image_view image:=/pantilt/camera2/image_raw

Platform simulation with RVIZ (arms only) :
=========================================

roslaunch platform_rviz platform_rviz.launch

Platform simulation with gazebo (arms and hands) :
================================================
'sh' means 'Shadow Hand', 'bh' means 'Barrett Hand'. 

-> 1 shadow hand left and 1 shadow hand right :
roslaunch platform_gazebo platform_gazebo_with_hands.launch use_left_sh:=true use_right_sh:=true

-> 1 shadow hand left and 1 barrett hand right :
roslaunch platform_gazebo platform_gazebo_with_hands.launch use_left_sh:=true use_right_bh:=true

How to move arms :
-----------------
rqt
Plugins/Platform Sigma Plugins/Controller Manager  (to start arms controller -> kuka_group_command_controller_fri)
Plugins/Platform Sigma Plugins/Joint Position		(to move arms joints)

How to move shadow hands : (namespace 'rsh' means 'right shadow hand', namespace 'lsh' means 'left shadow hand')
------------------------

Plugins/Shadow Robot Bi Manual/Joint Slider Left Hand	(to move left shadow hand joints) 
Plugins/Shadow Robot Bi Manual/Joint Slider Right Hand  (to move right shadow hand joints) 

rostopic echo /rsh/joint_states

Contacts: (namespace 'rh' means 'right hand', namespace 'lh' means 'left hand')
--------
rostopic echo /contacts/rh_ff/distal
rostopic echo /contacts/rh_ff/knuckle
rostopic echo /contacts/rh_ff/middle
rostopic echo /contacts/rh_ff/proximal
rostopic echo /contacts/rh_lf/distal
rostopic echo /contacts/rh_lf/knuckle
rostopic echo /contacts/rh_lf/metacarpal
rostopic echo /contacts/rh_lf/middle
rostopic echo /contacts/rh_lf/proximal
rostopic echo /contacts/rh_mf/distal
rostopic echo /contacts/rh_mf/knuckle
rostopic echo /contacts/rh_mf/middle
rostopic echo /contacts/rh_mf/proximal
rostopic echo /contacts/rh_palm
rostopic echo /contacts/rh_rf/distal
rostopic echo /contacts/rh_rf/knuckle
rostopic echo /contacts/rh_rf/middle
rostopic echo /contacts/rh_rf/proximal
rostopic echo /contacts/rh_th/base
rostopic echo /contacts/rh_th/distal
rostopic echo /contacts/rh_th/hub
rostopic echo /contacts/rh_th/middle
rostopic echo /contacts/rh_th/proximal


How to move Barrett Hand : (namespace 'rbh' means 'right barrett hand', namespace 'lbh' means 'left barrett hand')
------------------------
rostopic pub -1 /rbh/rbh_group_position_controller/command std_msgs/Float64MultiArray "data: [0.0,0.0,0.0,0.0]"

break away service -> rosservice call /rbh/rbh_group_position_controller/setBreakAway true

rostopic echo /rbh/joint_states

Contacts :
--------
rostopic echo /contacts/rbh/finger1/distal
rostopic echo /contacts/rbh/finger1/knuckle
rostopic echo /contacts/rbh/finger1/proximal
rostopic echo /contacts/rbh/finger2/distal
rostopic echo /contacts/rbh/finger2/knuckle
rostopic echo /contacts/rbh/finger2/proximal
rostopic echo /contacts/rbh/finger3/distal
rostopic echo /contacts/rbh/finger3/proximal
rostopic echo /contacts/rbh/palm






