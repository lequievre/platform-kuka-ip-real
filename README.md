Laurent LEQUIEVRE<br/>
Research Engineer, CNRS (France)<br/>
Institut Pascal UMR6602<br/>
laurent.lequievre@uca.fr<br/>

# Kuka platform - Institut Pascal - UMR6602.
## This was originally forked from CentroEPiaggio/kuka-lwr.


### How to start

#### 1- Start the network
Open a Terminal Window<br/>
cd<br/>
cd Bureau<br/>
./1_start_network.sh<br/>

#### Start KRL script on Kuka KRC computer
- 1- Put KCP en mode Automatique (switch the key to a spiral without a point on top)<br/>

- 2- Ackn All the messages<br/>
 
- 3- Start a KRL file (For the left Arm, for example) :<br/>
FRIDEMO/IP/FRICONTROLNOTOOL<br/>

- (For the right Arm, for example) :<br/>
FRIDEMO/test-laurent/FRIControl<br/>

- 4- Decrease speed to 30% top right (use KCP button)<br/>

- 5- Press the button I until the letter I at the bottom left turns green. (use KCP button)<br/>

- 6- Press the button green + one time (use KCP button) (multiple times if the script has multiple halt instruction) until to be at the WAIT instruction.<br/>

#### 2- Load ROS controllers and ros control hardware configuration with a script
Open a Terminal Window<br/>
cd<br/>
cd Bureau<br/>
./2_start_loading_controllers_left.sh<br/>
or<br/>
./2_start_loading_controllers_right.sh<br/>
or<br/>
./2_start_loading_controllers_both.sh<br/>

The script finish with the line :<br/>
"Controller Spawner: Loaded controllers: .... names of the controllers ...."


#### 3- Start a controller with rqt plugin

Open a Terminal Window<br/>
cd ~/git_project/platform-kuka-ip-real<br/>
source devel/setup.bash<br/>
rqt<br/>

Menu Plugins/Platform Sigma Plugins/Controller Manager<br/>

Choose the namespace 'kuka_lwr_left' or 'kuka_lwr_right'<br/>
Click right on a controller name and click left 'start controller'<br/>


#### 4- Examples of topics

- Position controller :<br/>
rostopic pub -1 /kuka_lwr_right/joint_position_controller/command std_msgs/Float64MultiArray "data: [0.9,0.9,0.9,0.7,0.5,0.5,0.5]"

- Torque controller :<br/>

rostopic pub -1 /kuka_lwr_left/torque_based_position_controller/command std_msgs/Float64MultiArray "data: [0.9,0.9,0.5,0.5,0.3,0.5,0.8]"

rostopic pub -1 /kuka_lwr_left/torque_based_position_controller/setKp std_msgs/Float64MultiArray "data: [100,200,100,100,50,50,50]"

rostopic pub -1 /kuka_lwr_left/torque_based_position_controller/setKd std_msgs/Float64MultiArray "data: [50,50,50,50,10,10,10]"

rostopic pub -1 /kuka_lwr_left/torque_based_position_controller/setStiffness std_msgs/Float64MultiArray "data: [200,200,200,200,200,200,200]"
rostopic pub -1 /kuka_lwr_left/torque_based_position_controller/setDamping std_msgs/Float64MultiArray "data: [0.7,0.7,0.7,0.7,0.7,0.7,0.7]"


- Cartesian controller :
rostopic pub -1 /kuka_lwr_left/kuka_one_task_inverse_kinematics/command kuka_lwr_controllers/PoseRPY '{id: 1, position: {x: -0.4, y: 0.3, z: 0.9}}'

rostopic pub -1 /kuka_lwr_left/kuka_one_task_inverse_kinematics/command kuka_lwr_controllers/PoseRPY '{id: 1, position: {x: 0.2, y: 0.7, z: 0.5}}'


- How to start and stop manually a controller :


rosservice call /kuka_lwr_left/controller_manager/switch_controller "{start_controllers: ['joint_position_controller'], stop_controllers: [], strictness: 2}"
rosservice call /kuka_lwr_left/controller_manager/switch_controller "{start_controllers: [], stop_controllers: ['joint_position_controller'], strictness: 2}"

rostopic pub -1 /kuka_lwr_left/joint_position_controller/command std_msgs/Float64MultiArray "data: [0.9,0.9,0.9,0.7,0.5,0.5,0.5]"



rosservice call /kuka_lwr_right/controller_manager/switch_controller "{start_controllers: ['joint_position_controller'], stop_controllers: [], strictness: 2}"
rosservice call /kuka_lwr_right/controller_manager/switch_controller "{start_controllers: [], stop_controllers: ['joint_position_controller'], strictness: 2}"

rostopic pub -1 /kuka_lwr_right/joint_position_controller/command std_msgs/Float64MultiArray "data: [0.9,0.9,0.9,0.7,0.5,0.5,0.5]"


#### 5- Follow Arms motion into Rviz
On a another computer<br/>
cd ~/git_project/platform-kuka-ip-real<br/>
source devel/setup.bash<br/>
export ROS_MASTER_URI=http://kuka-Precision-WorkStation-T7500:11311<br/>
roslaunch platform_rviz platform_rviz.launch<br/>


