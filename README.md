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
- For the left Arm<br/>
FRIDEMO/IP/FRICONTROLNOTOOL<br/>

- For the right Arm<br/>
FRIDEMO/test-laurent/FRIControl<br/>

#### 2- Load ROS controllers and ros control hardware configuration
Open a Terminal Window<br/>
cd<br/>
cd Bureau<br/>
./2_start_loading_controllers_left.sh<br/>

or<br/>

./2_start_loading_controllers_right.sh<br/>

