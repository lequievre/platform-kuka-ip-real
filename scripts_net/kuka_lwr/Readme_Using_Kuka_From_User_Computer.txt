Laurent LEQUIEVRE
Institut Pascal umr6602
laurent.lequievre@uca.fr

0- Add Ip address and name of the user's laptop to /etc/hosts of kuka-Precision-WorkStation-T7500 (kuka computer) :
192.168.100.105 HP-8570w
(In that case the name of my laptop is 'HP-8570w' and its IP address is always 192.168.100.105)

1- Launch the network bash on all computers of the platform :
cd Bureau
./1_start_network.sh

2-Deactivate network and wifi of the user's laptop.

3- Launch the network configuration from the user's laptop :
see and adapt the script sample net_user_kuka_ip.h saved in the directory :
~/git_project/platform-kuka-ip-real/src/scripts_net/kuka_lwr

Adapt the number of your eth :
ethX_associate_switch="eth0"

Adapt the hardware address of eth0 : 
hw_ethX_associate_switch="2c:44:fd:68:fb:58"

4- Add Ip address and name of kuka-Precision-WorkStation-T7500 (kuka computer) and the other computers of the platform
to /etc/hosts of user's laptop :
sudo nano /etc/hosts
192.168.100.123 kuka-Precision-WorkStation-T7500
192.168.100.113 hand-OptiPlex-7040
192.168.100.103 kuka-OptiPlex-7040

kuka-Precision-WorkStation-T7500 -> kuka computer
hand-OptiPlex-7040 -> shadow hand computer
kuka-OptiPlex-7040 -> force sensor computer

5- On each new terminal window on user's laptop, set the ROS MASTER URI :
export ROS_MASTER_URI=http://kuka-Precision-WorkStation-T7500:11311

rostopic list
rostopic pub ....




