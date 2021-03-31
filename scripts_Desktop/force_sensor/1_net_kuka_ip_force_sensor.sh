#!/bin/sh

# Laurent LEQUIEVRE
# laurent.lequievre@univ-bpclermont.fr
# UMR 6602 - Institut Pascal

# Ip address of Pan Tilt -> 192.168.100.252

# NETWORK INFORMATIONS
# ====================
# sudo lshw -C network

# eth0 -> 48:4d:7e:ac:cf:9c , Ethernet Connection (2) I219-LM
# eth1 -> 68:05:ca:1c:d7:c8 , 82571EB/82571GB Gigabit Ethernet Controller D0/D1
# eth2 -> 68:05:ca:1c:d7:c9 , 82571EB/82571GB Gigabit Ethernet Controller D0/D1

# eth0 -> On board, connexion internet
# eth2 -> connexion force sensor
# eth1 -> connexion switch

# Table de routage actuelle
# route -n

# Associate ethX
# eth2 -> force sensor
# eth1 -> switch
ethX_associate_switch="eth1"
ethX_associate_force_sensor="eth2"

# Define IP address for ethX associate to switch and ethX associate to pan tilt
ip_address_associate_switch="192.168.100.103"
ip_address_associate_sub_net_switch="192.168.100.0/24"

ip_address_associate_force_sensor="192.168.1.100"


# Define IP address for force sensor
ip_address_force_sensor="192.168.1.1"

# Define Netmask for switch card and pan tilt card
netmask_associate_switch="255.255.255.0"
netmask_associate_force_sensor="255.255.255.255"

# Define hw of each ethX associated
hw_ethX_associate_switch="68:05:ca:1c:d7:c8"
hw_ethX_associate_force_sensor="68:05:ca:1c:d7:c9"

do_start()
{
	echo "Start net config for switch and force sensor !"
	
	echo "ifconfig down ethX associated to switch !"
	sudo ifconfig $ethX_associate_switch down
	
	echo "ifconfig down ethX associated to force sensor  !"
	sudo ifconfig $ethX_associate_force_sensor down
	
	echo "Start ethX associate to switch !"
	sudo ifconfig $ethX_associate_switch up $ip_address_associate_switch netmask $netmask_associate_switch hw ether $hw_ethX_associate_switch
        
	echo "Start ethX associate to force sensor !"
	sudo ifconfig $ethX_associate_force_sensor up $ip_address_associate_force_sensor netmask $netmask_associate_force_sensor hw ether $hw_ethX_associate_force_sensor
	
	echo "Delete all routes of ethX associate to switch and force sensor !"
	sleep 5
	sudo route del -net $ip_address_associate_sub_net_switch dev $ethX_associate_force_sensor
	sudo route del -net $ip_address_associate_sub_net_switch dev $ethX_associate_switch

	echo "Add route to host force sensor (using ip address) !"
	sleep 5
	sudo route add -host $ip_address_force_sensor dev $ethX_associate_force_sensor

	echo "Add sub network for switch !"
	sleep 5
	sudo route add -net $ip_address_associate_sub_net_switch dev $ethX_associate_switch
}


case "$1" in
   start)
      do_start
      ;;
   stop)
      do_stop
      ;;
   *)
      echo "--> Usage: $0 {start|stop}"
      exit 1
esac

exit 0
