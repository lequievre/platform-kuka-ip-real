#!/bin/sh
# Laurent LEQUIEVRE
# laurent.lequievre@univ-bpclermont.fr
# UMR 6602 - Institut Pascal

# Ip address of Pan Tilt -> 192.168.100.252

# NETWORK INFORMATIONS
# ====================
# sudo lshw -C network

# eth0 -> 10:fe:ed:08:18:dc -> RTL8111/8168/8411 PCI Express Gigabit Ethernet Controller -> pci@0000:05:00.0
# eth1 -> 00:0a:f7:93:2f:fa -> NetXtreme BCM5722 Gigabit Ethernet PCI Express -> pci@0000:04:00.0
# eth2 -> 48:4d:7e:ac:d2:ae -> Ethernet Connection (2) I219-LM -> pci@0000:00:1f.6

#  ???? eth3 -> ethernet USB3 adapter

# eth0 -> connexion shadow hand
# eth1 -> connexion switch
# eth2 -> On board connexion internet

# Table de routage actuelle
# route -n


ethX_associate_switch="eth1"

# Define IP address for ethX associate to switch

ip_address_associate_switch="192.168.100.113"
ip_address_associate_sub_net_switch="192.168.100.0/24"


# Define Netmask for switch card

netmask_associate_switch="255.255.255.0"
netmask_associate_user_computer="255.255.255.0"
netmask_associate_fsensor="255.255.255.255"

# Define hw of each ethX associated

hw_ethX_associate_switch="00:0a:f7:93:2f:fa"


do_start()
{
	
	echo "Start net config for switch !"

	echo "ifconfig down ethX associated to switch !"
	sudo ifconfig $ethX_associate_switch down

	sleep 3

	echo "Start ethX associate to switch !"
	sudo ifconfig $ethX_associate_switch up $ip_address_associate_switch netmask $netmask_associate_switch hw ether $hw_ethX_associate_switch


	echo "Delete all route of switch !"
	sleep 5
	sudo route del -net $ip_address_associate_sub_net_switch dev $ethX_associate_switch


	echo "Add sub network for switch"
	sleep 5
	sudo route add -net $ip_address_associate_sub_net_switch dev $ethX_associate_switch
	
}

do_stop()
{
	echo "ifconfig down ethX associated to switch !"
	sudo ifconfig $ethX_associate_switch down

	sleep 3

	echo "Delete all routes of ethX associate to switch !"
	sleep 5
	sudo route del -net $ip_address_associate_sub_net_switch dev $ethX_associate_switch

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

