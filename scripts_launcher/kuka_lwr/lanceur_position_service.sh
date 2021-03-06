#!/bin/bash

# Laurent LEQUIEVRE
# laurent.lequievre@uca.fr
# UMR 6602 - Institut Pascal

do_start()
{
	cd /home/kuka/git_project/platform-kuka-ip-real
	source devel/setup.bash
	if [ $1 = "left" ]
	then
		namespace="kuka_lwr_left"
	else
		namespace="kuka_lwr_right"
	fi

	rosservice call /$namespace/controller_manager/switch_controller "{start_controllers: ['joint_position_controller'], stop_controllers: [], strictness: 2}"
}

do_stop()
{
	cd /home/kuka/git_project/platform-kuka-ip-real
	source devel/setup.bash
	if [ $1 = "left" ]
	then
		namespace="kuka_lwr_left"
	else
		namespace="kuka_lwr_right"
	fi
	rosservice call /$namespace/controller_manager/switch_controller "{start_controllers: [], stop_controllers: ['joint_position_controller'], strictness: 1}"
}

case "$1" in
   start)
      do_start $2
      ;;
   stop)
      do_stop $2
      ;;
   *)
      echo "--> Usage: $0 {start|stop} {left|right}"
      exit 1
esac

exit 0
