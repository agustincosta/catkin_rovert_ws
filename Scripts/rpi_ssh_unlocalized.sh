#!/bin/bash
HOST_IP=$(ifconfig | grep -v lo | grep 172.16 | grep inet | awk '{print $2}')
HOST_IP=${HOST_IP##*:}  #Retiene solo la parte luego de los ":"
echo $HOST_IP
USERNAME=ubuntu
PASSWORD='ubuntu'
HOST=ubiquityrobot
SCRIPT="source catkin_ws/devel/setup.bash ; export ROS_MASTER_URI=http://${HOST_IP}:11311; roslaunch rover_2dnav rpi_unlocalized_config.launch"

trap ctrl_c INT

function ctrl_c()
{
	rosnode kill -a
	exit
}

roscore & sshpass -p ${PASSWORD} ssh ${USERNAME}@${HOST} "${SCRIPT}"



