#!/bin/bash

# kills active nodes
killall main
killall hrp

#enable can UNCOMMENT
#print("ENABLING CAN \n")
sudo ip link set can0 up type can bitrate 1000000
sudo ifconfig can0 txqueuelen 5000

#activate
source /opt/ros/galactic/setup.bash

# aarch64
source /home/mower/Autonomus-Lawnmower/hrp-p2z-open-dist/aarch64/local_setup.bash

# x86_x64
#source /home/mower/Autonomus-Lawnmower/hrp-p2z-open-dist/x86_64/local_setup.bash


#the keepalive launch file will make the mower stay awak, it is good when developing, but will drain battery if mower stops in garden.
ros2 launch hrp_pkg hrp_keepalive.launch.py
#ros2 launch hrp_pkg hrp.launch.py
