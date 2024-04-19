#!/bin/bash

#enable can
sudo ip link set can0 up type can bitrate 1000000
sudo ifconfig can0 txqueuelen 5000

#activate
source /opt/ros/galactic/setup.bash

#source /home/mower/Autonomus-Lawnmower/hrp-p2z-open-dist/x86_64/local_setup.bash
source /home/mower/Autonomus-Lawnmower/hrp-p2z-open-dist/aarch64/local_setup.bash


python3 /home/mower/Autonomus-Lawnmower/src/drive_forward_v2.py