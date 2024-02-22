#!/bin/bash

source /opt/ros/galactic/setup.bash

# x86_x64
#source /home/mower/Autonomus-Lawnmower/hrp-p2z-open-dist/x86_64/local_setup.bash
#python3 /home/mower/Autonomus-Lawnmower/hrp-p2z-open-dist/x86_64/lib/python3.8/site-packages/hqv_keyboard_remote_drive/keyboard_remote_drive_node.py

# aarch64
source /home/mower/Autonomus-Lawnmower/hrp-p2z-open-dist/aarch64/local_setup.bash
python3 /home/mower/Autonomus-Lawnmower/hrp-p2z-open-dist/aarch64/lib/python3.8/site-packages/hqv_power_shutdown_linux/main.py 
