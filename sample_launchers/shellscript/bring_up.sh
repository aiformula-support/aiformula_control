#!/bin/bash
set -e

SCRIPT_DIR=$(cd $(dirname $0); pwd)

# CAN
sudo ip link set can0 down
bash ${SCRIPT_DIR}/init_sensors.sh

# IMU
sudo chmod 777 /dev/ttyUSB0 

#COMMAND
ros2 launch sample_launchers all_nodes.launch.py
