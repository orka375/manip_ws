#!/bin/bash

# Exit immediately if any command fails
# set -e

echo "==============================================="
echo " Welcome! Starting your Youbot Launch Sequence"
echo "==============================================="
echo " "
echo "Version 1.0"
echo "Developped by"
echo "           _"
echo "          / /    "
echo "         / / _      "
echo "        / / / \       "
echo "       / / / _ \      "          
echo "      / /_/ /_\ \         "                   
echo "     /__   ______\      "
echo "        / /________     "
echo "       /___________\   "       
echo "                      "


# Source your ROS 2 distro (e.g., humble, iron, etc.)
source /opt/ros/rolling/setup.bash

# Source your ROS 2 workspace
source /home/fabian/youbot_ws/install/setup.bash
source /home/fabian/ros2_opcua/install/setup.bash

python3 /home/fabian/youbot_ws/src/lcdtest.py 

# Run the launch file
ros2 launch youbot launch_youbot.launch.py