#!/bin/bash
cd ~/ga_ws
source /opt/ros/foxy/setup.bash 
source ./install/local_setup.bash

# Start USB camera node
echo "------------------------------"
echo "Starting CSI camera node..."
ros2 run image_pub mono_image_pub &
sleep 5
echo "CSI camera node started."


# Start Apriltag detection & Autolanding
echo "------------------------------"
echo "Starting Apriltag detection & Autolanding..."
ros2 launch goal_pub one_test.launch.py &
sleep 5
echo "Apriltag detection & Autolanding started."

# Keep the script running to maintain background processes
wait

