#!/bin/bash
set -e

# Set env vars
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE=/ros2/super_client.xml

# Source ROS setup
source /ros2/install/setup.bash

# Reset ros daemon
ros2 daemon stop
ros2 daemon start

# Give 2 seconds for daemon to setup
sleep 2

# Start discovery server in the background
fastdds discovery -i 0 -l 100.75.18.108 -p 11811 &
DISCOVERY_PID=$!

# Give 2 seconds for server to setup
sleep 2

# Start ROS launch
ros2 launch firo_bringup firo_bringup.launch &
LAUNCH_PID=$!

# Wait for both to keep container alive
wait $DISCOVERY_PID
wait $LAUNCH_PID