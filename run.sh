#!/bin/bash

# Start MicroXRCEAgent
gnome-terminal --tab -- bash -c "MicroXRCEAgent udp4 -p 8888; exec bash"
sleep 1

# Start PX4 SITL
gnome-terminal --tab -- bash -c "cd ~/Dev/PX4-Autopilot && make px4_sitl gazebo-classic; exec bash"
sleep 1

# Start ROS2 offboard control
gnome-terminal --tab -- bash -c "cd ~/Dev/DD_Nav_WS/dd_gazebo_ws/ && source install/setup.bash && ros2 launch px4_offboard offboard_velocity_control.launch.py; exec bash"
sleep 1

# Start ROS2 navigation
gnome-terminal --tab -- bash -c "cd ~/Dev/DD_Nav_WS/dd_gazebo_ws/ && source install/setup.bash && ros2 launch drone_nav navigation.launch.py; exec bash"
sleep 1

# Start QGroundControl
gnome-terminal --tab -- bash -c "cd ~/Dev && ./QGroundControl.AppImage; exec bash"
sleep 1

# Run px4_handler
gnome-terminal --tab -- bash -c "cd ~/Dev/DD_Nav_WS/dd_gazebo_ws/ && source install/setup.bash && ros2 run px4_handler offboard_control; exec bash"
