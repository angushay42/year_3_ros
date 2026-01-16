#! /usr/bin/bash
colcon build
source install/setup.bash
ros2 launch KV6022_assessment demo.launch.py
