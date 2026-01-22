#! /usr/bin/bash
ros2 service call /reset_simulation std_srvs/srv/Empty
colcon build
source install/setup.bash
ros2 launch KV6022_assessment demo.launch.py
