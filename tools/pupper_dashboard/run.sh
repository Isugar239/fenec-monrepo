#!/bin/bash
source /opt/ros/jazzy/setup.bash
source /home/pi/pupperv3-monorepo/ros2_ws/install/local_setup.bash
exec python3 /home/pi/pupperv3-monorepo/tools/pupper_dashboard/dashboard.py
