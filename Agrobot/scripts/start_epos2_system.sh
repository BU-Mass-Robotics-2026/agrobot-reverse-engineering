#!/usr/bin/env bash
set -e

cd ~/agrobot_ws
source /opt/ros/jazzy/setup.bash
source ~/agrobot_ws/install/setup.bash

ros2 launch epos2_bridge epos2_system.launch.py apply_live_remap:=true
