#!/bin/bash
# start_mc_sdk.sh

source /opt/ros/humble/setup.bash
source ~/mc_sdk_node-main/install/setup.bash

ros2 launch mc_sdk_node mc_sdk_node.launch.py
