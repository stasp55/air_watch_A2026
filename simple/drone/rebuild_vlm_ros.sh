#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/humble/setup.bash
cd /home/sverk/sverk_ws
colcon build --packages-select vlm_interfaces vlm_perception
