#!/usr/bin/env bash
# Starts the local VLM ROS action server.  It sends no flight commands.
# ROS Humble setup scripts read optional unset variables, so do not use `-u`.
set -eo pipefail
source /opt/ros/humble/setup.bash
source /home/sverk/sverk_ws/install/setup.bash
# cv_bridge in ROS Humble was built against system NumPy 1.x.  The container
# also has NumPy 2.x in /usr/local, so make the compatible ABI win.
export PYTHONPATH=/usr/lib/python3/dist-packages${PYTHONPATH:+:$PYTHONPATH}
exec ros2 launch vlm_perception vlm.launch.py \
  model:=/home/sverk/vlm_models/SmolVLM-256M-Instruct-Q4_K_M.gguf \
  mmproj:=/home/sverk/vlm_models/mmproj-SmolVLM-256M-Instruct-f16.gguf \
  llama_bin:=/home/sverk/llama.cpp/build/bin/llama-cli \
  max_tok:=96
