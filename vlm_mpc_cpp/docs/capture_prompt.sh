#!/bin/bash
# Complete script to capture and display VLM prompt

source /opt/ros/humble/setup.bash
source /home/huojiaxi/3D_detector/3D_bounding_box_gb_visual_detection/install/setup.bash
source install/setup.bash

echo "================================================"
echo "Complete VLM Prompt Capture"
echo "================================================"
echo ""

# Start node in background
echo "Starting vlm_integration_node..."
ros2 run social_mpc_nav vlm_integration_node \
  --ros-args --params-file social_mpc_nav/config/vlm_integration.yaml \
  > /dev/null 2>&1 &
NODE_PID=$!

echo "Waiting for node to initialize and process data..."
sleep 6

echo ""
echo "================================================"
echo "CAPTURED VLM PROMPT:"
echo "================================================"
echo ""

# Capture prompt to file
ros2 topic echo /vlm/prompt --once --no-arr 2>/dev/null | tee vlm_prompt_latest.txt

echo ""
echo "================================================"
echo ""
echo "✓ Prompt saved to: vlm_prompt_latest.txt"
echo ""

# Cleanup
kill $NODE_PID 2>/dev/null
wait $NODE_PID 2>/dev/null

echo "To view again: cat vlm_prompt_latest.txt"
echo "To view with pager: less vlm_prompt_latest.txt"
