#!/bin/bash
# Test script to view the new VLM prompt format

source /opt/ros/humble/setup.bash
source /home/huojiaxi/3D_detector/3D_bounding_box_gb_visual_detection/install/setup.bash
source install/setup.bash

echo "================================================"
echo "Testing New VLM Prompt Format"
echo "================================================"
echo ""
echo "Starting VLM integration node..."
echo ""

# Run node in background
ros2 run social_mpc_nav vlm_integration_node \
  --ros-args --params-file social_mpc_nav/config/vlm_integration.yaml \
  > /tmp/vlm_node.log 2>&1 &
NODE_PID=$!

# Wait for node to start and process messages
sleep 5

# Echo the most recent prompt
echo "=== Generated VLM Prompt ==="
echo ""
timeout 3 ros2 topic echo /vlm/prompt --once 2>/dev/null | head -200

echo ""
echo "=== Full log available at /tmp/vlm_node.log ==="

# Kill the node
kill $NODE_PID 2>/dev/null
wait $NODE_PID 2>/dev/null

echo ""
echo "Done!"
