#!/bin/bash
source /opt/ros/humble/setup.bash
source /home/huojiaxi/3D_detector/3D_bounding_box_gb_visual_detection/install/setup.bash
source install/setup.bash

# Run node in background
ros2 run social_mpc_nav vlm_integration_node --ros-args --params-file social_mpc_nav/config/vlm_integration.yaml > /dev/null 2>&1 &
NODE_PID=$!

# Wait for node to start
sleep 2

# Echo the prompt topic once
echo "=== Checking /vlm/prompt topic ==="
timeout 5 ros2 topic echo /vlm/prompt --once 2>/dev/null | head -100

# Kill the node
kill $NODE_PID 2>/dev/null
wait $NODE_PID 2>/dev/null

echo ""
echo "=== Done ==="
