#!/bin/bash
# Complete script to capture the FULL VLM prompt without truncation

source /opt/ros/humble/setup.bash
source /home/huojiaxi/3D_detector/3D_bounding_box_gb_visual_detection/install/setup.bash
source install/setup.bash

echo "================================================"
echo "FULL VLM Prompt Capture (No Truncation)"
echo "================================================"
echo ""

# Check if node is running
if ! ros2 node list 2>/dev/null | grep -q vlm_integration_node; then
    echo "⚠️  vlm_integration_node is NOT running!"
    echo ""
    echo "Starting it now..."
    ros2 run social_mpc_nav vlm_integration_node \
      --ros-args --params-file social_mpc_nav/config/vlm_integration.yaml \
      > /dev/null 2>&1 &
    NODE_PID=$!
    echo "Waiting for node to start..."
    sleep 5
    STARTED_NODE=true
else
    echo "✓ Node is already running"
    STARTED_NODE=false
fi

echo ""
echo "Capturing complete prompt..."
echo ""

# Run Python script to capture full prompt
python3 capture_full_prompt.py

# Show the captured file
if [ -f vlm_prompt_complete.txt ]; then
    echo ""
    echo "View complete prompt with:"
    echo "  cat vlm_prompt_complete.txt"
    echo "  less vlm_prompt_complete.txt"
    echo "  nano vlm_prompt_complete.txt"
    echo ""

    # Ask if user wants to view now
    read -p "Display prompt now? (y/n): " -n 1 -r
    echo
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        echo ""
        cat vlm_prompt_complete.txt
    fi
fi

# Cleanup if we started the node
if [ "$STARTED_NODE" = true ]; then
    echo ""
    echo "Stopping node..."
    kill $NODE_PID 2>/dev/null
fi

echo ""
echo "Done!"
