#!/bin/bash
# Script to view the complete VLM prompt

source /opt/ros/humble/setup.bash
source /home/huojiaxi/3D_detector/3D_bounding_box_gb_visual_detection/install/setup.bash
source install/setup.bash

echo "================================================"
echo "VLM Prompt Viewer"
echo "================================================"
echo ""

# Check if node is running
if ! ros2 node list | grep -q vlm_integration_node; then
    echo "⚠️  vlm_integration_node is not running!"
    echo ""
    echo "Please start it first with:"
    echo "  ./run_vlm_integration.sh"
    echo ""
    echo "Or start it in another terminal, then run this script again."
    exit 1
fi

echo "✓ Node is running"
echo ""
echo "Waiting for next prompt message..."
echo ""
echo "================================================"
echo ""

# Echo the complete prompt (remove limits)
ros2 topic echo /vlm/prompt --once --no-arr

echo ""
echo "================================================"
echo ""
echo "To save to file, run:"
echo "  ros2 topic echo /vlm/prompt --once --no-arr > prompt.txt"
echo ""
