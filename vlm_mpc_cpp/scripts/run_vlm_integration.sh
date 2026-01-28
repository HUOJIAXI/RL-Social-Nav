#!/bin/bash
# Quick launch script for VLM Integration Node

# Source all required environments
source /opt/ros/humble/setup.bash
source /home/huojiaxi/3D_detector/3D_bounding_box_gb_visual_detection/install/setup.bash
source /home/huojiaxi/phd_upm/vlm_mpc_cpp/install/setup.bash

echo "================================================"
echo "Starting VLM Integration Node"
echo "================================================"
echo ""
echo "Subscribed topics:"
echo "  - /darknet_ros_3d/bounding_boxes"
echo "  - /person_tracker/person_info"
echo "  - /task_generator_node/tiago_official/odom"
echo ""
echo "Published topics:"
echo "  - /vlm/prompt"
echo "  - /vlm/response"
echo ""
echo "================================================"
echo ""

# Run the node
ros2 run social_mpc_nav vlm_integration_node \
  --ros-args --params-file /home/huojiaxi/phd_upm/vlm_mpc_cpp/social_mpc_nav/config/vlm_integration.yaml
