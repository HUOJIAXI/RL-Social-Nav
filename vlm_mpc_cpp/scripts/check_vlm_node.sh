#!/bin/bash
# Script to check VLM integration node status

echo "=========================================="
echo "VLM Integration Node Diagnostic"
echo "=========================================="
echo ""

# Check if workspace is sourced
if [ -z "$ROS_DISTRO" ]; then
    echo "⚠️  ROS2 not sourced. Please run:"
    echo "   source /opt/ros/humble/setup.bash"
    echo "   source ~/phd_upm/vlm_mpc_cpp/install/setup.bash"
    exit 1
fi

echo "1. Checking if node is running..."
NODE_COUNT=$(ros2 node list 2>/dev/null | grep -c vlm_integration_node || echo "0")
if [ "$NODE_COUNT" -eq "0" ]; then
    echo "   ❌ Node is NOT running"
    echo "   Please start it with:"
    echo "   ros2 launch social_mpc_nav vlm_integration.launch.py"
else
    echo "   ✅ Node is running"
    ros2 node list | grep vlm_integration_node
fi
echo ""

echo "2. Checking topics..."
PROMPT_TOPIC=$(ros2 topic list 2>/dev/null | grep -c "/vlm/prompt" || echo "0")
if [ "$PROMPT_TOPIC" -eq "0" ]; then
    echo "   ❌ /vlm/prompt topic NOT found"
    echo "   This might be normal if node just started and hasn't published yet"
else
    echo "   ✅ /vlm/prompt topic exists"
    echo "   Topic info:"
    ros2 topic info /vlm/prompt 2>/dev/null | head -5
fi
echo ""

echo "3. Checking bounding boxes topic..."
BBOX_TOPIC=$(ros2 topic list 2>/dev/null | grep -c "/darknet_ros_3d/bounding_boxes" || echo "0")
if [ "$BBOX_TOPIC" -eq "0" ]; then
    echo "   ❌ /darknet_ros_3d/bounding_boxes topic NOT found"
    echo "   This is the input topic - it must exist for the node to work"
else
    echo "   ✅ /darknet_ros_3d/bounding_boxes topic exists"
    echo "   Topic info:"
    ros2 topic info /darknet_ros_3d/bounding_boxes 2>/dev/null | head -5
    echo "   Checking if data is being published..."
    timeout 2 ros2 topic echo /darknet_ros_3d/bounding_boxes --once 2>/dev/null > /dev/null
    if [ $? -eq 0 ]; then
        echo "   ✅ Data is being published"
    else
        echo "   ⚠️  No data received (might be normal if publisher is slow)"
    fi
fi
echo ""

echo "4. Checking odometry topic..."
ODOM_TOPIC=$(ros2 topic list 2>/dev/null | grep -c "odom" || echo "0")
if [ "$ODOM_TOPIC" -eq "0" ]; then
    echo "   ⚠️  No odometry topic found (node will still work but without robot position context)"
else
    echo "   ✅ Odometry topic(s) found:"
    ros2 topic list | grep odom
fi
echo ""

echo "5. Recent log messages from vlm_integration_node..."
if [ "$NODE_COUNT" -gt "0" ]; then
    echo "   (Check terminal where node is running for detailed logs)"
    echo "   Or run: ros2 topic echo /rosout | grep vlm_integration"
else
    echo "   Node not running - no logs available"
fi
echo ""

echo "=========================================="
echo "Diagnostic complete"
echo "=========================================="
