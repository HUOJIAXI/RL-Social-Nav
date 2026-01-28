#!/bin/bash
# Test script to verify VLM topic is published

echo "Testing VLM topic publication..."
echo ""

# Source workspace
if [ -f ~/phd_upm/vlm_mpc_cpp/install/setup.bash ]; then
    source ~/phd_upm/vlm_mpc_cpp/install/setup.bash
    echo "✅ Workspace sourced"
else
    echo "❌ Workspace not found. Please build first:"
    echo "   cd ~/phd_upm/vlm_mpc_cpp && colcon build"
    exit 1
fi

echo ""
echo "1. Checking if node is running..."
NODE_RUNNING=$(ros2 node list 2>/dev/null | grep -c vlm_integration_node || echo "0")
if [ "$NODE_RUNNING" -eq "0" ]; then
    echo "   ⚠️  Node not running. Starting node in background..."
    ros2 launch social_mpc_nav vlm_integration.launch.py > /tmp/vlm_node.log 2>&1 &
    NODE_PID=$!
    echo "   Node started with PID: $NODE_PID"
    echo "   Waiting 2 seconds for node to initialize..."
    sleep 2
else
    echo "   ✅ Node is already running"
fi

echo ""
echo "2. Checking for /vlm/prompt topic..."
for i in {1..5}; do
    TOPIC_EXISTS=$(ros2 topic list 2>/dev/null | grep -c "/vlm/prompt" || echo "0")
    if [ "$TOPIC_EXISTS" -gt "0" ]; then
        echo "   ✅ Topic found!"
        break
    else
        echo "   Attempt $i/5: Topic not found yet, waiting..."
        sleep 1
    fi
done

if [ "$TOPIC_EXISTS" -eq "0" ]; then
    echo "   ❌ Topic still not found after 5 attempts"
    echo ""
    echo "3. Checking node logs..."
    if [ -f /tmp/vlm_node.log ]; then
        echo "   Last 20 lines of node log:"
        tail -20 /tmp/vlm_node.log
    fi
    echo ""
    echo "4. Checking if publisher count > 0..."
    ros2 topic info /vlm/prompt 2>&1
    exit 1
fi

echo ""
echo "3. Testing topic echo..."
echo "   Waiting for message (timeout 3 seconds)..."
timeout 3 ros2 topic echo /vlm/prompt --once 2>&1 | head -10

echo ""
echo "4. Topic info:"
ros2 topic info /vlm/prompt 2>&1

echo ""
echo "✅ Test complete! Topic is working."
