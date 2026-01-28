#!/bin/bash
# Quick debug script to find the map topic

echo "======================================"
echo "Searching for OccupancyGrid topics..."
echo "======================================"
echo ""

# List all topics
echo "All available topics:"
ros2 topic list | grep -i map || echo "  No topics with 'map' in the name found"
echo ""

# Look for OccupancyGrid type
echo "Topics publishing OccupancyGrid:"
ros2 topic list -t | grep "nav_msgs/msg/OccupancyGrid" || echo "  No OccupancyGrid topics found"
echo ""

# Check if the expected topic exists
echo "Checking expected topic /task_generator_node/map:"
ros2 topic info /task_generator_node/map 2>/dev/null && echo "  ✓ Topic exists!" || echo "  ✗ Topic NOT found"
echo ""

# Check for alternative map topics
echo "Common map topic names:"
for topic in "/map" "/task_generator_node/map" "/global_costmap/costmap" "/occupancy_grid"; do
    if ros2 topic info $topic &>/dev/null; then
        echo "  ✓ $topic exists"
        ros2 topic info $topic | head -2
    fi
done
echo ""

echo "======================================"
echo "Is Gazebo/simulation running?"
ros2 topic list | grep -q "/gazebo" && echo "  ✓ Gazebo topics detected" || echo "  ✗ No Gazebo topics found - simulation may not be running"
echo "======================================"
