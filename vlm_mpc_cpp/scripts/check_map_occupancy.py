#!/usr/bin/env python3
"""
Diagnostic tool to visualize the occupancy grid map and verify obstacle data.
This helps debug why the global planner might not be avoiding obstacles.
"""

import sys
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from nav_msgs.msg import OccupancyGrid
import numpy as np


class MapDiagnostic(Node):
    def __init__(self, map_topic=None):
        super().__init__('map_diagnostic')

        # Auto-detect map topic if not specified
        if map_topic is None:
            map_topic = self.detect_map_topic()

        # Use transient_local QoS to match map publishers
        # This is required for topics that publish once and store the message
        qos_profile = QoSProfile(
            depth=10,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE
        )

        self.subscription = self.create_subscription(
            OccupancyGrid,
            map_topic,
            self.map_callback,
            qos_profile
        )
        self.get_logger().info(f'Waiting for map on {map_topic}...')
        self.get_logger().info('Using QoS: transient_local durability (for late-joining subscribers)')

    def detect_map_topic(self):
        """Try to find an OccupancyGrid topic automatically"""
        # Common map topic names in order of preference
        common_topics = [
            '/task_generator_node/map',
            '/map',
            '/global_costmap/costmap',
            '/occupancy_grid',
        ]

        # Get list of available topics
        topic_list = self.get_topic_names_and_types()

        # Check each common topic
        for candidate in common_topics:
            for topic_name, topic_types in topic_list:
                if topic_name == candidate and 'nav_msgs/msg/OccupancyGrid' in topic_types:
                    self.get_logger().info(f'Auto-detected map topic: {candidate}')
                    return candidate

        # If no match found, check for any OccupancyGrid topic
        for topic_name, topic_types in topic_list:
            if 'nav_msgs/msg/OccupancyGrid' in topic_types:
                self.get_logger().warn(f'Using first available OccupancyGrid topic: {topic_name}')
                return topic_name

        # Default fallback
        self.get_logger().error('No OccupancyGrid topics found! Using default /task_generator_node/map')
        self.get_logger().error('Make sure your simulation/map publisher is running!')
        return '/task_generator_node/map'

    def map_callback(self, msg):
        """Analyze and display map statistics"""
        self.get_logger().info('=' * 60)
        self.get_logger().info('MAP DIAGNOSTIC REPORT')
        self.get_logger().info('=' * 60)

        # Basic info
        self.get_logger().info(f'Map size: {msg.info.width}x{msg.info.height} cells')
        self.get_logger().info(f'Resolution: {msg.info.resolution:.3f} m/cell')
        self.get_logger().info(f'Origin: ({msg.info.origin.position.x:.2f}, {msg.info.origin.position.y:.2f})')

        # Convert to numpy array for analysis
        data = np.array(msg.data, dtype=np.int8)

        # Count cell types
        unknown = np.sum(data < 0)
        free = np.sum((data >= 0) & (data < 50))
        occupied = np.sum(data >= 50)
        total = len(data)

        self.get_logger().info('')
        self.get_logger().info('Cell occupancy distribution:')
        self.get_logger().info(f'  Unknown  (< 0):    {unknown:7d} cells ({100*unknown/total:5.1f}%)')
        self.get_logger().info(f'  Free     (0-49):   {free:7d} cells ({100*free/total:5.1f}%)')
        self.get_logger().info(f'  Occupied (50-100): {occupied:7d} cells ({100*occupied/total:5.1f}%)')
        self.get_logger().info(f'  Total:             {total:7d} cells')

        # Occupancy value statistics
        occupied_vals = data[data >= 50]
        if len(occupied_vals) > 0:
            self.get_logger().info('')
            self.get_logger().info('Occupied cell values:')
            self.get_logger().info(f'  Min: {occupied_vals.min()}')
            self.get_logger().info(f'  Max: {occupied_vals.max()}')
            self.get_logger().info(f'  Mean: {occupied_vals.mean():.1f}')

        # Sample some occupied cells
        if occupied > 0:
            occupied_indices = np.where(data >= 50)[0][:10]  # First 10
            self.get_logger().info('')
            self.get_logger().info('Sample occupied cells (index -> value):')
            for idx in occupied_indices:
                y = idx // msg.info.width
                x = idx % msg.info.width
                world_x = msg.info.origin.position.x + (x + 0.5) * msg.info.resolution
                world_y = msg.info.origin.position.y + (y + 0.5) * msg.info.resolution
                self.get_logger().info(
                    f'  Grid({x:3d},{y:3d}) = {data[idx]:3d} '
                    f'-> World({world_x:6.2f}, {world_y:6.2f})'
                )

        self.get_logger().info('=' * 60)
        self.get_logger().info('RECOMMENDATIONS:')

        if occupied == 0:
            self.get_logger().warn('⚠ NO OCCUPIED CELLS FOUND!')
            self.get_logger().warn('  The map appears completely free. Check:')
            self.get_logger().warn('  1. Is the correct map being published?')
            self.get_logger().warn('  2. Are obstacles present in the Gazebo world?')
        elif unknown > total * 0.8:
            self.get_logger().warn('⚠ Map is mostly unknown!')
            self.get_logger().warn('  Consider setting treat_unknown_as_free: false')
        elif free < total * 0.1:
            self.get_logger().warn('⚠ Very little free space!')
            self.get_logger().warn('  The map might be too cluttered or incorrectly configured')
        else:
            self.get_logger().info('✓ Map looks reasonable for A* planning')

        self.get_logger().info('=' * 60)

        # Shutdown after one message
        rclpy.shutdown()


def main(args=None):
    """Main function with optional command-line map topic specification"""
    rclpy.init(args=args)

    # Check for command-line argument
    map_topic = None
    if len(sys.argv) > 1:
        map_topic = sys.argv[1]
        print(f"Using specified map topic: {map_topic}")

    node = MapDiagnostic(map_topic=map_topic)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()


if __name__ == '__main__':
    if '--help' in sys.argv or '-h' in sys.argv:
        print("Usage: python3 check_map_occupancy.py [map_topic]")
        print("")
        print("Arguments:")
        print("  map_topic    Optional: Specific map topic to subscribe to")
        print("               If not provided, will auto-detect from available topics")
        print("")
        print("Examples:")
        print("  python3 check_map_occupancy.py")
        print("  python3 check_map_occupancy.py /task_generator_node/map")
        print("  python3 check_map_occupancy.py /map")
        sys.exit(0)

    main()
