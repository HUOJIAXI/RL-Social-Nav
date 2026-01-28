#!/usr/bin/env python3
"""
Verify that robot odometry and people positions are in the same coordinate frame.
This script subscribes to both topics and displays them side-by-side.
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from people_msgs.msg import People
import sys

class CoordinateVerifier(Node):
    def __init__(self):
        super().__init__('coordinate_verifier')

        self.robot_pos = None
        self.robot_frame = None
        self.people_data = None
        self.people_frame = None

        # Subscribe to robot odometry
        self.odom_sub = self.create_subscription(
            Odometry,
            '/task_generator_node/tiago_official/odom',
            self.odom_callback,
            10)

        # Subscribe to people data
        self.people_sub = self.create_subscription(
            People,
            '/task_generator_node/people',
            self.people_callback,
            10)

        # Timer to print status
        self.timer = self.create_timer(2.0, self.print_status)

        self.get_logger().info('Coordinate Verifier started')
        self.get_logger().info('Checking coordinate frames for robot and people...')

    def odom_callback(self, msg):
        self.robot_pos = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        self.robot_frame = msg.header.frame_id

    def people_callback(self, msg):
        self.people_frame = msg.header.frame_id
        self.people_data = [(p.name, p.position.x, p.position.y) for p in msg.people]

    def print_status(self):
        print("\n" + "="*80)
        print("COORDINATE FRAME VERIFICATION")
        print("="*80)

        if self.robot_pos and self.robot_frame:
            print(f"\n🤖 ROBOT ODOMETRY:")
            print(f"   Frame: '{self.robot_frame}'")
            print(f"   Position: x={self.robot_pos[0]:.3f}, y={self.robot_pos[1]:.3f}")
        else:
            print(f"\n🤖 ROBOT ODOMETRY: Waiting for data...")

        if self.people_data and self.people_frame:
            print(f"\n🚶 PEOPLE DATA:")
            print(f"   Frame: '{self.people_frame}'")
            print(f"   Number of people: {len(self.people_data)}")
            for name, x, y in self.people_data[:3]:  # Show first 3 people
                print(f"   - {name}: x={x:.3f}, y={y:.3f}")
            if len(self.people_data) > 3:
                print(f"   ... and {len(self.people_data) - 3} more")
        else:
            print(f"\n🚶 PEOPLE DATA: Waiting for data...")

        # Check if frames match
        if self.robot_frame and self.people_frame:
            print(f"\n📍 FRAME COMPARISON:")
            if self.robot_frame == self.people_frame:
                print(f"   ✅ FRAMES MATCH: Both in '{self.robot_frame}' frame")
                print(f"   ✅ Ground-truth comparison is VALID")
            else:
                print(f"   ❌ FRAME MISMATCH!")
                print(f"   ❌ Robot: '{self.robot_frame}' vs People: '{self.people_frame}'")
                print(f"   ❌ Direct position comparison NOT valid!")

        # Calculate distances if we have both
        if self.robot_pos and self.people_data:
            print(f"\n📏 DISTANCES FROM ROBOT:")
            for name, px, py in self.people_data[:3]:
                dist = ((px - self.robot_pos[0])**2 + (py - self.robot_pos[1])**2)**0.5
                print(f"   - {name}: {dist:.2f}m")

        print("="*80)

def main(args=None):
    rclpy.init(args=args)
    verifier = CoordinateVerifier()

    try:
        rclpy.spin(verifier)
    except KeyboardInterrupt:
        print("\nShutting down...")
    finally:
        verifier.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
