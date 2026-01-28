#!/usr/bin/env python3
"""
Script to capture the COMPLETE VLM prompt without truncation.
ROS2 topic echo truncates long strings, this script gets the full content.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import sys

class PromptCapturer(Node):
    def __init__(self):
        super().__init__('prompt_capturer')
        self.subscription = self.create_subscription(
            String,
            '/vlm/prompt',
            self.prompt_callback,
            10)
        self.received = False
        self.get_logger().info('Waiting for VLM prompt...')

    def prompt_callback(self, msg):
        if not self.received:
            self.received = True
            print("\n" + "="*80)
            print("COMPLETE VLM PROMPT:")
            print("="*80 + "\n")
            print(msg.data)
            print("\n" + "="*80)

            # Save to file
            with open('vlm_prompt_complete.txt', 'w') as f:
                f.write(msg.data)

            print(f"\n✓ Saved to: vlm_prompt_complete.txt")
            print(f"✓ Prompt length: {len(msg.data)} characters")
            print(f"✓ Lines: {len(msg.data.splitlines())}")
            print("\n" + "="*80 + "\n")

            # Exit after receiving one message
            self.get_logger().info('Prompt captured successfully. Exiting...')
            sys.exit(0)

def main(args=None):
    rclpy.init(args=args)
    node = PromptCapturer()

    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
