#!/usr/bin/env python3
"""
Quick test to verify TF transform between tiago_official/odom and map works.
"""

import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
import tf2_ros

class TFTest(Node):
    def __init__(self):
        super().__init__('tf_test')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Wait a bit for TF buffer to fill
        self.create_timer(2.0, self.test_transform)
        self.get_logger().info('TF Test Node started - will test transform in 2 seconds...')

    def test_transform(self):
        try:
            # Test 1: map -> tiago_official/odom
            transform = self.tf_buffer.lookup_transform(
                'map',
                'tiago_official/odom',
                rclpy.time.Time())

            self.get_logger().info('✅ SUCCESS: Found transform map -> tiago_official/odom')
            self.get_logger().info(f'   Translation: x={transform.transform.translation.x:.3f}, '
                                 f'y={transform.transform.translation.y:.3f}, '
                                 f'z={transform.transform.translation.z:.3f}')

        except tf2_ros.LookupException as e:
            self.get_logger().error(f'❌ FAILED: LookupException: {e}')
        except tf2_ros.ConnectivityException as e:
            self.get_logger().error(f'❌ FAILED: ConnectivityException: {e}')
        except tf2_ros.ExtrapolationException as e:
            self.get_logger().error(f'❌ FAILED: ExtrapolationException: {e}')
        except Exception as e:
            self.get_logger().error(f'❌ FAILED: {type(e).__name__}: {e}')

        # Shutdown after test
        self.get_logger().info('Test complete - shutting down')
        rclpy.shutdown()

def main():
    rclpy.init()
    node = TFTest()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
