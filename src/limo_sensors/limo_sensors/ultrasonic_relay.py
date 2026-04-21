#!/usr/bin/env python3
"""
ultrasonic_relay.py
Converts Range messages to LaserScan so other nodes
can subscribe without the ROS2 Humble Range DDS bug.
Range  /ultrasonic_front  →  LaserScan  /ultrasonic_front_scan
Range  /ultrasonic_left   →  LaserScan  /ultrasonic_left_scan
Range  /ultrasonic_right  →  LaserScan  /ultrasonic_right_scan
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Range, LaserScan


class UltrasonicRelay(Node):
    def __init__(self):
        super().__init__('ultrasonic_relay')

        self.create_subscription(
            Range, '/ultrasonic_front',
            self.front_cb, qos_profile_sensor_data)
        self.create_subscription(
            Range, '/ultrasonic_left',
            self.left_cb, qos_profile_sensor_data)
        self.create_subscription(
            Range, '/ultrasonic_right',
            self.right_cb, qos_profile_sensor_data)

        self.pub_front = self.create_publisher(
            LaserScan, '/ultrasonic_front_scan', 10)
        self.pub_left = self.create_publisher(
            LaserScan, '/ultrasonic_left_scan', 10)
        self.pub_right = self.create_publisher(
            LaserScan, '/ultrasonic_right_scan', 10)

        self.get_logger().info('Ultrasonic relay ready')

    def _to_scan(self, msg: Range) -> LaserScan:
        scan = LaserScan()
        scan.header = msg.header
        scan.angle_min = -0.2618
        scan.angle_max =  0.2618
        scan.angle_increment = 0.1047
        scan.range_min = msg.min_range
        scan.range_max = msg.max_range
        scan.ranges = [msg.range] * 5
        return scan

    def front_cb(self, msg: Range):
        self.pub_front.publish(self._to_scan(msg))

    def left_cb(self, msg: Range):
        self.pub_left.publish(self._to_scan(msg))

    def right_cb(self, msg: Range):
        self.pub_right.publish(self._to_scan(msg))


def main(args=None):
    rclpy.init(args=args)
    node = UltrasonicRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
