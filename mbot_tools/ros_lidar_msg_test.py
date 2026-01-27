#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class LaserScanTimingChecker(Node):
    def __init__(self):
        super().__init__('laser_scan_timing_checker')
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10)
        self.get_logger().info('Listening to /scan...')

    def scan_callback(self, msg):
        # Calculate t_end using method 1: header.stamp + (time_increment × number of beams)
        num_beams = len(msg.ranges)
        t_end_method1 = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9 + (msg.time_increment * num_beams)

        # Calculate t_end using method 2: header.stamp + scan_duration
        t_end_method2 = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9 + msg.scan_time

        # Check if they are the same
        difference = abs(t_end_method1 - t_end_method2)

        self.get_logger().info(f'\n'
                              f'Number of beams: {num_beams}\n'
                              f'Time increment: {msg.time_increment:.6f} s\n'
                              f'Scan duration: {msg.scan_time:.6f} s\n'
                              f't_end (method 1): {t_end_method1:.6f} s\n'
                              f't_end (method 2): {t_end_method2:.6f} s\n'
                              f'Difference: {difference:.9f} s\n'
                              f'Are they equal? {difference < 1e-6}')


def main(args=None):
    rclpy.init(args=args)
    node = LaserScanTimingChecker()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
