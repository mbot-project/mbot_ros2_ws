#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
import yaml
import numpy as np


class CalibrationMarkerPublisher(Node):
    def __init__(self):
        super().__init__('calibration_marker_publisher')
        
        # Declare parameters
        self.declare_parameter('marker_config_file', 'marker.yaml')
        self.declare_parameter('camera_frame', 'camera')
        self.declare_parameter('marker_frame', 'camera')
        
        # Get parameters
        marker_config_file = self.get_parameter('marker_config_file').get_parameter_value().string_value
        self.camera_frame = self.get_parameter('camera_frame').get_parameter_value().string_value
        self.marker_frame = self.get_parameter('marker_frame').get_parameter_value().string_value
        
        # Load marker configuration
        self.load_marker_config(marker_config_file)
        
        # Create publisher
        self.marker_pub = self.create_publisher(Marker, 'calibration_marker', 10)
        
        # Create timer to publish marker
        self.timer = self.create_timer(0.1, self.publish_marker)
        
        self.get_logger().info('Calibration marker publisher started')
    
    def load_marker_config(self, config_file):
        """Load marker configuration from YAML file"""
        try:
            # Try to load from package share directory first
            from ament_index_python.packages import get_package_share_directory
            import os
            package_share_dir = get_package_share_directory('mbot_vision')
            config_path = os.path.join(package_share_dir, 'config', config_file)
            
            with open(config_path, 'r') as file:
                config = yaml.safe_load(file)
                
            # Extract marker properties
            self.marker_pose = config['marker']['pose']
            self.marker_size = config['marker']['size']
            self.marker_type = config['marker'].get('type', 'CUBE')
            self.marker_color = config['marker'].get('color', {'r': 1.0, 'g': 0.0, 'b': 0.0, 'a': 0.8})
            
            self.get_logger().info(f'Loaded marker config from {config_path}')
            
        except Exception as e:
            self.get_logger().warn(f'Could not load config file: {e}')
            self.get_logger().info('Using default marker configuration')
            
            # Default configuration
            self.marker_pose = {'x': 0.5, 'y': 0.0, 'z': 0.1}
            self.marker_size = {'x': 0.1, 'y': 0.1, 'z': 0.1}
            self.marker_type = 'CUBE'
            self.marker_color = {'r': 1.0, 'g': 0.0, 'b': 0.0, 'a': 0.8}
    
    def publish_marker(self):
        """Publish the calibration marker"""
        marker = Marker()
        
        # Header
        marker.header.frame_id = self.marker_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        
        # Marker properties
        marker.ns = "calibration_test"
        marker.id = 0
        marker.type = getattr(Marker, self.marker_type)
        marker.action = Marker.ADD
        
        # Position
        marker.pose.position.x = float(self.marker_pose['x'])
        marker.pose.position.y = float(self.marker_pose['y'])
        marker.pose.position.z = float(self.marker_pose['z'])
        
        # Orientation (default)
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        
        # Scale
        marker.scale.x = float(self.marker_size['x'])
        marker.scale.y = float(self.marker_size['y'])
        marker.scale.z = float(self.marker_size['z'])
        
        # Color
        marker.color.r = float(self.marker_color['r'])
        marker.color.g = float(self.marker_color['g'])
        marker.color.b = float(self.marker_color['b'])
        marker.color.a = float(self.marker_color['a'])
        
        # Lifetime (0 = forever)
        marker.lifetime.sec = 0
        marker.lifetime.nanosec = 0
        
        # Publish marker
        self.marker_pub.publish(marker)


def main(args=None):
    rclpy.init(args=args)
    node = CalibrationMarkerPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()