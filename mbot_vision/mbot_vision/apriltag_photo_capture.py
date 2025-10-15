#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from apriltag_msgs.msg import AprilTagDetectionArray
from cv_bridge import CvBridge
import cv2
import os
from datetime import datetime
import numpy as np
from ament_index_python.packages import get_package_share_directory


class AprilTagPhotoCapture(Node):
    def __init__(self):
        super().__init__('apriltag_photo_capture')
        
        # Declare parameters
        self.declare_parameter('save_directory', '~/Pictures')
        self.declare_parameter('image_topic', '/image_rect')
        self.declare_parameter('detection_topic', '/detections')
        self.declare_parameter('min_detection_interval', 2.0)  # seconds between photos of same tag
        
        # Get save directory from parameter
        self.save_directory = os.path.expanduser(
            self.get_parameter('save_directory').get_parameter_value().string_value
        )

        # Get other parameters
        image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        detection_topic = self.get_parameter('detection_topic').get_parameter_value().string_value
        self.min_detection_interval = self.get_parameter('min_detection_interval').get_parameter_value().double_value
        
        # Initialize CV bridge
        self.bridge = CvBridge()
        
        # Store the latest image
        self.latest_image = None
        self.last_detection_times = {}  # Track when each tag ID was last photographed
        
        # Create subscribers
        self.image_sub = self.create_subscription(
            Image,
            image_topic,
            self.image_callback,
            10
        )
        
        self.detection_sub = self.create_subscription(
            AprilTagDetectionArray,
            detection_topic,
            self.detection_callback,
            10
        )
        
        self.get_logger().info(f'AprilTag Photo Capture node started')
        self.get_logger().info(f'Saving photos to: {self.save_directory}')
        self.get_logger().info(f'Listening for images on: {image_topic}')
        self.get_logger().info(f'Listening for detections on: {detection_topic}')
    
    def image_callback(self, msg):
        """Store the latest image for photo capture"""
        try:
            self.latest_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().error(f'Failed to convert image: {e}')
    
    def detection_callback(self, msg):
        """Handle AprilTag detections and capture photos"""
        if self.latest_image is None:
            self.get_logger().warn('No image available for photo capture')
            return
        
        if not msg.detections:
            return  # No detections
        
        current_time = self.get_clock().now()
        current_timestamp = current_time.nanoseconds / 1e9
        
        # Check if any tag is ready for a new photo (respecting rate limiting)
        detections_to_capture = []
        for detection in msg.detections:
            tag_id = detection.id
            
            # Check if enough time has passed since last photo of this tag
            if tag_id in self.last_detection_times:
                time_since_last = current_timestamp - self.last_detection_times[tag_id]
                if time_since_last < self.min_detection_interval:
                    continue  # Skip this detection, too soon
            
            detections_to_capture.append(detection)
            self.last_detection_times[tag_id] = current_timestamp
        
        # If any detections are ready, capture one photo with all of them
        if detections_to_capture:
            self.capture_photo(detections_to_capture, current_time)
    
    def capture_photo(self, detections, timestamp):
        """Capture and save a photo with detection information"""
        try:
            # Create timestamp string
            dt = datetime.fromtimestamp(timestamp.nanoseconds / 1e9)
            timestamp_str = dt.strftime('%Y%m%d_%H%M%S')
            
            # Create filename with all detected tag IDs
            tag_ids = [str(detection.id) for detection in detections]
            tag_ids_str = "_".join(tag_ids)
            filename = f'tags_{tag_ids_str}_{timestamp_str}.jpg'
            filepath = os.path.join(self.save_directory, filename)
            
            # Create a copy of the image for annotation
            annotated_image = self.latest_image.copy()
            
            # Draw all detections on image
            for detection in detections:
                self.annotate_detection(annotated_image, detection)
            
            # Save the image
            success = cv2.imwrite(filepath, annotated_image)
            
            if success:
                self.get_logger().info(f'Photo captured for tags {tag_ids_str}: {filename}')
                
            else:
                self.get_logger().error(f'Failed to save photo: {filepath}')
                
        except Exception as e:
            self.get_logger().error(f'Error capturing photo: {e}')
    
    def annotate_detection(self, image, detection):
        """Annotate the image with detection information"""
        try:
            tag_id = detection.id
            
            # Get corner points (assuming they exist in the detection message)
            if hasattr(detection, 'corners') and detection.corners:
                # Draw tag outline
                corners = []
                for corner in detection.corners:
                    corners.append([int(corner.x), int(corner.y)])
                
                corners = np.array(corners, dtype=np.int32)
                cv2.polylines(image, [corners], True, (0, 255, 0), 2)
                
                # Draw tag ID on top of bounding box
                top_y = int(np.min(corners[:, 1]))
                center_x = int(np.mean(corners[:, 0]))
                cv2.putText(image, f'ID: {tag_id}', (center_x - 20, top_y - 10), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            
            # Add timestamp
            timestamp_str = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
            cv2.putText(image, timestamp_str, (10, 30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            
        except Exception as e:
            self.get_logger().warn(f'Could not annotate image: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = AprilTagPhotoCapture()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()