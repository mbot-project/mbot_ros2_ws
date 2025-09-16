#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
import numpy as np
from collections import deque
import math
from tf2_ros import Buffer, TransformListener
from tf2_ros import TransformException

class FrontierExplorer(Node):
    def __init__(self):
        super().__init__('frontier_explorer')
        
        # Subscribe to map (from SLAM)
        self.costmap_sub = self.create_subscription(
            OccupancyGrid, 
            '/map',
            self.costmap_callback, 
            10
        )
        
        # Action client for navigation
        self.nav_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        
        # TF for robot position
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Frontier exploration state
        self.costmap = None
        self.exploring = False
        self.min_frontier_distance = 0.15  # Minimum distance to frontiers in meters
        self.frontier_cluster_distance = 0.15  # Distance to cluster frontiers
        self.goal_clearance = 0.1  # Required clearance around goals in meters (robot_radius + small margin)

        self.get_logger().info("Frontier Explorer started!")

        # Start exploration after 5 seconds
        self.create_timer(5.0, self.start_exploration)
    
    def costmap_callback(self, msg):
        self.costmap = msg
        
    def start_exploration(self):
        self.get_logger().info(f"start_exploration called - exploring: {self.exploring}, costmap available: {self.costmap is not None}")
        if not self.exploring and self.costmap is not None:
            self.exploring = True
            self.get_logger().info("Starting exploration...")
            self.explore_next()
        elif self.exploring:
            self.get_logger().info("Already exploring")
        else:
            self.get_logger().warn("No costmap available yet")
    
    def explore_next(self):
        if self.costmap is None:
            self.get_logger().warn("No costmap available")
            return
            
        # Find frontiers
        frontiers = self.find_frontiers()
        
        if not frontiers:
            self.get_logger().info("Exploration complete - no more frontiers!")
            self.exploring = False
            return
            
        # Cluster frontiers to reduce redundancy
        clustered_frontiers = self.cluster_frontiers(frontiers)
        
        # Filter frontiers by minimum distance and clearance
        robot_x, robot_y = self.get_robot_position()
        valid_frontiers = []
        for f in clustered_frontiers:
            distance = math.sqrt((f[0]-robot_x)**2 + (f[1]-robot_y)**2)
            if distance >= self.min_frontier_distance and self.has_clearance(f[0], f[1]):
                valid_frontiers.append((f[0], f[1], distance))
        
        if not valid_frontiers:
            self.get_logger().warn(f"No frontiers with sufficient distance ({self.min_frontier_distance}m) and clearance ({self.goal_clearance}m) found!")
            self.exploring = False
            return
            
        # Pick closest valid frontier
        closest_frontier = min(valid_frontiers, key=lambda f: f[2])
        
        self.get_logger().info(f"Robot at ({robot_x:.2f}, {robot_y:.2f}), selected frontier at ({closest_frontier[0]:.2f}, {closest_frontier[1]:.2f}), distance: {closest_frontier[2]:.2f}m")
        
        # Send goal
        self.send_goal(closest_frontier[0], closest_frontier[1])
    
    def find_frontiers(self):
        """Find frontier cells: free cells adjacent to unknown cells"""
        frontiers = []
        
        # Convert to numpy array for easier processing
        data = np.array(self.costmap.data).reshape(
            (self.costmap.info.height, self.costmap.info.width))
        
        height, width = data.shape
        
        for y in range(1, height-1):
            for x in range(1, width-1):
                # Check if current cell is free
                if data[y, x] == 0:  # free space
                    # Check if any neighbor is unknown (8-directional)
                    neighbors = [
                        data[y-1, x], data[y+1, x], 
                        data[y, x-1], data[y, x+1],
                        data[y-1, x-1], data[y-1, x+1],
                        data[y+1, x-1], data[y+1, x+1]
                    ]
                    if any(n == -1 for n in neighbors):  # -1 = unknown in ROS
                        # Convert grid to world coordinates
                        world_x = x * self.costmap.info.resolution + self.costmap.info.origin.position.x
                        world_y = y * self.costmap.info.resolution + self.costmap.info.origin.position.y
                        frontiers.append((world_x, world_y))
        
        self.get_logger().info(f"Found {len(frontiers)} frontiers")
        return frontiers
    
    def cluster_frontiers(self, frontiers):
        """Group nearby frontiers into clusters and return cluster centers"""
        if not frontiers:
            return []
            
        clusters = []
        used = set()
        
        for i, frontier in enumerate(frontiers):
            if i in used:
                continue
                
            # Start new cluster
            cluster = [frontier]
            used.add(i)
            
            # Find nearby frontiers
            for j, other in enumerate(frontiers):
                if j in used:
                    continue
                    
                distance = math.sqrt((frontier[0] - other[0])**2 + (frontier[1] - other[1])**2)
                if distance <= self.frontier_cluster_distance:
                    cluster.append(other)
                    used.add(j)
            
            # Calculate cluster center
            center_x = sum(f[0] for f in cluster) / len(cluster)
            center_y = sum(f[1] for f in cluster) / len(cluster)
            clusters.append((center_x, center_y))
        
        self.get_logger().info(f"Clustered {len(frontiers)} frontiers into {len(clusters)} clusters")
        return clusters
    
    def has_clearance(self, x, y):
        """Check if a position has sufficient clearance from obstacles"""
        if self.costmap is None:
            return False
            
        # Convert world coordinates to grid coordinates
        grid_x = int((x - self.costmap.info.origin.position.x) / self.costmap.info.resolution)
        grid_y = int((y - self.costmap.info.origin.position.y) / self.costmap.info.resolution)
        
        # Calculate clearance radius in grid cells
        clearance_cells = int(self.goal_clearance / self.costmap.info.resolution)
        
        # Check bounds
        if (grid_x < clearance_cells or grid_x >= self.costmap.info.width - clearance_cells or
            grid_y < clearance_cells or grid_y >= self.costmap.info.height - clearance_cells):
            return False
        
        # Convert to numpy array for easier processing
        data = np.array(self.costmap.data).reshape(
            (self.costmap.info.height, self.costmap.info.width))
        
        # Check area around the position - only reject actual obstacles (100)
        # Allow unknown (-1) areas to encourage exploration
        for dy in range(-clearance_cells, clearance_cells + 1):
            for dx in range(-clearance_cells, clearance_cells + 1):
                cell_value = data[grid_y + dy, grid_x + dx]
                # Only reject if actual obstacle (100), allow free (0) and unknown (-1)
                if cell_value == 100:  # Obstacle
                    return False
        
        return True
    
    def get_robot_position(self):
        """Get robot position from TF"""
        try:
            # Get transform from map to base_footprint
            transform = self.tf_buffer.lookup_transform(
                'map', 'base_footprint', rclpy.time.Time()
            )
            x = transform.transform.translation.x
            y = transform.transform.translation.y
            return (x, y)
        except TransformException as ex:
            self.get_logger().warn(f"Could not get robot position: {ex}")
            return (0.0, 0.0)  # Fallback to origin
    
    def send_goal(self, x, y):
        """Send navigation goal"""
        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Navigation server not available")
            return
            
        # Get current robot position for approach angle calculation
        robot_x, robot_y = self.get_robot_position()
        
        # Calculate approach angle (direction from robot to goal)
        approach_angle = math.atan2(y - robot_y, x - robot_x)
        
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        
        # Set orientation based on approach direction
        goal_msg.pose.pose.orientation.z = math.sin(approach_angle / 2.0)
        goal_msg.pose.pose.orientation.w = math.cos(approach_angle / 2.0)
        
        self.get_logger().info(f"Sending goal: ({x:.2f}, {y:.2f})")
        
        # Send goal and wait for result
        send_goal_future = self.nav_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)
    
    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            self.explore_next()  # Try next frontier
            return
            
        self.get_logger().info('Goal accepted')
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.goal_result_callback)
    
    def goal_result_callback(self, future):
        result = future.result().result
        status = result.result if hasattr(result, 'result') else None
        
        if status == 1:  # SUCCEEDED
            self.get_logger().info('Goal reached - continuing exploration')
            self.explore_next()  # No delay for smooth exploration
        else:
            self.get_logger().warn(f'Navigation failed with status: {status} - trying next frontier')
            self.explore_next()  # Try immediately on failure

def main():
    rclpy.init()
    explorer = FrontierExplorer()
    rclpy.spin(explorer)
    explorer.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()