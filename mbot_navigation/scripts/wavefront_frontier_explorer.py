#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.action import NavigateToPose
import numpy as np
import math
from collections import deque
from tf2_ros import Buffer, TransformListener, TransformException


class WavefrontFrontierExplorer(Node):
    def __init__(self):
        super().__init__('wavefront_frontier_explorer')

        # Subscriptions / clients
        self.map_sub = self.create_subscription(
            OccupancyGrid, 'map', self.map_callback, 10
        )
        self.nav_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')

        # TF
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # State
        self.map = None
        self.started = False  # start exploration when map+TF are ready
        
        # Exploration parameters
        self.min_frontier_distance = 0.2  # Minimum distance to frontiers in meters
        self.goal_clearance = 0.1  # Required clearance around goals in meters
        self.frontier_cluster_distance = 0.2  # Distance to cluster frontiers
        self.recent_goals = deque(maxlen=5)  # Remember recent goals to avoid repeats

        self.get_logger().info("Wavefront explorer ready; waiting for map + TF...")

    def map_callback(self, msg: OccupancyGrid):
        self.map = msg

        # Start exactly once, when TF is available to compute robot grid position
        if not self.started:
            gx, gy = self.get_robot_position_grid()
            if gx is not None and gy is not None:
                self.started = True
                self.get_logger().info("Starting wavefront frontier exploration…")
                self.explore()

    # ---- Main loop
    def explore(self):
        occupancy = np.array(self.map.data, dtype=np.int16).reshape(
            (self.map.info.height, self.map.info.width)
        )
        height, width = occupancy.shape
        robot_gx, robot_gy = self.get_robot_position_grid()

        if robot_gx is None or occupancy[robot_gy, robot_gx] != 0:
            self.get_logger().warn("Robot position unknown or invalid in map; retrying...")
            self.started = False
            return
        
        visited_mask = np.zeros((height, width), dtype=bool)
        queue = deque([(robot_gx, robot_gy)])
        visited_mask[robot_gy, robot_gx] = True
        frontier_indices = set()

        # return 4 neighbors but skip out of bounds cell
        def neighbors4(x, y):
            if x + 1 < width:  yield x + 1, y
            if x - 1 >= 0:     yield x - 1, y
            if y + 1 < height: yield x, y + 1
            if y - 1 >= 0:     yield x, y - 1

        # return 8 neighbors but skip out of bounds cell
        def neighbors8(x, y):
            for dx in (-1, 0, 1):
                for dy in (-1, 0, 1):
                    if dx == 0 and dy == 0:
                        continue
                    nx, ny = x + dx, y + dy
                    if 0 <= nx < width and 0 <= ny < height:
                        yield nx, ny

        while queue:
            gx, gy = queue.popleft()
            for nx, ny in neighbors4(gx, gy):
                if visited_mask[ny, nx]:
                    continue
                visited_mask[ny, nx] = True

                if occupancy[ny, nx] == 0:  # free -> enqueue and check if it's a frontier
                    queue.append((nx, ny))

                    # Frontier: free cell with ANY unknown neighbor (8-connected)
                    for mx, my in neighbors8(nx, ny):
                        if occupancy[my, mx] == -1:
                            frontier_indices.add((nx, ny))
                            break  # no need to keep checking this cell
        
        if not frontier_indices:
            self.get_logger().info("No frontiers found. Exploration complete.")
            return
        
        self.get_logger().info(f"Found {len(frontier_indices)} raw frontiers")
        
        # Convert to world coordinates and filter
        robot_wx, robot_wy = self.get_robot_position_world()
        valid_frontiers = []
        
        for gx, gy in frontier_indices:
            wx, wy = self.grid_to_world(gx, gy)
            
            # Check minimum distance
            distance = math.hypot(wx - robot_wx, wy - robot_wy)
            if distance < self.min_frontier_distance:
                continue
                
            # Check clearance
            if not self.has_clearance(wx, wy):
                continue
                
            # Check if too close to recent goals
            if self.too_close_to_recent_goals(wx, wy):
                continue
                
            valid_frontiers.append((wx, wy, distance))
        
        if not valid_frontiers:
            self.get_logger().warn(f"No valid frontiers found (min_distance={self.min_frontier_distance}m, clearance={self.goal_clearance}m)")
            # Try again with reduced constraints
            self.min_frontier_distance *= 0.8
            self.goal_clearance *= 0.8
            self.get_logger().info(f"Reducing constraints: min_distance={self.min_frontier_distance:.2f}m, clearance={self.goal_clearance:.2f}m")
            return
        
        # Cluster frontiers to avoid redundancy
        clustered_frontiers = self.cluster_frontiers(valid_frontiers)
        
        # Select best frontier (closest after filtering)
        goal_x, goal_y, distance = min(clustered_frontiers, key=lambda f: f[2])
        
        self.get_logger().info(f"Selected frontier at ({goal_x:.2f}, {goal_y:.2f}), distance: {distance:.2f}m from {len(clustered_frontiers)} clusters")
        
        # Remember this goal
        self.recent_goals.append((goal_x, goal_y))
        
        self.send_goal(goal_x, goal_y)

    def cluster_frontiers(self, frontiers):
        """Group nearby frontiers into clusters and return cluster centers with distances"""
        if not frontiers:
            return []
            
        clusters = []
        used = set()
        
        for i, (fx, fy, fdist) in enumerate(frontiers):
            if i in used:
                continue
                
            # Start new cluster
            cluster = [(fx, fy, fdist)]
            used.add(i)
            
            # Find nearby frontiers
            for j, (ox, oy, odist) in enumerate(frontiers):
                if j in used:
                    continue
                    
                distance = math.hypot(fx - ox, fy - oy)
                if distance <= self.frontier_cluster_distance:
                    cluster.append((ox, oy, odist))
                    used.add(j)
            
            # Calculate cluster center and average distance
            center_x = sum(f[0] for f in cluster) / len(cluster)
            center_y = sum(f[1] for f in cluster) / len(cluster)
            avg_dist = sum(f[2] for f in cluster) / len(cluster)
            clusters.append((center_x, center_y, avg_dist))
        
        self.get_logger().info(f"Clustered {len(frontiers)} frontiers into {len(clusters)} clusters")
        return clusters

    def has_clearance(self, x, y):
        """Check if a position has sufficient clearance from obstacles"""
        # Convert world coordinates to grid coordinates
        grid_x = int((x - self.map.info.origin.position.x) / self.map.info.resolution)
        grid_y = int((y - self.map.info.origin.position.y) / self.map.info.resolution)
        
        # Calculate clearance radius in grid cells
        clearance_cells = int(self.goal_clearance / self.map.info.resolution)
        
        # Check bounds
        if (grid_x < clearance_cells or grid_x >= self.map.info.width - clearance_cells or
            grid_y < clearance_cells or grid_y >= self.map.info.height - clearance_cells):
            return False
        
        # Convert to numpy array for easier processing
        data = np.array(self.map.data).reshape(
            (self.map.info.height, self.map.info.width))
        
        # Check area around the position - only reject actual obstacles (100)
        for dy in range(-clearance_cells, clearance_cells + 1):
            for dx in range(-clearance_cells, clearance_cells + 1):
                cell_value = data[grid_y + dy, grid_x + dx]
                if cell_value == 100:  # Obstacle
                    return False
        
        return True

    def too_close_to_recent_goals(self, x, y):
        """Check if the goal is too close to recent goals"""
        for recent_x, recent_y in self.recent_goals:
            if math.hypot(x - recent_x, y - recent_y) < self.min_frontier_distance:
                return True
        return False

    # ---- Nav2 ----
    def send_goal(self, x, y):
        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Nav2 action server not available.")
            self.started = False
            return
        robot_x, robot_y = self.get_robot_position_world()
        # Calculate approach angle (direction from robot to goal)
        approach_angle = math.atan2(y - robot_y, x - robot_x)

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y

        # Set orientation based on approach direction
        goal.pose.pose.orientation.z = math.sin(approach_angle / 2.0)
        goal.pose.pose.orientation.w = math.cos(approach_angle / 2.0)

        self.get_logger().info(f"Sending frontier goal: ({x:.2f}, {y:.2f})")

        # Send goal and wait for result
        send_goal_future = self.nav_client.send_goal_async(goal)
        send_goal_future.add_done_callback(self.on_goal_response)

    def on_goal_response(self, future):
        handle = future.result()
        if not handle or not handle.accepted:
            self.get_logger().warn("Goal rejected; recomputing frontier.")
            self.explore()
            return
        
        self.get_logger().info('Goal accepted')
        get_result_future = handle.get_result_async()
        get_result_future.add_done_callback(self.on_goal_result)

    def on_goal_result(self, future):
        # Regardless of success/failure, continue exploring
        result = future.result().result
        status = result.result if hasattr(result, 'result') else None
        
        if status == 1:  # SUCCEEDED
            self.get_logger().info('Goal reached - continuing exploration')
            self.explore() 
        else:
            self.get_logger().warn(f'Navigation failed with status: {status} - trying next frontier')
            self.explore()

    # ---- Helpers ----
    def grid_to_world(self, gx, gy):
        res = self.map.info.resolution
        origin = self.map.info.origin.position
        return gx * res + origin.x, gy * res + origin.y
    
    def get_robot_position_grid(self):
        if self.map is None:
            return None, None
        try:
            tf = self.tf_buffer.lookup_transform('map', 'base_footprint', rclpy.time.Time())
            wx = tf.transform.translation.x
            wy = tf.transform.translation.y
            gx = int((wx - self.map.info.origin.position.x) / self.map.info.resolution)
            gy = int((wy - self.map.info.origin.position.y) / self.map.info.resolution)
            if 0 <= gx < self.map.info.width and 0 <= gy < self.map.info.height:
                return gx, gy
            return None, None
        except TransformException:
            return None, None
        
    def get_robot_position_world(self):
        try:
            tf = self.tf_buffer.lookup_transform('map', 'base_footprint', rclpy.time.Time())
            return tf.transform.translation.x, tf.transform.translation.y
        except TransformException as ex:
            self.get_logger().warn(f"Could not get robot position: {ex}")
            return (0.0, 0.0)  # Fallback to origin
        
def main():
    rclpy.init()
    node = WavefrontFrontierExplorer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()