#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped, Point
from visualization_msgs.msg import Marker
import numpy as np

class ExplorationNode(Node):
    def __init__(self):
        super().__init__('exploration_node')
        
        # Publishers
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.frontier_pub = self.create_publisher(Marker, '/frontiers', 10)
        self.test_pub = self.create_publisher(Marker, '/test_marker', 10)
        
        # Subscribers
        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
        
        self.map = None
        self.frontiers = []
        self.timer = self.create_timer(2.0, self.explore)
        self.get_logger().info('Exploration Node Started')
        
        # Publish a test marker immediately to check RViz
        self.publish_test_marker()
    
    def publish_test_marker(self):
        """Publish a test marker to verify RViz is working"""
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'test'
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.5
        marker.scale.x = 0.3
        marker.scale.y = 0.3
        marker.scale.z = 0.3
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        
        self.test_pub.publish(marker)
        self.get_logger().info('Test marker published at (0,0)')
    
    def map_callback(self, msg):
        self.map = msg
        self.get_logger().info('Map received')
        self.find_frontiers()
    
    def find_frontiers(self):
        """Find frontier cells (boundary between free and unknown)"""
        if self.map is None:
            self.get_logger().warn('No map data')
            return
        
        width = self.map.info.width
        height = self.map.info.height
        data = np.array(self.map.data).reshape((height, width))
        resolution = self.map.info.resolution
        origin_x = self.map.info.origin.position.x
        origin_y = self.map.info.origin.position.y
        
        self.frontiers = []
        frontier_count = 0
        
        for y in range(1, height-1):
            for x in range(1, width-1):
                idx = y * width + x
                
                # Check if current cell is free (0)
                if data[y, x] == 0:
                    # Check neighbors for unknown (-1)
                    for dy in [-1, 0, 1]:
                        for dx in [-1, 0, 1]:
                            if dx == 0 and dy == 0:
                                continue
                            ny, nx = y + dy, x + dx
                            if 0 <= ny < height and 0 <= nx < width:
                                if data[ny, nx] == -1:
                                    # Convert to world coordinates
                                    wx = x * resolution + origin_x
                                    wy = y * resolution + origin_y
                                    self.frontiers.append((wx, wy))
                                    frontier_count += 1
                                    break
        
        self.get_logger().info(f'Found {frontier_count} frontier cells')
        if frontier_count > 0:
            self.publish_frontiers()
    
    def publish_frontiers(self):
        """Visualize frontiers as red dots"""
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'frontiers'
        marker.id = 0
        marker.type = Marker.POINTS
        marker.action = Marker.ADD
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        
        # Limit to first 200 for performance
        points_to_publish = min(200, len(self.frontiers))
        for i in range(points_to_publish):
            fx, fy = self.frontiers[i]
            p = Point()
            p.x = fx
            p.y = fy
            p.z = 0.0
            marker.points.append(p)
        
        self.frontier_pub.publish(marker)
        self.get_logger().info(f'Published {points_to_publish} frontier markers')
    
    def explore(self):
        """Send goal to first frontier"""
        if not self.frontiers:
            self.get_logger().info('No frontiers to explore')
            return
        
        goal_x, goal_y = self.frontiers[0]
        
        goal_msg = PoseStamped()
        goal_msg.header.frame_id = 'map'
        goal_msg.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.position.x = goal_x
        goal_msg.pose.position.y = goal_y
        goal_msg.pose.position.z = 0.0
        goal_msg.pose.orientation.w = 1.0
        
        self.goal_pub.publish(goal_msg)
        self.get_logger().info(f'Exploring to: ({goal_x:.1f}, {goal_y:.1f})')

def main(args=None):
    rclpy.init(args=args)
    node = ExplorationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
