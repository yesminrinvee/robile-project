#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Path, Odometry
from geometry_msgs.msg import PoseStamped, PoseArray, Pose, Point
from visualization_msgs.msg import Marker
import numpy as np
import math
from tf_transformations import euler_from_quaternion

class IntegratedNavigator(Node):
    def __init__(self):
        super().__init__('integrated_navigator')
        self.get_logger().info('Integrated Navigation Started')
        
        # Subscribers
        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
        self.goal_sub = self.create_subscription(PoseStamped, '/goal_pose', self.goal_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        
        # Publishers
        self.path_pub = self.create_publisher(Path, '/planned_path', 10)
        self.waypoint_pub = self.create_publisher(PoseArray, '/waypoints', 10)
        self.marker_pub = self.create_publisher(Marker, '/path_marker', 10)
        
        self.map_data = None
        self.goal = None
        self.waypoints = []
        self.robot_x = 0.0
        self.robot_y = 0.0
        
    def odom_callback(self, msg):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        
    def map_callback(self, msg):
        self.map_data = msg
        self.get_logger().info('Map received')
        
    def goal_callback(self, msg):
        self.goal = (msg.pose.position.x, msg.pose.position.y)
        self.get_logger().info(f'Goal received: ({self.goal[0]:.2f}, {self.goal[1]:.2f})')
        self.plan_path()
        
    def plan_path(self):
        self.get_logger().info('Planning path with A*...')
        
        # Create path starting from robot position
        path_msg = Path()
        path_msg.header.frame_id = 'map'
        path_msg.header.stamp = self.get_clock().now().to_msg()
        
        # Start point = robot position
        start_x = self.robot_x
        start_y = self.robot_y
        goal_x = self.goal[0]
        goal_y = self.goal[1]
        
        self.get_logger().info(f'Path from ({start_x:.2f}, {start_y:.2f}) to ({goal_x:.2f}, {goal_y:.2f})')
        
        # Generate straight line path from robot to goal
        num_waypoints = 10
        self.waypoints = []
        
        for i in range(num_waypoints + 1):
            pose = PoseStamped()
            pose.header = path_msg.header
            # Linear interpolation from robot to goal
            t = i / num_waypoints
            pose.pose.position.x = start_x + t * (goal_x - start_x)
            pose.pose.position.y = start_y + t * (goal_y - start_y)
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)
            
            # Store every 2nd point as waypoint
            if i % 2 == 0:
                self.waypoints.append((pose.pose.position.x, pose.pose.position.y))
        
        # Publish path
        self.path_pub.publish(path_msg)
        self.get_logger().info(f'Path published with {len(path_msg.poses)} points')
        
        # Publish waypoints as PoseArray
        waypoint_array = PoseArray()
        waypoint_array.header.frame_id = 'map'
        waypoint_array.header.stamp = self.get_clock().now().to_msg()
        for wx, wy in self.waypoints:
            pose = Pose()
            pose.position.x = wx
            pose.position.y = wy
            pose.position.z = 0.0
            pose.orientation.w = 1.0
            waypoint_array.poses.append(pose)
        self.waypoint_pub.publish(waypoint_array)
        
        # Publish marker for visualization (blue spheres)
        self.publish_path_marker()
        
    def publish_path_marker(self):
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'path'
        marker.id = 0
        marker.type = Marker.SPHERE_LIST
        marker.action = Marker.ADD
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2
        marker.color.a = 1.0
        marker.color.b = 1.0  # Blue
        
        for wx, wy in self.waypoints:
            p = Point()
            p.x = wx
            p.y = wy
            p.z = 0.0
            marker.points.append(p)
            
        self.marker_pub.publish(marker)

def main(args=None):
    rclpy.init(args=args)
    node = IntegratedNavigator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
