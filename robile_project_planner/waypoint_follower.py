#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped, PoseArray
from nav_msgs.msg import Odometry, Path
import numpy as np
import math
from tf_transformations import euler_from_quaternion

class WaypointFollower(Node):
    def __init__(self):
        super().__init__('waypoint_follower')
        
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.path_sub = self.create_subscription(Path, '/planned_path', self.path_callback, 10)
        
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        self.waypoints = []
        self.current_waypoint_index = 0
        self.goal_threshold = 0.3
        self.max_speed = 0.2
        self.max_turn = 0.8
        
        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info('Waypoint Follower Started')
    
    def odom_callback(self, msg):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        _, _, self.robot_yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
    
    def path_callback(self, msg):
        """Extract waypoints from path"""
        self.waypoints = []
        for pose_stamped in msg.poses:
            self.waypoints.append((pose_stamped.pose.position.x, pose_stamped.pose.position.y))
        
        self.current_waypoint_index = 0
        self.get_logger().info(f'Received path with {len(self.waypoints)} waypoints')
    
    def control_loop(self):
        if not self.waypoints or self.current_waypoint_index >= len(self.waypoints):
            # No path or all waypoints reached
            self.cmd_pub.publish(Twist())
            return
        
        # Get current target waypoint
        target_x, target_y = self.waypoints[self.current_waypoint_index]
        
        # Calculate distance to current waypoint
        dx = target_x - self.robot_x
        dy = target_y - self.robot_y
        dist = math.hypot(dx, dy)
        
        # If close to current waypoint, move to next
        if dist < self.goal_threshold:
            self.current_waypoint_index += 1
            if self.current_waypoint_index >= len(self.waypoints):
                self.get_logger().info('All waypoints reached!')
                self.cmd_pub.publish(Twist())
                return
            target_x, target_y = self.waypoints[self.current_waypoint_index]
            dx = target_x - self.robot_x
            dy = target_y - self.robot_y
            dist = math.hypot(dx, dy)
        
        # Calculate angle to target
        target_angle = math.atan2(dy, dx)
        angle_error = target_angle - self.robot_yaw
        
        # Normalize angle
        while angle_error > math.pi:
            angle_error -= 2*math.pi
        while angle_error < -math.pi:
            angle_error += 2*math.pi
        
        cmd = Twist()
        
        # Simple proportional control
        cmd.linear.x = min(self.max_speed, dist * 0.5)
        cmd.angular.z = max(-self.max_turn, min(self.max_turn, angle_error * 1.5))
        
        self.cmd_pub.publish(cmd)
        self.get_logger().info(f'Moving to waypoint {self.current_waypoint_index+1}/{len(self.waypoints)}, dist={dist:.2f}m')

def main(args=None):
    rclpy.init(args=args)
    node = WaypointFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
