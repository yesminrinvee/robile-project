#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import math
import numpy as np
from tf_transformations import euler_from_quaternion

class PotentialFieldPlanner(Node):
    def __init__(self):
        super().__init__('potential_field_planner')
        
        # Publishers
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Subscribers
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.goal_sub = self.create_subscription(PoseStamped, '/goal_pose', self.goal_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        
        # Robot state
        self.goal = None
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        self.latest_scan = None
        
        # Parameters (tuned from senior's code)
        self.k_a = 1.0      # Attractive force
        self.k_r = 3.0      # Repulsive force
        self.rho_0 = 1.5    # Obstacle influence distance
        self.max_speed = 0.25
        self.max_turn = 0.8
        self.goal_threshold = 0.25
        
        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info('Potential Field Planner Started')
    
    def odom_callback(self, msg):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        _, _, self.robot_yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
    
    def goal_callback(self, msg):
        self.goal = (msg.pose.position.x, msg.pose.position.y)
        self.get_logger().info(f'Goal received: ({self.goal[0]:.2f}, {self.goal[1]:.2f})')
    
    def scan_callback(self, msg):
        self.latest_scan = msg
    
    def calculate_repulsive_force(self):
        """Calculate repulsive force from laser scan data"""
        if self.latest_scan is None:
            return 0.0, 0.0
        
        force_x = 0.0
        force_y = 0.0
        angle = self.latest_scan.angle_min
        
        for distance in self.latest_scan.ranges:
            if 0.1 < distance < self.rho_0:
                # Force magnitude increases as robot gets closer
                magnitude = self.k_r * (1.0/distance - 1.0/self.rho_0) / (distance * distance)
                
                # Direction away from obstacle
                force_x -= magnitude * math.cos(angle)
                force_y -= magnitude * math.sin(angle)
            
            angle += self.latest_scan.angle_increment
        
        return force_x, force_y
    
    def control_loop(self):
        if self.goal is None:
            self.cmd_pub.publish(Twist())
            return
        
        # Vector to goal
        dx = self.goal[0] - self.robot_x
        dy = self.goal[1] - self.robot_y
        dist_to_goal = math.hypot(dx, dy)
        
        # Check if goal reached
        if dist_to_goal < self.goal_threshold:
            self.get_logger().info('Goal reached!')
            self.goal = None
            self.cmd_pub.publish(Twist())
            return
        
        # Attractive force (towards goal)
        if dist_to_goal > 0:
            fx_att = self.k_a * dx / dist_to_goal
            fy_att = self.k_a * dy / dist_to_goal
        else:
            fx_att, fy_att = 0, 0
        
        # Repulsive force (away from obstacles)
        fx_rep, fy_rep = self.calculate_repulsive_force()
        
        # Total force in robot frame
        cos_yaw = math.cos(self.robot_yaw)
        sin_yaw = math.sin(self.robot_yaw)
        
        # Transform attractive force to robot frame
        fx = fx_att * cos_yaw + fy_att * sin_yaw + fx_rep
        fy = -fx_att * sin_yaw + fy_att * cos_yaw + fy_rep
        
        # Convert to velocity command
        cmd = Twist()
        cmd.linear.x = min(self.max_speed, abs(fx))
        
        # Angular velocity based on lateral force
        if abs(fy) > 0.1:
            cmd.angular.z = max(-self.max_turn, min(self.max_turn, fy * 2.0))
        else:
            # Turn towards goal
            goal_angle = math.atan2(dy, dx)
            angle_error = goal_angle - self.robot_yaw
            while angle_error > math.pi:
                angle_error -= 2*math.pi
            while angle_error < -math.pi:
                angle_error += 2*math.pi
            cmd.angular.z = max(-self.max_turn, min(self.max_turn, angle_error * 1.5))
        
        self.cmd_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = PotentialFieldPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
