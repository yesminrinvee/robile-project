#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry, OccupancyGrid, Path
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
import math
import numpy as np
from tf_transformations import euler_from_quaternion

class FullNavigation(Node):
    def __init__(self):
        super().__init__('full_navigation')
        
        # Publishers
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.path_pub = self.create_publisher(Path, '/planned_path', 10)
        
        # Subscribers
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.goal_sub = self.create_subscription(PoseStamped, '/goal_pose', self.goal_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        
        # Map subscription with correct QoS
        map_qos = QoSProfile(
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.map_callback, map_qos)
        
        # Robot state
        self.goal = None
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        self.latest_scan = None
        self.map_data = None
        self.path = []
        self.current_waypoint = 0
        
        # Parameters
        self.k_a = 1.0
        self.k_r = 3.0
        self.rho_0 = 1.5
        self.max_speed = 0.25
        self.max_turn = 0.8
        self.goal_threshold = 0.25
        self.waypoint_threshold = 0.3
        
        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info('Full Navigation Started')
    
    def odom_callback(self, msg):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        _, _, self.robot_yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
    
    def map_callback(self, msg):
        self.map_data = msg
        self.get_logger().info(f'Map received: {msg.info.width}x{msg.info.height}')
    
    def goal_callback(self, msg):
        self.goal = (msg.pose.position.x, msg.pose.position.y)
        self.get_logger().info(f'Goal received: ({self.goal[0]:.2f}, {self.goal[1]:.2f})')
        if self.map_data is not None:
            self.plan_path()
        else:
            self.get_logger().warn('No map yet, cannot plan path')
    
    def scan_callback(self, msg):
        self.latest_scan = msg
    
    def plan_path(self):
        """Simple straight line path"""
        self.path = []
        steps = 10
        for i in range(steps + 1):
            t = i / steps
            x = self.robot_x + t * (self.goal[0] - self.robot_x)
            y = self.robot_y + t * (self.goal[1] - self.robot_y)
            self.path.append((x, y))
        
        self.current_waypoint = 1
        self.get_logger().info(f'Path created with {len(self.path)} waypoints')
        self.publish_path()
    
    def publish_path(self):
        path_msg = Path()
        path_msg.header.frame_id = 'map'
        path_msg.header.stamp = self.get_clock().now().to_msg()
        
        for x, y in self.path:
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)
        
        self.path_pub.publish(path_msg)
    
    def calculate_repulsive_force(self):
        if self.latest_scan is None:
            return 0.0, 0.0
        
        fx_rep, fy_rep = 0.0, 0.0
        angle = self.latest_scan.angle_min
        
        for distance in self.latest_scan.ranges:
            if 0.1 < distance < self.rho_0:
                magnitude = self.k_r * (1.0/distance - 1.0/self.rho_0) / (distance * distance)
                fx_rep -= magnitude * math.cos(angle)
                fy_rep -= magnitude * math.sin(angle)
            angle += self.latest_scan.angle_increment
        
        return fx_rep, fy_rep
    
    def control_loop(self):
        if self.goal is None or not self.path:
            self.cmd_pub.publish(Twist())
            return
        
        # Get current target
        if self.current_waypoint < len(self.path):
            target_x, target_y = self.path[self.current_waypoint]
            
            # Check if reached current waypoint
            dist_to_waypoint = math.hypot(target_x - self.robot_x, target_y - self.robot_y)
            if dist_to_waypoint < self.waypoint_threshold:
                self.current_waypoint += 1
                if self.current_waypoint >= len(self.path):
                    target_x, target_y = self.goal
                else:
                    target_x, target_y = self.path[self.current_waypoint]
        else:
            target_x, target_y = self.goal
        
        # Calculate forces
        dx = target_x - self.robot_x
        dy = target_y - self.robot_y
        dist = math.hypot(dx, dy)
        
        # Check if final goal reached
        if self.current_waypoint >= len(self.path) and dist < self.goal_threshold:
            self.get_logger().info('Goal reached!')
            self.goal = None
            self.path = []
            self.cmd_pub.publish(Twist())
            return
        
        # Attractive force
        if dist > 0:
            fx_att = self.k_a * dx / dist
            fy_att = self.k_a * dy / dist
        else:
            fx_att, fy_att = 0, 0
        
        # Repulsive force
        fx_rep, fy_rep = self.calculate_repulsive_force()
        
        # Transform to robot frame
        cos_yaw = math.cos(self.robot_yaw)
        sin_yaw = math.sin(self.robot_yaw)
        
        fx = fx_att * cos_yaw + fy_att * sin_yaw + fx_rep
        fy = -fx_att * sin_yaw + fy_att * cos_yaw + fy_rep
        
        # Velocity commands
        cmd = Twist()
        cmd.linear.x = min(self.max_speed, abs(fx))
        
        if abs(fy) > 0.1:
            cmd.angular.z = max(-self.max_turn, min(self.max_turn, fy * 2.0))
        else:
            angle_to_target = math.atan2(dy, dx)
            angle_error = angle_to_target - self.robot_yaw
            while angle_error > math.pi:
                angle_error -= 2*math.pi
            while angle_error < -math.pi:
                angle_error += 2*math.pi
            cmd.angular.z = max(-self.max_turn, min(self.max_turn, angle_error * 1.5))
        
        self.cmd_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = FullNavigation()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
