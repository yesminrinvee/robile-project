#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped, Pose, Point
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry, OccupancyGrid, Path
from visualization_msgs.msg import Marker
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
import math
import numpy as np
import heapq
from tf_transformations import euler_from_quaternion

class AStarNode:
    def __init__(self, x, y, g, h, parent=None):
        self.x = x
        self.y = y
        self.g = g
        self.h = h
        self.f = g + h
        self.parent = parent
    
    def __lt__(self, other):
        return self.f < other.f

class AStarNavigation(Node):
    def __init__(self):
        super().__init__('astar_navigation')
        
        # Publishers
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.path_pub = self.create_publisher(Path, '/planned_path', 10)
        self.marker_pub = self.create_publisher(Marker, '/path_marker', 10)
        
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
        self.goal_pos = None
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        self.latest_scan = None
        self.map_data = None
        self.map_info = None
        self.grid = None
        self.path = []
        self.current_waypoint = 0
        self.path_received = False
        
        # Parameters
        self.cell_size = 0.05
        self.max_speed = 0.2
        self.max_turn = 0.8
        self.goal_threshold = 0.3
        self.waypoint_threshold = 0.3
        self.obstacle_distance = 1.0
        self.repulsive_gain = 3.0
        
        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info('A* Navigation Started')
    
    def odom_callback(self, msg):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        _, _, self.robot_yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
    
    def map_callback(self, msg):
        self.map_info = msg.info
        self.map_data = msg.data
        height = msg.info.height
        width = msg.info.width
        self.grid = np.array(msg.data).reshape((height, width))
        
        # Inflate obstacles slightly for safer planning
        inflated_grid = np.copy(self.grid)
        for y in range(height):
            for x in range(width):
                if self.grid[y, x] == 100:  # Occupied
                    # Mark neighboring cells as occupied too
                    for dy in [-1, 0, 1]:
                        for dx in [-1, 0, 1]:
                            nx, ny = x + dx, y + dy
                            if 0 <= nx < width and 0 <= ny < height:
                                inflated_grid[ny, nx] = 100
        self.grid = inflated_grid
        self.get_logger().info(f'Map loaded: {width}x{height} with obstacle inflation')
    
    def goal_callback(self, msg):
        self.goal = (msg.pose.position.x, msg.pose.position.y)
        self.goal_pos = msg.pose
        self.get_logger().info(f'Goal received: ({self.goal[0]:.2f}, {self.goal[1]:.2f})')
        
        if self.grid is not None:
            self.plan_path()
        else:
            self.get_logger().warn('No map available')
    
    def scan_callback(self, msg):
        self.latest_scan = msg
    
    def world_to_grid(self, x, y):
        gx = int((x - self.map_info.origin.position.x) / self.map_info.resolution)
        gy = int((y - self.map_info.origin.position.y) / self.map_info.resolution)
        return gx, gy
    
    def grid_to_world(self, gx, gy):
        x = gx * self.map_info.resolution + self.map_info.origin.position.x
        y = gy * self.map_info.resolution + self.map_info.origin.position.y
        return x, y
    
    def is_cell_free(self, gx, gy):
        if gx < 0 or gx >= self.map_info.width or gy < 0 or gy >= self.map_info.height:
            return False
        return self.grid[gy, gx] == 0
    
    def heuristic(self, x1, y1, x2, y2):
        return math.hypot(x2 - x1, y2 - y1)
    
    def get_neighbors(self, x, y):
        neighbors = []
        for dx in [-1, 0, 1]:
            for dy in [-1, 0, 1]:
                if dx == 0 and dy == 0:
                    continue
                nx, ny = x + dx, y + dy
                if self.is_cell_free(nx, ny):
                    cost = 1.4 if dx != 0 and dy != 0 else 1.0
                    neighbors.append((nx, ny, cost))
        return neighbors
    
    def plan_path(self):
        start_gx, start_gy = self.world_to_grid(self.robot_x, self.robot_y)
        goal_gx, goal_gy = self.world_to_grid(self.goal[0], self.goal[1])
        
        self.get_logger().info(f'Start grid: ({start_gx}, {start_gy}), Goal grid: ({goal_gx}, {goal_gy})')
        
        # Check if start and goal are free
        if not self.is_cell_free(start_gx, start_gy):
            self.get_logger().error(f'Start cell ({start_gx}, {start_gy}) is occupied! Using direct navigation.')
            self.use_direct_path()
            return
            
        if not self.is_cell_free(goal_gx, goal_gy):
            self.get_logger().error(f'Goal cell ({goal_gx}, {goal_gy}) is occupied! Using direct navigation.')
            self.use_direct_path()
            return
        
        # A* algorithm
        open_set = []
        closed_set = set()
        start_node = AStarNode(start_gx, start_gy, 0, 
                               self.heuristic(start_gx, start_gy, goal_gx, goal_gy))
        heapq.heappush(open_set, start_node)
        
        nodes = {(start_gx, start_gy): start_node}
        path_found = False
        
        while open_set:
            current = heapq.heappop(open_set)
            
            if (current.x, current.y) == (goal_gx, goal_gy):
                path_found = True
                self.path = []
                while current:
                    wx, wy = self.grid_to_world(current.x, current.y)
                    self.path.append((wx, wy))
                    current = current.parent
                self.path.reverse()
                break
            
            closed_set.add((current.x, current.y))
            
            for nx, ny, move_cost in self.get_neighbors(current.x, current.y):
                if (nx, ny) in closed_set:
                    continue
                
                g = current.g + move_cost
                h = self.heuristic(nx, ny, goal_gx, goal_gy)
                
                if (nx, ny) not in nodes:
                    node = AStarNode(nx, ny, g, h, current)
                    nodes[(nx, ny)] = node
                    heapq.heappush(open_set, node)
                elif g < nodes[(nx, ny)].g:
                    node = nodes[(nx, ny)]
                    node.g = g
                    node.f = g + node.h
                    node.parent = current
                    heapq.heappush(open_set, node)
        
        if path_found:
            self.path_received = True
            self.current_waypoint = 1
            self.get_logger().info(f'Path found with {len(self.path)} waypoints')
            self.publish_path()
            self.publish_markers()
        else:
            self.get_logger().error('No path found! Using direct navigation.')
            self.use_direct_path()
    
    def use_direct_path(self):
        """Fallback to straight line path"""
        self.path = [(self.robot_x, self.robot_y), self.goal]
        self.path_received = True
        self.current_waypoint = 1
        self.get_logger().warn('Using straight line fallback')
        self.publish_path()
        self.publish_markers()
    
    def publish_path(self):
        path_msg = Path()
        path_msg.header.frame_id = 'map'
        path_msg.header.stamp = self.get_clock().now().to_msg()
        
        for x, y in self.path:
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)
        
        self.path_pub.publish(path_msg)
    
    def publish_markers(self):
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'waypoints'
        marker.id = 0
        marker.type = Marker.SPHERE_LIST
        marker.action = Marker.ADD
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2
        marker.color.a = 1.0
        marker.color.b = 1.0
        
        for x, y in self.path:
            p = Point()
            p.x = x
            p.y = y
            p.z = 0.0
            marker.points.append(p)
        
        self.marker_pub.publish(marker)
    
    def calculate_repulsive_force(self):
        if self.latest_scan is None:
            return 0.0, 0.0
        
        fx_rep, fy_rep = 0.0, 0.0
        angle = self.latest_scan.angle_min
        
        for distance in self.latest_scan.ranges:
            if 0.1 < distance < self.obstacle_distance:
                magnitude = self.repulsive_gain * (1.0/distance - 1.0/self.obstacle_distance) / (distance * distance)
                fx_rep -= magnitude * math.cos(angle)
                fy_rep -= magnitude * math.sin(angle)
            angle += self.latest_scan.angle_increment
        
        return fx_rep, fy_rep
    
    def control_loop(self):
        if not self.path_received or not self.path:
            self.cmd_pub.publish(Twist())
            return
        
        if self.current_waypoint < len(self.path):
            target_x, target_y = self.path[self.current_waypoint]
            
            dist = math.hypot(target_x - self.robot_x, target_y - self.robot_y)
            if dist < self.waypoint_threshold:
                self.current_waypoint += 1
                self.get_logger().info(f'Reached waypoint {self.current_waypoint-1}/{len(self.path)}')
                if self.current_waypoint >= len(self.path):
                    target_x, target_y = self.goal
        else:
            target_x, target_y = self.goal
        
        dx = target_x - self.robot_x
        dy = target_y - self.robot_y
        dist = math.hypot(dx, dy)
        
        if self.current_waypoint >= len(self.path) and dist < self.goal_threshold:
            self.get_logger().info('Goal reached!')
            self.goal = None
            self.path = []
            self.path_received = False
            self.cmd_pub.publish(Twist())
            return
        
        if dist > 0:
            fx_att = dx / dist
            fy_att = dy / dist
        else:
            fx_att, fy_att = 0, 0
        
        fx_rep, fy_rep = self.calculate_repulsive_force()
        
        fx = fx_att + fx_rep
        fy = fy_att + fy_rep
        
        cos_yaw = math.cos(self.robot_yaw)
        sin_yaw = math.sin(self.robot_yaw)
        fx_robot = fx * cos_yaw + fy * sin_yaw
        fy_robot = -fx * sin_yaw + fy * cos_yaw
        
        cmd = Twist()
        
        if fx_robot > 0:
            cmd.linear.x = min(self.max_speed, fx_robot * self.max_speed)
        else:
            cmd.linear.x = 0.0
        
        angle_to_target = math.atan2(dy, dx)
        angle_error = angle_to_target - self.robot_yaw
        while angle_error > math.pi:
            angle_error -= 2*math.pi
        while angle_error < -math.pi:
            angle_error += 2*math.pi
        
        cmd.angular.z = max(-self.max_turn, min(self.max_turn, angle_error * 1.5))
        
        if abs(fy_robot) > 0.3:
            cmd.angular.z = max(-self.max_turn, min(self.max_turn, fy_robot * 2.0))
        
        self.cmd_pub.publish(cmd)
        self.get_logger().info(f'Moving to waypoint {self.current_waypoint+1}/{len(self.path)}, dist={dist:.2f}')

def main(args=None):
    rclpy.init(args=args)
    node = AStarNavigation()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
