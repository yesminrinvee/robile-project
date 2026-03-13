#!/usr/bin/env python3
import math
import numpy as np

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist, PoseStamped, Point
from nav_msgs.msg import Odometry, Path
from visualization_msgs.msg import Marker
from sensor_msgs.msg import LaserScan

from tf_transformations import euler_from_quaternion


class FinalNavigation(Node):
    def __init__(self):
        super().__init__('final_navigation')

        # Publishers
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.path_pub = self.create_publisher(Path, '/planned_path', 10)
        self.marker_pub = self.create_publisher(Marker, '/path_markers', 10)

        # Subscribers
        self.goal_sub = self.create_subscription(
            PoseStamped, '/goal_pose', self.goal_callback, 10
        )
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10
        )
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10
        )

        # Robot state
        self.goal = None
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        self.have_odom = False
        self.have_scan = False

        # Path / waypoints
        self.path = []
        self.current_waypoint = 0

        # Laser scan data
        self.scan_ranges = []
        self.scan_angle_min = 0.0
        self.scan_angle_increment = 0.0

        # Parameters
        self.max_speed = 0.18
        self.max_turn = 0.8
        self.goal_threshold = 0.25
        self.waypoint_threshold = 0.25
        self.num_waypoints = 12

        # Obstacle avoidance
        self.obstacle_distance = 0.55
        self.side_check_distance = 0.45
        self.rotate_in_place_angle_error = 0.45

        # Use odom frame unless your goal and odom are both transformed to map correctly
        self.path_frame = 'odom'

        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info('FINAL NAVIGATION STARTED - Ready for goals')

    # --------------------------
    # Callbacks
    # --------------------------
    def odom_callback(self, msg):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation
        _, _, self.robot_yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.have_odom = True

    def scan_callback(self, msg):
        self.scan_ranges = list(msg.ranges)
        self.scan_angle_min = msg.angle_min
        self.scan_angle_increment = msg.angle_increment
        self.have_scan = True

    def goal_callback(self, msg):
        if not self.have_odom:
            self.get_logger().warn('No odom yet, cannot create path')
            return

        self.goal = (msg.pose.position.x, msg.pose.position.y)
        self.get_logger().info(
            f'GOAL RECEIVED: ({self.goal[0]:.2f}, {self.goal[1]:.2f})'
        )

        self.create_path()
        self.publish_path()
        self.publish_markers()

    # --------------------------
    # Path creation
    # --------------------------
    def create_path(self):
        """
        Create simple straight-line waypoints from current robot pose to goal.
        This gives visible waypoints/path in RViz.
        """
        self.path = []

        if self.goal is None:
            return

        start_x = self.robot_x
        start_y = self.robot_y
        goal_x, goal_y = self.goal

        for i in range(self.num_waypoints + 1):
            t = i / float(self.num_waypoints)
            x = start_x + t * (goal_x - start_x)
            y = start_y + t * (goal_y - start_y)
            self.path.append((x, y))

        # Skip waypoint 0 because it is the current robot position
        self.current_waypoint = 1 if len(self.path) > 1 else 0

        self.get_logger().info(f'Path created with {len(self.path)} waypoints')

    def publish_path(self):
        path_msg = Path()
        path_msg.header.frame_id = self.path_frame
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
        marker.header.frame_id = self.path_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'waypoints'
        marker.id = 0
        marker.type = Marker.SPHERE_LIST
        marker.action = Marker.ADD
        marker.scale.x = 0.15
        marker.scale.y = 0.15
        marker.scale.z = 0.15
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 1.0

        for x, y in self.path:
            p = Point()
            p.x = x
            p.y = y
            p.z = 0.0
            marker.points.append(p)

        self.marker_pub.publish(marker)

    # --------------------------
    # Laser helper functions
    # --------------------------
    def get_range_at_angle(self, angle_rad):
        """
        Get approximate scan range at given angle relative to robot front.
        angle_rad = 0 means front, positive left, negative right.
        """
        if not self.have_scan or len(self.scan_ranges) == 0:
            return float('inf')

        index = int((angle_rad - self.scan_angle_min) / self.scan_angle_increment)
        if index < 0 or index >= len(self.scan_ranges):
            return float('inf')

        r = self.scan_ranges[index]
        if math.isinf(r) or math.isnan(r):
            return float('inf')
        return r

    def get_sector_min_distance(self, angle_start, angle_end, samples=15):
        """
        Get minimum valid distance in a scan sector.
        """
        if not self.have_scan or len(self.scan_ranges) == 0:
            return float('inf')

        angles = np.linspace(angle_start, angle_end, samples)
        vals = []

        for a in angles:
            d = self.get_range_at_angle(a)
            if not math.isinf(d) and not math.isnan(d):
                vals.append(d)

        if len(vals) == 0:
            return float('inf')

        return min(vals)

    def obstacle_info(self):
        """
        Returns front, left, right minimum distances.
        """
        front = self.get_sector_min_distance(-0.30, 0.30)
        left = self.get_sector_min_distance(0.35, 1.0)
        right = self.get_sector_min_distance(-1.0, -0.35)
        return front, left, right

    # --------------------------
    # Control
    # --------------------------
    def control_loop(self):
        if not self.have_odom:
            return

        if self.goal is None:
            return

        if len(self.path) == 0:
            self.create_path()
            self.publish_path()
            self.publish_markers()
            return

        
        self.publish_path()
        self.publish_markers()

        # Determine current target waypoint
        if self.current_waypoint < len(self.path):
            target_x, target_y = self.path[self.current_waypoint]
        else:
            target_x, target_y = self.goal

        # Check waypoint reached
        waypoint_dist = math.hypot(target_x - self.robot_x, target_y - self.robot_y)
        if waypoint_dist < self.waypoint_threshold:
            self.current_waypoint += 1
            self.get_logger().info(
                f'Reached waypoint {self.current_waypoint}/{len(self.path)}'
            )
            if self.current_waypoint >= len(self.path):
                target_x, target_y = self.goal
            else:
                target_x, target_y = self.path[self.current_waypoint]

        # Goal reached
        goal_dist = math.hypot(self.goal[0] - self.robot_x, self.goal[1] - self.robot_y)
        if goal_dist < self.goal_threshold:
            self.get_logger().info('GOAL REACHED!')
            self.cmd_pub.publish(Twist())
            self.goal = None
            self.path = []
            self.current_waypoint = 0
            return

        # Compute heading to current target
        dx = target_x - self.robot_x
        dy = target_y - self.robot_y
        target_angle = math.atan2(dy, dx)
        angle_error = self.normalize_angle(target_angle - self.robot_yaw)

        cmd = Twist()

        # --------------------------
        # Obstacle avoidance
        # --------------------------
        front_dist, left_dist, right_dist = self.obstacle_info()

        # Case 1: obstacle directly in front -> turn to clearer side
        if front_dist < self.obstacle_distance:
            cmd.linear.x = 0.0

            if left_dist > right_dist:
                cmd.angular.z = min(self.max_turn, 0.7)
                self.get_logger().info(
                    f'Obstacle ahead ({front_dist:.2f}m). Turning LEFT'
                )
            else:
                cmd.angular.z = max(-self.max_turn, -0.7)
                self.get_logger().info(
                    f'Obstacle ahead ({front_dist:.2f}m). Turning RIGHT'
                )

            self.cmd_pub.publish(cmd)
            return

        # Case 2: large heading error -> rotate first
        if abs(angle_error) > self.rotate_in_place_angle_error:
            cmd.linear.x = 0.0
            cmd.angular.z = max(-self.max_turn, min(self.max_turn, 1.2 * angle_error))
            self.cmd_pub.publish(cmd)
            return

        # Case 3: path is free -> move toward waypoint
        cmd.linear.x = min(self.max_speed, 0.12 + 0.25 * waypoint_dist)
        cmd.angular.z = max(-self.max_turn, min(self.max_turn, 1.5 * angle_error))

        # Slight steering bias away from closer side obstacle
        if left_dist < self.side_check_distance:
            cmd.angular.z -= 0.25
        if right_dist < self.side_check_distance:
            cmd.angular.z += 0.25

        cmd.angular.z = max(-self.max_turn, min(self.max_turn, cmd.angular.z))

        self.cmd_pub.publish(cmd)


    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle


def main(args=None):
    rclpy.init(args=args)
    node = FinalNavigation()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cmd_pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
