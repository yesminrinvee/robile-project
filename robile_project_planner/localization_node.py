#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile
from geometry_msgs.msg import Quaternion, Pose, Point, PoseStamped, PoseArray, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry, Path, OccupancyGrid
from sensor_msgs.msg import LaserScan
import numpy as np
from math import atan2, asin, sqrt, pi
import traceback
import sys
import random

MS = 0.001

class ParticleEntity:
    def __init__(self, pose: Pose, weight: float):
        self.pose = pose
        self.weight = weight

class ParticleFilterLocalizer(Node):

    def __init__(self):
        try:
            super().__init__('particle_filter_localizer')
            self.get_logger().info('Launching Particle Filter Localizer node')
            
            # Subscriptions
            self.create_subscription(PoseWithCovarianceStamped, '/initialpose',
                                    self.handle_initial_pose, 10)
            self.create_subscription(Odometry, '/odom',
                                    self.handle_odometry, 10)
            self.create_subscription(LaserScan, '/scan',
                                    self.handle_scan, 10)
            
            # Map subscription with proper QoS
            map_quality = QoSProfile(
                                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                                history=HistoryPolicy.KEEP_LAST,
                                depth=1)
            self.create_subscription(OccupancyGrid, '/map', 
                                    self.handle_map, map_quality)
            
            # Publishers
            self._particle_cloud_pub = self.create_publisher(PoseArray, 
                                                    '/particles', 
                                                    10)
            self.pose_estimation_pub = self.create_publisher(Odometry, 
                                                    '/estimated_pose', 
                                                    10)
            
            # Timer for periodic updates
            self.create_timer(100 * MS, self.periodic_update)
            
            self._particles = []
            self._map_data = None
            self._previous_odom = None
            self._current_position = None
            self._latest_scan = None
            self._robot_moving = False
            self._particle_system_initialized = False
            
            # Initialize default particles after delay
            self.create_timer(2.0, self.initialize_default_particles)
            self.get_logger().info('Setup complete')
            
        except Exception as e:
            self.get_logger().error(f'Setup error: {str(e)}')
            traceback.print_exc()
            raise

    def initialize_default_particles(self):
        """Create default particles if none exist"""
        try:
            if not self._particles:
                self.get_logger().info('Creating default particles')
                default_pose = Pose()
                default_pose.position.x = 0.0
                default_pose.position.y = 0.0
                default_pose.orientation.w = 1.0
                self._current_position = default_pose
                self._previous_odom = default_pose
                self._setup_gaussian_particles()
        except Exception as e:
            self.get_logger().error(f'Error in initialize_default_particles: {str(e)}')

    def handle_map(self, data):
        """Store map data"""
        self._map_data = {
            'resolution': data.info.resolution,
            'width': data.info.width,
            'height': data.info.height,
            'origin': {
                'x': data.info.origin.position.x,
                'y': data.info.origin.position.y
            },
            'data': data.data
        }
        self.get_logger().info(f'Map received: {data.info.width}x{data.info.height}')

    def handle_initial_pose(self, msg):
        """Initialize particles at given pose"""
        try:
            self.get_logger().info('Initial pose received')
            x = msg.pose.pose.position.x
            y = msg.pose.pose.position.y
            q = msg.pose.pose.orientation
            
            # Create pose
            pose = Pose()
            pose.position.x = x
            pose.position.y = y
            pose.orientation = q
            
            self._current_position = pose
            self._previous_odom = pose
            self._setup_gaussian_particles(pose)
            
            # Publish initial estimate
            odom_msg = Odometry()
            odom_msg.header.stamp = self.get_clock().now().to_msg()
            odom_msg.header.frame_id = 'map'
            odom_msg.child_frame_id = 'base_link'
            odom_msg.pose.pose = pose
            self.pose_estimation_pub.publish(odom_msg)
            
        except Exception as e:
            self.get_logger().error(f'Error in initial pose: {str(e)}')

    def handle_odometry(self, msg):
        """Process odometry data and update particles"""
        try:
            if self._previous_odom is None:
                self._previous_odom = msg.pose.pose
                self.get_logger().info('First odometry received')
                return
                
            # Calculate movement
            dx = msg.pose.pose.position.x - self._previous_odom.position.x
            dy = msg.pose.pose.position.y - self._previous_odom.position.y
            
            # Get orientation change
            _, _, yaw1 = self.quaternion_to_euler(
                self._previous_odom.orientation.x,
                self._previous_odom.orientation.y,
                self._previous_odom.orientation.z,
                self._previous_odom.orientation.w)
            _, _, yaw2 = self.quaternion_to_euler(
                msg.pose.pose.orientation.x,
                msg.pose.pose.orientation.y,
                msg.pose.pose.orientation.z,
                msg.pose.pose.orientation.w)
            
            dyaw = yaw2 - yaw1
            # Normalize dyaw to [-pi, pi]
            while dyaw > pi:
                dyaw -= 2*pi
            while dyaw < -pi:
                dyaw += 2*pi
            
            # Log movement for debugging
            if abs(dx) > 0.001 or abs(dy) > 0.001 or abs(dyaw) > 0.001:
                self.get_logger().info(f'Movement: dx={dx:.3f}, dy={dy:.3f}, dyaw={dyaw:.3f}')
                self._robot_moving = True
                
                if self._particles:
                    # Update each particle
                    for particle in self._particles:
                        # Get current heading
                        _, _, heading = self.quaternion_to_euler(
                            particle.pose.orientation.x,
                            particle.pose.orientation.y,
                            particle.pose.orientation.z,
                            particle.pose.orientation.w)
                        
                        # Apply motion in robot frame
                        cos_heading = np.cos(heading)
                        sin_heading = np.sin(heading)
                        
                        # Transform movement to world frame
                        world_dx = dx * cos_heading - dy * sin_heading
                        world_dy = dx * sin_heading + dy * cos_heading
                        
                        # Add noise
                        world_dx += np.random.normal(0, 0.02)
                        world_dy += np.random.normal(0, 0.02)
                        dyaw_noisy = dyaw + np.random.normal(0, 0.01)
                        
                        # Update particle position
                        particle.pose.position.x += world_dx
                        particle.pose.position.y += world_dy
                        
                        # Update heading
                        new_heading = heading + dyaw_noisy
                        new_heading = (new_heading + pi) % (2 * pi) - pi
                        particle.pose.orientation = self.euler_to_quaternion(new_heading)
                    
                    self.get_logger().info(f'Updated {len(self._particles)} particles')
            else:
                self._robot_moving = False
                
            self._previous_odom = msg.pose.pose
            
        except Exception as e:
            self.get_logger().error(f'Error in odometry: {str(e)}')

    def handle_scan(self, msg):
        """Store latest scan"""
        self._latest_scan = msg

    def quaternion_to_euler(self, x, y, z, w):
        """Convert quaternion to euler angles"""
        try:
            t0 = +2.0 * (w * x + y * z)
            t1 = +1.0 - 2.0 * (x * x + y * y)
            roll_x = atan2(t0, t1)

            t2 = +2.0 * (w * y - z * x)
            t2 = +1.0 if t2 > +1.0 else t2
            t2 = -1.0 if t2 < -1.0 else t2
            pitch_y = asin(t2)

            t3 = +2.0 * (w * z + x * y)
            t4 = +1.0 - 2.0 * (y * y + z * z)
            yaw_z = atan2(t3, t4)

            return roll_x, pitch_y, yaw_z
        except:
            return 0.0, 0.0, 0.0

    def euler_to_quaternion(self, yaw, pitch=0.0, roll=0.0):
        """Convert euler angles to quaternion"""
        qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
        qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
        qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        return Quaternion(x=qx, y=qy, z=qz, w=qw)

    def _setup_gaussian_particles(self, pose: Pose = None, scale: float = 0.05):
        """Create particles around given pose"""
        try:
            if pose is None:
                pose = self._current_position
            
            num_particles = 2000
            x_coords = np.random.normal(loc=pose.position.x, scale=scale, size=num_particles)
            y_coords = np.random.normal(loc=pose.position.y, scale=scale, size=num_particles)
            
            _, _, current_heading = self.quaternion_to_euler(
                pose.orientation.x, pose.orientation.y,
                pose.orientation.z, pose.orientation.w)
            headings = np.random.normal(loc=current_heading, scale=0.05, size=num_particles)

            initial_weight = 1.0 / num_particles
            self._particles = []
            
            for x, y, heading in zip(x_coords, y_coords, headings):
                position = Point(x=x, y=y, z=0.0)
                orientation = self.euler_to_quaternion(heading)
                temp_pose = Pose(position=position, orientation=orientation)
                self._particles.append(ParticleEntity(temp_pose, initial_weight))
            
            self.get_logger().info(f'Created {len(self._particles)} particles')
            self._publish_particle_cloud()
            
        except Exception as e:
            self.get_logger().error(f'Error setting up particles: {str(e)}')

    def _publish_particle_cloud(self):
        """Publish particles for visualization"""
        try:
            if not self._particles:
                return
                
            cloud_msg = PoseArray()
            cloud_msg.header.frame_id = 'map'
            cloud_msg.header.stamp = self.get_clock().now().to_msg()
            
            for particle in self._particles:
                cloud_msg.poses.append(particle.pose)
                
            self._particle_cloud_pub.publish(cloud_msg)
            
        except Exception as e:
            self.get_logger().error(f'Error publishing particles: {str(e)}')

    def periodic_update(self):
        """Timer callback for publishing"""
        try:
            if self._particles:
                self._publish_particle_cloud()
                self.get_logger().info(f'Periodic update - particles: {len(self._particles)}', throttle_duration_sec=2.0)
                
                # Publish best estimate
                if self._particles:
                    best_particle = max(self._particles, key=lambda p: p.weight)
                    odom_msg = Odometry()
                    odom_msg.header.stamp = self.get_clock().now().to_msg()
                    odom_msg.header.frame_id = 'map'
                    odom_msg.child_frame_id = 'base_link'
                    odom_msg.pose.pose = best_particle.pose
                    self.pose_estimation_pub.publish(odom_msg)
                    
        except Exception as e:
            self.get_logger().error(f'Error in periodic update: {str(e)}')

def main(args=None):
    print("Starting Particle Filter Localizer...")
    rclpy.init(args=args)
    node = ParticleFilterLocalizer()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Shutting down...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
