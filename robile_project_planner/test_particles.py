#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PoseArray
from std_msgs.msg import Header

class ParticleTester(Node):
    def __init__(self):
        super().__init__('particle_tester')
        self.sub = self.create_subscription(
            PoseArray, '/particles', self.particle_callback, 10)
        self.pub = self.create_publisher(
            PoseStamped, '/initialpose', 10)
        self.get_logger().info('Particle tester started')
        
    def particle_callback(self, msg):
        self.get_logger().info(f'Received {len(msg.poses)} particles')
        
    def publish_initial_pose(self, x, y, theta):
        msg = PoseStamped()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.orientation.w = 1.0
        self.pub.publish(msg)
        self.get_logger().info(f'Published initial pose at ({x}, {y})')

def main(args=None):
    rclpy.init(args=args)
    node = ParticleTester()
    
    # Test publishing an initial pose
    node.publish_initial_pose(0.0, 0.0, 0.0)
    
    # Spin for a few seconds to receive particles
    for i in range(30):  # 3 seconds
        rclpy.spin_once(node, timeout_sec=0.1)
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
