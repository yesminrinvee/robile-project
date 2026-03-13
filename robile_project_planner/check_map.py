#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy

class MapChecker(Node):
    def __init__(self):
        super().__init__('map_checker')
        
        # Create QoS profile matching map_server (TRANSIENT_LOCAL durability)
        qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST
        )
        
        self.sub = self.create_subscription(
            OccupancyGrid, 
            '/map', 
            self.map_callback, 
            qos
        )
        self.get_logger().info('Waiting for map (with correct QoS)...')
        self.map_received = False
        
    def map_callback(self, msg):
        self.map_received = True
        self.get_logger().info('✓✓✓ MAP RECEIVED! ✓✓✓')
        self.get_logger().info(f'Width: {msg.info.width}')
        self.get_logger().info(f'Height: {msg.info.height}')
        self.get_logger().info(f'Resolution: {msg.info.resolution}')
        self.get_logger().info(f'Origin: ({msg.info.origin.position.x}, {msg.info.origin.position.y})')
        self.get_logger().info(f'Data size: {len(msg.data)} cells')
        
        # Print some sample values
        if len(msg.data) > 0:
            self.get_logger().info(f'First 10 cells: {msg.data[:10]}')
        
        # Shutdown after receiving map
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = MapChecker()
    
    # Wait up to 5 seconds for map
    for i in range(50):  # 5 seconds total
        rclpy.spin_once(node, timeout_sec=0.1)
        if node.map_received:
            break
    
    if not node.map_received:
        node.get_logger().error('no map received after 5 seconds!')
    
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()

if __name__ == '__main__':
    main()
