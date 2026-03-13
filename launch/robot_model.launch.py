from launch import LaunchDescription
from launch_ros.actions import Node
import xacro
import os

def generate_launch_description():
    # Path to the robot description file (this is just a file, not hardware)
    xacro_file = '/home/yesmin/ros2_ws/src/robile_description/robots/robile3_config.urdf.xacro'
    
    # Convert xacro to URDF string
    robot_desc = xacro.process_file(xacro_file).toxml()
    
    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': robot_desc}]
        )
    ])
