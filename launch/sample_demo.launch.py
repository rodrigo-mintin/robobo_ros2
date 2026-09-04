#!/usr/bin/env python3
"""
Sample launch file for Robobo demonstration.
"""

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Create a launch description
    ld = LaunchDescription()
    
    # Launch the demo node
    demo_node = Node(
        package='robobo_ros2',
        executable='sample_demo.py',
        name='robobo_demo',
        output='screen'
    )
    
    ld.add_action(demo_node)
    
    return ld