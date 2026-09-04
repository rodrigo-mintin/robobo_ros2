import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Get package directory
    pkg_dir = get_package_share_directory('robobo_ros2')
    
    # Declare launch arguments
    ip_arg = DeclareLaunchArgument(
        'ip',
        default_value='127.0.0.1',
        description='IP address of the Robobo robot'
    )
    
    robot_id_arg = DeclareLaunchArgument(
        'robot_id',
        default_value='0',
        description='Robot ID'
    )
    
    # Create the robobo_container node
    container_node = Node(
        package='robobo_ros2',
        executable='robobo_container.py',
        name='robobo_container',
        parameters=[
            {
                'ip': LaunchConfiguration('ip'),
                'robot_id': LaunchConfiguration('robot_id')
            }
        ],
        output='screen'
    )
    
    # Create the demo node
    demo_node = Node(
        package='robobo_ros2',
        executable='sample_demo_new.py',
        name='robobo_demo',
        parameters=[
            {
                'ip': LaunchConfiguration('ip'),
                'robot_id': LaunchConfiguration('robot_id')
            }
        ],
        output='screen'
    )
    
    return LaunchDescription([
        ip_arg,
        robot_id_arg,
        container_node,
        demo_node
    ])