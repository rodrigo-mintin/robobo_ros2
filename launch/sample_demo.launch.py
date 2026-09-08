from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
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

    robot_name_arg = DeclareLaunchArgument(
        'robot_name',
        default_value='0',
        description='Robot name for ROS namespace'
    )

    # Create the robobo_container node
    container_node = Node(
        package='robobo_ros2',
        executable='robobo_container',
        parameters=[
            {
                'ip': LaunchConfiguration('ip'),
                'robot_id': LaunchConfiguration('robot_id'),
                'robot_name': LaunchConfiguration('robot_name')
            }
        ],
        output='screen'
    )

    # Create the demo node
    demo_node = Node(
        package='robobo_ros2',
        executable='sample_demo',
        parameters=[
            {
                'ip': LaunchConfiguration('ip'),
                'robot_id': LaunchConfiguration('robot_id'),
                'robot_name': LaunchConfiguration('robot_name')
            }
        ],
        output='screen'
    )

    return LaunchDescription([
        ip_arg,
        robot_id_arg,
        robot_name_arg,
        container_node,
        demo_node
    ])