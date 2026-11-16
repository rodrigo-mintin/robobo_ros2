#!/usr/bin/env python3
"""
Launch file for spinning up a pool of Robobo Container nodes.
Spawns N container nodes using standard sequential robot_id indexing,
where each robot_id maps automatically to its standard Robobo ports:
  - Robot i WebSocket:  40404 + (i * 10)
  - Robot i Camera:     40405 + (i * 10)
  - Namespace:          /robobo/robot_<i>/...

Modules are automatically tuned for headless simulation:
  - Excludes audio/speech/emotion modules (no physical audio/screen endpoints in headless mode).
  - Keeps simulated perception (aruco, blob, qr, object_recognition) which are calculated
    directly by RoboboSim without video streams.
  - Adds camera stream only if camera_stream:=true (when off-screen GPU camera rendering is active).

Usage:
  # Launch 4 nodes in pure headless mode (robot_id 0..3):
  ros2 launch robobo_ros2 robobo_pool.launch.py num_robots:=4

  # Launch 4 nodes with off-screen GPU camera streaming:
  ros2 launch robobo_ros2 robobo_pool.launch.py num_robots:=4 camera_stream:=true

  # Launch 2 nodes in standard GUI mode:
  ros2 launch robobo_ros2 robobo_pool.launch.py num_robots:=2 headless:=false
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    num_robots_str = LaunchConfiguration('num_robots').perform(context).strip()
    ip_str = LaunchConfiguration('ip').perform(context).strip()
    headless_str = LaunchConfiguration('headless').perform(context).strip().lower()
    camera_stream_str = LaunchConfiguration('camera_stream').perform(context).strip().lower()
    custom_modules_str = LaunchConfiguration('custom_modules').perform(context).strip()

    num_robots = int(num_robots_str) if num_robots_str else 4
    ip = ip_str if ip_str else '127.0.0.1'
    is_headless = headless_str in ('true', '1', 'yes')
    enable_camera = camera_stream_str in ('true', '1', 'yes')

    # Determine active modules
    if custom_modules_str:
        active_modules = [m.strip() for m in custom_modules_str.split(',') if m.strip()]
    else:
        # Standard sensors, simulation bridge, and simulated vision (calculated by sim without video stream)
        active_modules = [
            'imu', 'brightness', 'touch', 'sim',
            'aruco', 'blob', 'qr', 'object_recognition'
        ]

        # Multimedia modules are dropped in headless mode (no speaker/mic/display in headless runs)
        if not is_headless:
            active_modules += ['audio', 'speech', 'noise', 'emotion']

        # Camera streaming module is included only when explicitly enabled or in GUI mode
        if enable_camera or not is_headless:
            active_modules.append('camera')

    # Spawn container node for each robot
    nodes = []
    for i in range(num_robots):
        robot_name = str(i)

        node_params = {
            'robot_name': robot_name,
            'robot_id': i,
            'ip': ip,
            'modules': active_modules
        }

        container_node = Node(
            package='robobo_ros2',
            executable='robobo_container',
            name=f'robobo_container_{robot_name}',
            parameters=[node_params],
            output='screen'
        )
        nodes.append(container_node)

    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'num_robots',
            default_value='4',
            description='Number of Robobo instances/nodes to launch in parallel'
        ),
        DeclareLaunchArgument(
            'ip',
            default_value='127.0.0.1',
            description='IP address of the RoboboSim host machine (default: 127.0.0.1)'
        ),
        DeclareLaunchArgument(
            'headless',
            default_value='true',
            description='Whether the target simulation instances are running in headless mode'
        ),
        DeclareLaunchArgument(
            'camera_stream',
            default_value='false',
            description='Whether off-screen GPU camera streaming is enabled in headless mode'
        ),
        DeclareLaunchArgument(
            'custom_modules',
            default_value='',
            description='Optional comma-separated list of modules to override defaults (e.g. "imu,sim,aruco")'
        ),
        OpaqueFunction(function=launch_setup)
    ])
