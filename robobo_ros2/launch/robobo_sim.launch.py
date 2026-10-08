#!/usr/bin/env python3
"""
Launch file for the RoboboSim Simulation Bridge node.
Publishes object locations/rotations/poses as topics and services.

Usage:
  # Launch with defaults (IP: 127.0.0.1, namespace: /robobo_sim)
  ros2 launch robobo_ros2 robobo_sim.launch.py

  # Launch with custom IP and publish frequency
  ros2 launch robobo_ros2 robobo_sim.launch.py ip:=192.168.1.50 publish_frequency:=20.0
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    ip_str = LaunchConfiguration('ip').perform(context).strip()
    sim_namespace_str = LaunchConfiguration('sim_namespace').perform(context).strip()
    publish_frequency_str = LaunchConfiguration('publish_frequency').perform(context).strip()
    publish_tf_str = LaunchConfiguration('publish_tf').perform(context).strip()
    world_frame_id_str = LaunchConfiguration('world_frame_id').perform(context).strip()

    node_params = {
        'ip': ip_str if ip_str else '127.0.0.1',
        'sim_namespace': sim_namespace_str if sim_namespace_str else '/robobo_sim',
        'publish_frequency': float(publish_frequency_str) if publish_frequency_str else 10.0,
        'publish_tf': publish_tf_str.lower() in ('true', '1') if publish_tf_str else True,
        'world_frame_id': world_frame_id_str if world_frame_id_str else 'world'
    }

    sim_node = Node(
        package='robobo_ros2',
        executable='robobo_sim_node',
        name='robobo_sim_node',
        parameters=[node_params],
        output='screen'
    )

    return [sim_node]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'ip',
            default_value='127.0.0.1',
            description='IP address of RoboboSim machine (default: 127.0.0.1)'
        ),
        DeclareLaunchArgument(
            'sim_namespace',
            default_value='/robobo_sim',
            description='ROS namespace for simulation topics and services (default: /robobo_sim)'
        ),
        DeclareLaunchArgument(
            'publish_frequency',
            default_value='10.0',
            description='Rate in Hz to publish SimObjectArray (default: 10.0)'
        ),
        DeclareLaunchArgument(
            'publish_tf',
            default_value='true',
            description='Whether to publish TF transforms for objects (default: true)'
        ),
        DeclareLaunchArgument(
            'world_frame_id',
            default_value='world',
            description='World frame ID for TF broadcasts (default: world)'
        ),
        OpaqueFunction(function=launch_setup)
    ])
