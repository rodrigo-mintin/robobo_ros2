#!/usr/bin/env python3
"""
Launch file for the Robobo Container node.
Supports both command-line arguments and YAML parameter files.

Usage:
  # Launch with defaults (IP: 127.0.0.1, robot_name: '0', robot_id: 0)
  ros2 launch robobo_ros2 robobo.launch.py

  # Launch with custom connection arguments
  ros2 launch robobo_ros2 robobo.launch.py ip:=192.168.1.50 robot_name:=0 robot_id:=0

  # Launch with a YAML parameter configuration file
  ros2 launch robobo_ros2 robobo.launch.py params_file:=/path/to/params.yaml
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    ip_str = LaunchConfiguration('ip').perform(context).strip()
    robot_name_str = LaunchConfiguration('robot_name').perform(context).strip()
    robot_id_str = LaunchConfiguration('robot_id').perform(context).strip()
    params_file_str = LaunchConfiguration('params_file').perform(context).strip()

    node_params = []

    # 1. Load YAML parameter file if provided
    if params_file_str:
        if os.path.isfile(params_file_str):
            node_params.append(params_file_str)
        else:
            print(f"[WARN] Specified params_file does not exist: {params_file_str}")

    # 2. Build CLI overrides or default values
    cli_params = {}

    if params_file_str:
        # When params_file is supplied, only override parameters explicitly passed on CLI
        if ip_str:
            cli_params['ip'] = ip_str
        if robot_name_str:
            cli_params['robot_name'] = robot_name_str
        if robot_id_str:
            try:
                cli_params['robot_id'] = int(robot_id_str)
            except ValueError:
                cli_params['robot_id'] = robot_id_str
    else:
        # When no params_file is supplied, use CLI values or standard defaults
        cli_params['ip'] = ip_str if ip_str else '127.0.0.1'
        cli_params['robot_name'] = robot_name_str if robot_name_str else '0'
        try:
            cli_params['robot_id'] = int(robot_id_str) if robot_id_str else 0
        except ValueError:
            cli_params['robot_id'] = robot_id_str if robot_id_str else 0

    if cli_params:
        node_params.append(cli_params)

    container_node = Node(
        package='robobo_ros2',
        executable='robobo_container',
        parameters=node_params,
        output='screen'
    )

    return [container_node]


def generate_launch_description():
    ip_arg = DeclareLaunchArgument(
        'ip',
        default_value='',
        description='IP address of the Robobo robot (default: 127.0.0.1)'
    )

    robot_name_arg = DeclareLaunchArgument(
        'robot_name',
        default_value='',
        description='Robot name for ROS namespace (default: "0")'
    )

    robot_id_arg = DeclareLaunchArgument(
        'robot_id',
        default_value='',
        description='Robot index in RoboboSim (default: 0)'
    )

    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value='',
        description='Full path to YAML parameter configuration file (optional)'
    )

    return LaunchDescription([
        ip_arg,
        robot_name_arg,
        robot_id_arg,
        params_file_arg,
        OpaqueFunction(function=launch_setup)
    ])