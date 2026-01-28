#!/usr/bin/env python3
"""
VLM Integration Launch File for Jackal Robot

This launch file starts the VLM integration node configured for Jackal simulation.

Usage:
    # Basic usage with default config
    ros2 launch social_mpc_nav vlm_integration_jackal.launch.py

    # With custom config file
    ros2 launch social_mpc_nav vlm_integration_jackal.launch.py \
        config_file:=/path/to/custom_config.yaml

    # With custom API endpoint
    ros2 launch social_mpc_nav vlm_integration_jackal.launch.py \
        vlm_api_url:=http://localhost:11434/v1/chat/completions
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Get package share directory
    pkg_share = FindPackageShare('social_mpc_nav')

    # Declare launch arguments
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=PathJoinSubstitution([
            pkg_share, 'config', 'vlm_integration_jackal.yaml'
        ]),
        description='Path to VLM integration configuration file'
    )

    enable_vlm_call_arg = DeclareLaunchArgument(
        'enable_vlm_call',
        default_value='true',
        description='Enable actual VLM API calls'
    )

    vlm_api_url_arg = DeclareLaunchArgument(
        'vlm_api_url',
        default_value='',
        description='VLM API URL (overrides config file if provided)'
    )

    # VLM Integration Node for Jackal
    vlm_integration_node = Node(
        package='social_mpc_nav',
        executable='vlm_integration_node',
        name='vlm_integration_node',
        parameters=[
            LaunchConfiguration('config_file'),
            {
                'enable_vlm_call': LaunchConfiguration('enable_vlm_call'),
            }
        ],
        output='screen',
        emulate_tty=True,
        prefix=['stdbuf -o L'],
    )

    return LaunchDescription([
        config_file_arg,
        enable_vlm_call_arg,
        vlm_api_url_arg,
        vlm_integration_node,
    ])
