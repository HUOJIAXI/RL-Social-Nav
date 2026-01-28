#!/usr/bin/env python3
"""
Complete launch file for VLM-Enhanced Social MPC Navigation.

This launch file starts:
1. global_planner_node - Plans waypoint-based path to goal
2. gt_localization_node - Provides TF transforms (map->odom)
3. vlm_integration_node - VLM scene understanding
4. vlm_translator_node - Translates VLM outputs to MPC parameters
5. mpc_controller_vlm_node - VLM-enhanced MPC controller
   (subscribes directly to /person_tracker/person_info for people detection)

Usage:
    # Basic usage
    ros2 launch social_mpc_nav mpc_vlm_full.launch.py

    # With custom goal
    ros2 launch social_mpc_nav mpc_vlm_full.launch.py \
        goal_x:=-5.0 goal_y:=-15.0

    # With debug logging and custom VLM weights
    ros2 launch social_mpc_nav mpc_vlm_full.launch.py \
        goal_x:=-5.0 goal_y:=-15.0 \
        enable_debug_logging:=true \
        w_vlm_action:=3.0
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Get the package directory
    pkg_dir = get_package_share_directory('social_mpc_nav')

    # Default config file paths
    default_global_planner_config = os.path.join(
        pkg_dir, 'config', 'global_planner.yaml')
    # default_people_adapter_config = os.path.join(
    #     pkg_dir, 'config', 'people_adapter.yaml')  # No longer needed
    default_navigation_config = os.path.join(
        pkg_dir, 'config', 'navigation_params.yaml')
    default_vlm_integration_config = os.path.join(
        pkg_dir, 'config', 'vlm_integration_tiago_base.yaml')
    default_vlm_translator_config = os.path.join(
        pkg_dir, 'config', 'vlm_translator.yaml')

    # Declare launch arguments - Goal Settings
    goal_x_arg = DeclareLaunchArgument(
        'goal_x',
        default_value='-5.0',
        description='Goal X coordinate (meters)'
    )

    goal_y_arg = DeclareLaunchArgument(
        'goal_y',
        default_value='-15.0',
        description='Goal Y coordinate (meters)'
    )

    goal_tolerance_arg = DeclareLaunchArgument(
        'goal_tolerance',
        default_value='0.15',
        description='Distance threshold to consider goal reached (meters)'
    )

    waypoint_spacing_arg = DeclareLaunchArgument(
        'waypoint_spacing',
        default_value='1.0',
        description='Distance between waypoints in global path (meters)'
    )

    # Logging arguments
    log_directory_arg = DeclareLaunchArgument(
        'log_directory',
        default_value='~/ros2_logs/social_mpc_nav',
        description='Directory for CSV log files'
    )

    enable_debug_logging_arg = DeclareLaunchArgument(
        'enable_debug_logging',
        default_value='false',
        description='Enable detailed MPC debug logging'
    )

    # MPC tuning arguments
    num_rollouts_arg = DeclareLaunchArgument(
        'num_rollouts',
        default_value='100',
        description='Number of MPC rollouts (increase for crowds)'
    )

    N_arg = DeclareLaunchArgument(
        'N',
        default_value='15',
        description='MPC horizon steps'
    )

    w_obstacle_arg = DeclareLaunchArgument(
        'w_obstacle',
        default_value='3.0',
        description='Obstacle avoidance weight'
    )

    # VLM cost weight arguments
    w_vlm_directional_arg = DeclareLaunchArgument(
        'w_vlm_directional',
        default_value='1.0',
        description='VLM directional preference cost weight'
    )

    w_vlm_action_arg = DeclareLaunchArgument(
        'w_vlm_action',
        default_value='2.0',
        description='VLM action-based cost weight'
    )

    w_vlm_scene_arg = DeclareLaunchArgument(
        'w_vlm_scene',
        default_value='1.5',
        description='VLM scene-specific cost weight'
    )

    w_vlm_personal_arg = DeclareLaunchArgument(
        'w_vlm_personal',
        default_value='5.0',
        description='VLM personal distance violation cost weight'
    )

    # VLM integration arguments
    enable_vlm_api_arg = DeclareLaunchArgument(
        'enable_vlm_api',
        default_value='false',
        description='Enable actual VLM API calls (requires API key)'
    )

    # Launch configurations
    goal_x = LaunchConfiguration('goal_x')
    goal_y = LaunchConfiguration('goal_y')
    goal_tolerance = LaunchConfiguration('goal_tolerance')
    waypoint_spacing = LaunchConfiguration('waypoint_spacing')
    log_directory = LaunchConfiguration('log_directory')
    enable_debug_logging = LaunchConfiguration('enable_debug_logging')
    num_rollouts = LaunchConfiguration('num_rollouts')
    N = LaunchConfiguration('N')
    w_obstacle = LaunchConfiguration('w_obstacle')
    w_vlm_directional = LaunchConfiguration('w_vlm_directional')
    w_vlm_action = LaunchConfiguration('w_vlm_action')
    w_vlm_scene = LaunchConfiguration('w_vlm_scene')
    w_vlm_personal = LaunchConfiguration('w_vlm_personal')
    enable_vlm_api = LaunchConfiguration('enable_vlm_api')

    # Info message
    launch_info = LogInfo(
        msg=[
            '\n',
            '==================================================\n',
            'VLM-Enhanced Social MPC Navigation\n',
            '==================================================\n',
            'Goal: (', goal_x, ', ', goal_y, ') tolerance: ', goal_tolerance, ' m\n',
            'Waypoint spacing: ', waypoint_spacing, ' m\n',
            'MPC: N=', N, ' rollouts=', num_rollouts, '\n',
            'VLM weights: dir=', w_vlm_directional, ' action=', w_vlm_action,
            ' scene=', w_vlm_scene, ' personal=', w_vlm_personal, '\n',
            'VLM API enabled: ', enable_vlm_api, '\n',
            'Debug logging: ', enable_debug_logging, '\n',
            'Log directory: ', log_directory, '\n',
            '==================================================\n'
        ]
    )

    # Node 1: Global Planner
    # Plans waypoint-based path from start to goal
    global_planner_node = Node(
        package='social_mpc_nav',
        executable='global_planner_node',
        name='global_planner_node',
        output='screen',
        parameters=[
            default_global_planner_config,
            {
                'goal_x': goal_x,
                'goal_y': goal_y,
                'waypoint_spacing': waypoint_spacing,
                'odom_topic': '/task_generator_node/tiago_base/odom',
                'map_frame': 'map',
                'odom_frame': 'tiago_base/odom',
            }
        ],
        prefix=['stdbuf -o L'],
    )

    # Node 2: People Adapter - REMOVED
    # The people adapter is no longer needed as MPC controllers now subscribe
    # directly to /person_tracker/person_info from the 3D detector
    # people_adapter_node = Node(
    #     package='social_mpc_nav',
    #     executable='people_adapter_node',
    #     name='people_adapter_node',
    #     output='screen',
    #     parameters=[
    #         default_people_adapter_config,
    #         {
    #             'log_directory': log_directory,
    #         }
    #     ],
    #     prefix=['stdbuf -o L'],
    # )

    # Node 3: Ground-Truth Localization
    # Publishes map -> odom TF transform
    gt_localization_node = Node(
        package='social_mpc_nav',
        executable='gt_localization_node',
        name='gt_localization_node',
        output='screen',
        parameters=[
            {
                'robot_model_name': 'tiago_base',
                'map_frame': 'map',
                'odom_frame': 'tiago_base/odom',
                'base_link_frame': 'tiago_base/base_footprint',
                'world_frame': 'world',
                'model_states_topic': '/gazebo/model_states'
            }
        ],
        prefix=['stdbuf -o L'],
    )

    # Node 4: VLM Integration
    # Generates VLM responses with scene understanding
    vlm_integration_node = Node(
        package='social_mpc_nav',
        executable='vlm_integration_node',
        name='vlm_integration_node',
        output='screen',
        parameters=[
            default_vlm_integration_config,
            {
                'enable_vlm_api': enable_vlm_api,
                'odom_topic': '/task_generator_node/tiago_base/odom',
            }
        ],
        prefix=['stdbuf -o L'],
    )

    # Node 5: VLM Translator
    # Translates VLM responses to MPC parameters
    vlm_translator_node = Node(
        package='social_mpc_nav',
        executable='vlm_translator_node',
        name='vlm_translator_node',
        output='screen',
        parameters=[
            default_vlm_translator_config,
            {
                'odom_topic': '/task_generator_node/tiago_base/odom',
                'log_directory': log_directory,
            }
        ],
        prefix=['stdbuf -o L'],
    )

    # Node 6: VLM-Enhanced MPC Controller
    # Tracks waypoints with VLM-guided social navigation
    mpc_controller_vlm_node = Node(
        package='social_mpc_nav',
        executable='mpc_controller_vlm_node',
        name='mpc_controller_node',
        output='screen',
        parameters=[
            default_navigation_config,
            {
                'goal_x': goal_x,
                'goal_y': goal_y,
                'goal_tolerance': goal_tolerance,
                'log_directory': log_directory,
                'enable_debug_logging': enable_debug_logging,
                'num_rollouts': num_rollouts,
                'N': N,
                'w_obstacle': w_obstacle,
                'w_vlm_directional': w_vlm_directional,
                'w_vlm_action': w_vlm_action,
                'w_vlm_scene': w_vlm_scene,
                'w_vlm_personal': w_vlm_personal,
                'cmd_vel_topic': '/task_generator_node/tiago_base/cmd_vel',
                'odom_topic': '/task_generator_node/tiago_base/odom',
                'scan_topic': '/task_generator_node/tiago_base/lidar',
                'min_valid_laser_range': 0.5,
                'map_frame': 'map',
                'odom_frame': 'tiago_base/odom',
            }
        ],
        prefix=['stdbuf -o L'],
    )

    # Pedestrian Logger
    # Logs ground truth pedestrian movements from hunav (arena4) for social compliance analysis
    pedestrian_logger_node = Node(
        package='social_mpc_nav',
        executable='pedestrian_logger_node',
        name='pedestrian_logger_node',
        output='screen',
        parameters=[
            {
                'log_directory': log_directory,
                'human_states_topic': '/task_generator_node/human_states',
                'log_rate_hz': 10.0,
                'enable_logging': True,
            }
        ],
        prefix=['stdbuf -o L'],
    )

    return LaunchDescription([
        # Launch arguments - Goal
        goal_x_arg,
        goal_y_arg,
        goal_tolerance_arg,
        waypoint_spacing_arg,

        # Launch arguments - Logging
        log_directory_arg,
        enable_debug_logging_arg,

        # Launch arguments - MPC tuning
        num_rollouts_arg,
        N_arg,
        w_obstacle_arg,

        # Launch arguments - VLM weights
        w_vlm_directional_arg,
        w_vlm_action_arg,
        w_vlm_scene_arg,
        w_vlm_personal_arg,

        # Launch arguments - VLM integration
        enable_vlm_api_arg,

        # Info message
        launch_info,

        # Nodes
        global_planner_node,
        # people_adapter_node,  # Removed - MPC now subscribes to /person_tracker/person_info directly
        gt_localization_node,
        vlm_integration_node,
        vlm_translator_node,
        mpc_controller_vlm_node,
        pedestrian_logger_node,
    ])
