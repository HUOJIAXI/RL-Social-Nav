#!/usr/bin/env python3
"""
Complete launch file for CasADi-based VLM-Enhanced Social MPC Navigation.

This launch file starts:
1. global_planner_node - Plans waypoint-based path to goal
2. gt_localization_node - Provides TF transforms (map->odom)
3. vlm_integration_node - VLM scene understanding
4. vlm_translator_node - Translates VLM outputs to MPC parameters
5. mpc_controller_casadi_node - CasADi optimization-based MPC with VLM integration
   (subscribes directly to /person_tracker/person_info for people detection)
6. rviz2 - Visualization with 2D Goal Pose tool for setting navigation goals

Usage:
    # Basic usage (set goals using RViz2 2D Goal Pose tool)
    ros2 launch social_mpc_nav mpc_casadi_full.launch.py

    # With custom MPC parameters
    ros2 launch social_mpc_nav mpc_casadi_full.launch.py \
        N:=20 \
        enable_debug_logging:=true \
        enable_vlm:=true

    # Optional: Set initial goal via launch parameters (can be overridden in RViz2)
    ros2 launch social_mpc_nav mpc_casadi_full.launch.py \
        goal_x:=-5.0 goal_y:=-15.0
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

    # RViz config file (relative path from workspace root)
    workspace_dir = os.path.abspath(os.path.join(pkg_dir, '..', '..', '..', '..'))
    rviz_config_file = os.path.join(workspace_dir, 'rviz_config', 'mpc.rviz')

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
    # Note: Goals are now set via RViz2 2D Goal Pose tool. These are fallback defaults.
    goal_x_arg = DeclareLaunchArgument(
        'goal_x',
        default_value='0.0',
        description='Initial goal X coordinate (meters) - Use RViz2 2D Goal Pose to set goals dynamically'
    )

    goal_y_arg = DeclareLaunchArgument(
        'goal_y',
        default_value='0.0',
        description='Initial goal Y coordinate (meters) - Use RViz2 2D Goal Pose to set goals dynamically'
    )

    goal_tolerance_arg = DeclareLaunchArgument(
        'goal_tolerance',
        default_value='0.3',
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

    # CasADi MPC tuning arguments
    N_arg = DeclareLaunchArgument(
        'N',
        default_value='15',
        description='MPC horizon steps'
    )

    dt_arg = DeclareLaunchArgument(
        'dt',
        default_value='0.2',
        description='MPC time step (seconds)'
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

    # VLM integration - MASTER CONTROL
    enable_vlm_arg = DeclareLaunchArgument(
        'enable_vlm',
        default_value='false',
        description='Master VLM control: enables VLM integration, API calls, warmup, and MPC modulation (requires API key when true)'
    )

    # Launch configurations
    goal_x = LaunchConfiguration('goal_x')
    goal_y = LaunchConfiguration('goal_y')
    goal_tolerance = LaunchConfiguration('goal_tolerance')
    waypoint_spacing = LaunchConfiguration('waypoint_spacing')
    log_directory = LaunchConfiguration('log_directory')
    enable_debug_logging = LaunchConfiguration('enable_debug_logging')
    N = LaunchConfiguration('N')
    dt = LaunchConfiguration('dt')
    w_obstacle = LaunchConfiguration('w_obstacle')
    w_vlm_directional = LaunchConfiguration('w_vlm_directional')
    w_vlm_action = LaunchConfiguration('w_vlm_action')
    w_vlm_scene = LaunchConfiguration('w_vlm_scene')
    w_vlm_personal = LaunchConfiguration('w_vlm_personal')
    enable_vlm = LaunchConfiguration('enable_vlm')

    # Info message
    launch_info = LogInfo(
        msg=[
            '\n',
            '==================================================\n',
            'CasADi-Based VLM-Enhanced Social MPC Navigation\n',
            '==================================================\n',
            'Goal: (', goal_x, ', ', goal_y, ') tolerance: ', goal_tolerance, ' m\n',
            'Waypoint spacing: ', waypoint_spacing, ' m\n',
            'MPC: N=', N, ' dt=', dt, ' (CasADi optimization)\n',
            'VLM integration in MPC: ENABLED\n',
            'VLM weights: dir=', w_vlm_directional, ' action=', w_vlm_action,
            ' scene=', w_vlm_scene, ' personal=', w_vlm_personal, '\n',
            'VLM enabled (master control): ', enable_vlm, '\n',
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

    # Node 3: Ground-Truth Localization (Physics-Based)
    # Uses Gazebo Transport to get physics-based robot pose
    # Publishes map -> odom transform (not map -> base_footprint)
    # Respects existing odom -> base_footprint from wheel odometry
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
                'gazebo_world_name': 'default'
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
                'enable_vlm_call': enable_vlm,
                'enable_vlm_heartbeat': enable_vlm,
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

    # Node 6: CasADi-Based MPC Controller
    # Optimization-based MPC with VLM-guided social navigation
    mpc_controller_casadi_node = Node(
        package='social_mpc_nav',
        executable='mpc_controller_casadi_node',
        name='mpc_controller_casadi_node',
        output='screen',
        parameters=[
            default_navigation_config,
            {
                'goal_x': goal_x,
                'goal_y': goal_y,
                'goal_tolerance': goal_tolerance,
                'log_directory': log_directory,
                'enable_debug_logging': enable_debug_logging,
                'N': N,
                'dt': dt,
                'w_obstacle': w_obstacle,
                'w_vlm_directional': w_vlm_directional,
                'w_vlm_action': w_vlm_action,
                'w_vlm_scene': w_vlm_scene,
                'w_vlm_personal': w_vlm_personal,
                'enable_vlm': enable_vlm,  # Master VLM control - enables VLM parameter modulation in MPC
                'cmd_vel_topic': '/task_generator_node/tiago_base/cmd_vel',
                'odom_topic': '/task_generator_node/tiago_base/odom',
                'scan_topic': '/task_generator_node/tiago_base/lidar',
                'global_path_topic': '/global_path',
                'min_valid_laser_range': 0.5,
                'map_frame': 'map',
                'odom_frame': 'tiago_base/odom',
            }
        ],
        prefix=['stdbuf -o L'],
    )

    # Node 7: Pedestrian Logger
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

    # Node 8: RViz2 Visualization
    # Visualize robot, map, laser scan, global path, and goals
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': True}],
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

        # Launch arguments - CasADi MPC tuning
        N_arg,
        dt_arg,
        w_obstacle_arg,

        # Launch arguments - VLM weights
        w_vlm_directional_arg,
        w_vlm_action_arg,
        w_vlm_scene_arg,
        w_vlm_personal_arg,

        # Launch arguments - VLM integration (master control)
        enable_vlm_arg,

        # Info message
        launch_info,

        # Nodes
        global_planner_node,
        # people_adapter_node,  # Removed - MPC now subscribes to /person_tracker/person_info directly
        gt_localization_node,
        vlm_integration_node,
        vlm_translator_node,
        mpc_controller_casadi_node,
        pedestrian_logger_node,
        rviz_node,
    ])
