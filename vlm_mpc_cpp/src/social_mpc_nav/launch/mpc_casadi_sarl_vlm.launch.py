#!/usr/bin/env python3
"""
Complete launch file for CasADi-based VLM + SARL Social MPC Navigation.

This launch file starts the full interpretable social navigation stack using
CasADi/IPOPT optimization with SARL attention-weighted social costs:
1. global_planner_node          - Plans waypoint-based path to goal
2. gt_localization_node         - Provides TF transforms (map->odom)
3. sarl_bridge_node             - SARL attention weights + V(s) at 10 Hz
4. vlm_integration_node         - VLM scene understanding (with SARL-enriched prompts)
5. vlm_translator_node          - Translates VLM outputs to MPC parameters
6. mpc_controller_casadi_sarl_node - CasADi/IPOPT MPC with SARL attention + VLM
7. pedestrian_logger_node       - Logs ground-truth pedestrian movements
8. rviz2                        - Visualization with 2D Goal Pose tool

Usage:
    # Basic usage (set goals using RViz2 2D Goal Pose tool)
    ros2 launch social_mpc_nav mpc_casadi_sarl_vlm.launch.py

    # With VLM enabled and logging
    ros2 launch social_mpc_nav mpc_casadi_sarl_vlm.launch.py \\
        enable_vlm:=true \\
        log_mpc_to_csv:=true

    # With custom goal and SARL weights
    ros2 launch social_mpc_nav mpc_casadi_sarl_vlm.launch.py \\
        goal_x:=-5.0 goal_y:=-15.0 \\
        enable_vlm:=true \\
        w_sarl_attention:=1.5
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

    # Workspace root (vlm_mpc_cpp/) derived from installed package share dir
    workspace_dir = os.path.abspath(os.path.join(pkg_dir, '..', '..', '..', '..'))
    rviz_config_file = os.path.join(workspace_dir, 'rviz_config', 'mpc.rviz')

    # SARL model path (relative to repo root, one level above workspace)
    repo_dir = os.path.abspath(os.path.join(workspace_dir, '..'))
    default_sarl_model_path = os.path.join(
        repo_dir, 'CrowdNav', 'crowd_nav', 'data', 'output', 'rl_model.pth')

    # Default config file paths
    default_global_planner_config = os.path.join(
        pkg_dir, 'config', 'global_planner.yaml')
    default_mpc_casadi_sarl_config = os.path.join(
        pkg_dir, 'config', 'mpc_casadi_sarl.yaml')
    default_sarl_bridge_config = os.path.join(
        pkg_dir, 'config', 'sarl_bridge.yaml')
    default_vlm_integration_config = os.path.join(
        pkg_dir, 'config', 'vlm_integration_tiago_base.yaml')
    default_vlm_translator_config = os.path.join(
        pkg_dir, 'config', 'vlm_translator.yaml')

    # ========== Launch Arguments ==========

    # Goal settings
    goal_x_arg = DeclareLaunchArgument(
        'goal_x', default_value='0.0',
        description='Initial goal X coordinate (meters)')
    goal_y_arg = DeclareLaunchArgument(
        'goal_y', default_value='0.0',
        description='Initial goal Y coordinate (meters)')
    goal_tolerance_arg = DeclareLaunchArgument(
        'goal_tolerance', default_value='0.3',
        description='Distance threshold to consider goal reached (meters)')
    waypoint_spacing_arg = DeclareLaunchArgument(
        'waypoint_spacing', default_value='1.0',
        description='Distance between waypoints in global path (meters)')

    # Logging
    log_directory_arg = DeclareLaunchArgument(
        'log_directory', default_value='~/ros2_logs/social_mpc_nav',
        description='Directory for CSV log files')
    enable_debug_logging_arg = DeclareLaunchArgument(
        'enable_debug_logging', default_value='false',
        description='Enable detailed MPC debug logging')
    log_mpc_to_csv_arg = DeclareLaunchArgument(
        'log_mpc_to_csv', default_value='true',
        description='Enable MPC CSV logging')

    # MPC tuning
    N_arg = DeclareLaunchArgument(
        'N', default_value='15',
        description='MPC horizon steps')
    dt_arg = DeclareLaunchArgument(
        'dt', default_value='0.2',
        description='MPC time step (seconds)')
    w_obstacle_arg = DeclareLaunchArgument(
        'w_obstacle', default_value='3.0',
        description='Obstacle avoidance weight')

    # VLM cost weights
    w_vlm_directional_arg = DeclareLaunchArgument(
        'w_vlm_directional', default_value='1.0',
        description='VLM directional preference cost weight')
    w_vlm_action_arg = DeclareLaunchArgument(
        'w_vlm_action', default_value='2.0',
        description='VLM action-based cost weight')
    w_vlm_scene_arg = DeclareLaunchArgument(
        'w_vlm_scene', default_value='1.5',
        description='VLM scene-specific cost weight')
    w_vlm_personal_arg = DeclareLaunchArgument(
        'w_vlm_personal', default_value='5.0',
        description='VLM personal distance violation cost weight')

    # SARL cost weights
    w_sarl_attention_arg = DeclareLaunchArgument(
        'w_sarl_attention', default_value='1.0',
        description='SARL attention-weighted social cost weight')

    # SARL bridge
    sarl_model_path_arg = DeclareLaunchArgument(
        'sarl_model_path',
        default_value=default_sarl_model_path,
        description='Path to trained SARL model weights')
    sarl_rate_hz_arg = DeclareLaunchArgument(
        'sarl_rate_hz', default_value='10.0',
        description='SARL inference rate (Hz)')

    # VLM integration - MASTER CONTROL
    enable_vlm_arg = DeclareLaunchArgument(
        'enable_vlm', default_value='false',
        description='Master VLM control: enables VLM integration, API calls, warmup, and MPC modulation')

    # ========== Launch Configurations ==========
    goal_x = LaunchConfiguration('goal_x')
    goal_y = LaunchConfiguration('goal_y')
    goal_tolerance = LaunchConfiguration('goal_tolerance')
    waypoint_spacing = LaunchConfiguration('waypoint_spacing')
    log_directory = LaunchConfiguration('log_directory')
    enable_debug_logging = LaunchConfiguration('enable_debug_logging')
    log_mpc_to_csv = LaunchConfiguration('log_mpc_to_csv')
    N = LaunchConfiguration('N')
    dt = LaunchConfiguration('dt')
    w_obstacle = LaunchConfiguration('w_obstacle')
    w_vlm_directional = LaunchConfiguration('w_vlm_directional')
    w_vlm_action = LaunchConfiguration('w_vlm_action')
    w_vlm_scene = LaunchConfiguration('w_vlm_scene')
    w_vlm_personal = LaunchConfiguration('w_vlm_personal')
    w_sarl_attention = LaunchConfiguration('w_sarl_attention')
    sarl_model_path = LaunchConfiguration('sarl_model_path')
    sarl_rate_hz = LaunchConfiguration('sarl_rate_hz')
    enable_vlm = LaunchConfiguration('enable_vlm')

    # ========== Info Message ==========
    launch_info = LogInfo(
        msg=[
            '\n',
            '==================================================\n',
            'CasADi/IPOPT + SARL + VLM Social MPC Navigation\n',
            '==================================================\n',
            'Goal: (', goal_x, ', ', goal_y, ') tolerance: ', goal_tolerance, ' m\n',
            'Waypoint spacing: ', waypoint_spacing, ' m\n',
            'MPC: N=', N, ' dt=', dt, ' (CasADi/IPOPT optimization)\n',
            'VLM weights: dir=', w_vlm_directional, ' action=', w_vlm_action,
            ' scene=', w_vlm_scene, ' personal=', w_vlm_personal, '\n',
            'SARL weights: attention=', w_sarl_attention, '\n',
            'SARL model: ', sarl_model_path, ' @ ', sarl_rate_hz, ' Hz\n',
            'VLM enabled (master control): ', enable_vlm, '\n',
            'Debug logging: ', enable_debug_logging, '\n',
            'Log directory: ', log_directory, '\n',
            '==================================================\n'
        ]
    )

    # ========== Nodes ==========

    # Node 1: Global Planner
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

    # Node 2: Ground-Truth Localization (Physics-Based)
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

    # Node 3: SARL Bridge (Python)
    # Publishes attention weights + V(s) at 10 Hz
    # Publishes RViz attention markers on /sarl/attention_markers
    sarl_bridge_node = Node(
        package='social_mpc_nav',
        executable='sarl_bridge_node.py',
        name='sarl_bridge_node',
        output='screen',
        parameters=[
            default_sarl_bridge_config,
            {
                'model_path': sarl_model_path,
                'rate_hz': sarl_rate_hz,
                'goal_x': goal_x,
                'goal_y': goal_y,
                'odom_topic': '/task_generator_node/tiago_base/odom',
                'crowd_topic': '/person_tracker/person_info',
                'map_frame': 'map',
            }
        ],
        prefix=['stdbuf -o L'],
    )

    # Node 4: VLM Integration (with SARL-enriched prompts)
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
                'sarl_output_topic': '/sarl/output',
                'trigger_on_sarl_attention_shift': True,
                'sarl_attention_shift_threshold': 0.3,
            }
        ],
        prefix=['stdbuf -o L'],
    )

    # Node 5: VLM Translator
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

    # Node 6: CasADi+SARL MPC Controller
    # CasADi/IPOPT optimization with SARL attention-weighted social cost + VLM
    mpc_controller_casadi_sarl_node = Node(
        package='social_mpc_nav',
        executable='mpc_controller_casadi_sarl_node',
        name='mpc_controller_casadi_sarl_node',
        output='screen',
        parameters=[
            default_mpc_casadi_sarl_config,
            {
                'goal_x': goal_x,
                'goal_y': goal_y,
                'goal_tolerance': goal_tolerance,
                'log_directory': log_directory,
                'log_mpc_to_csv': log_mpc_to_csv,
                'enable_debug_logging': enable_debug_logging,
                'N': N,
                'dt': dt,
                'w_obstacle': w_obstacle,
                'w_vlm_directional': w_vlm_directional,
                'w_vlm_action': w_vlm_action,
                'w_vlm_scene': w_vlm_scene,
                'w_vlm_personal': w_vlm_personal,
                'w_sarl_attention': w_sarl_attention,
                'enable_vlm': enable_vlm,
                'cmd_vel_topic': '/task_generator_node/tiago_base/cmd_vel',
                'scan_topic': '/task_generator_node/tiago_base/lidar',
                'global_path_topic': '/global_path',
                'min_valid_laser_range': 0.5,
                'map_frame': 'map',
                'base_link_frame': 'tiago_base/base_footprint',
            }
        ],
        prefix=['stdbuf -o L'],
    )

    # Node 7: Pedestrian Logger
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
        log_mpc_to_csv_arg,

        # Launch arguments - MPC tuning
        N_arg,
        dt_arg,
        w_obstacle_arg,

        # Launch arguments - VLM weights
        w_vlm_directional_arg,
        w_vlm_action_arg,
        w_vlm_scene_arg,
        w_vlm_personal_arg,

        # Launch arguments - SARL weights
        w_sarl_attention_arg,

        # Launch arguments - SARL bridge
        sarl_model_path_arg,
        sarl_rate_hz_arg,

        # Launch arguments - VLM integration (master control)
        enable_vlm_arg,

        # Info message
        launch_info,

        # Nodes
        global_planner_node,
        gt_localization_node,
        sarl_bridge_node,
        vlm_integration_node,
        vlm_translator_node,
        mpc_controller_casadi_sarl_node,
        pedestrian_logger_node,
        rviz_node,
    ])
