#!/usr/bin/env python3
"""
Complete launch file for VLM + SARL Social MPC Navigation.

This launch file starts the full interpretable social navigation stack:
1. global_planner_node      - Plans waypoint-based path to goal
2. gt_localization_node     - Provides TF transforms (map->odom)
3. sarl_bridge_node         - SARL attention weights + V(s) at 10 Hz
4. vlm_integration_node     - VLM scene understanding (with SARL-enriched prompts)
5. vlm_translator_node      - Translates VLM outputs to MPC parameters
6. mpc_controller_sarl_node - SARL+VLM-enhanced MPC controller
7. pedestrian_logger_node   - Logs ground-truth pedestrian movements
8. rviz2                    - Visualization with 2D Goal Pose tool

Usage:
    # Basic usage (set goals using RViz2 2D Goal Pose tool)
    ros2 launch social_mpc_nav mpc_sarl_vlm.launch.py

    # With VLM enabled and logging
    ros2 launch social_mpc_nav mpc_sarl_vlm.launch.py \\
        enable_vlm:=true \\
        log_directory:="$VLM_TEST_LOG_DIR" \\
        log_mpc_to_csv:=true

    # With custom goal and SARL weights
    ros2 launch social_mpc_nav mpc_sarl_vlm.launch.py \\
        goal_x:=-5.0 goal_y:=-15.0 \\
        enable_vlm:=true \\
        w_sarl_attention:=1.5 w_sarl_terminal:=0.8
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
    default_mpc_sarl_config = os.path.join(
        pkg_dir, 'config', 'mpc_sarl.yaml')
    default_sarl_bridge_config = os.path.join(
        pkg_dir, 'config', 'sarl_bridge.yaml')
    default_vlm_integration_config = os.path.join(
        pkg_dir, 'config', 'vlm_integration_tiago_base.yaml')
    default_vlm_translator_config = os.path.join(
        pkg_dir, 'config', 'vlm_translator.yaml')

    # ========== Launch Arguments ==========

    # Goal settings
    # Note: Goals are now set via RViz2 2D Goal Pose tool. These are fallback defaults.
    goal_x_arg = DeclareLaunchArgument(
        'goal_x', default_value='0.0',
        description='Initial goal X coordinate (meters) - Use RViz2 2D Goal Pose to set goals dynamically')
    goal_y_arg = DeclareLaunchArgument(
        'goal_y', default_value='0.0',
        description='Initial goal Y coordinate (meters) - Use RViz2 2D Goal Pose to set goals dynamically')
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
    num_rollouts_arg = DeclareLaunchArgument(
        'num_rollouts', default_value='100',
        description='Number of MPC rollouts (increase for crowds)')
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
    w_sarl_terminal_arg = DeclareLaunchArgument(
        'w_sarl_terminal', default_value='0.5',
        description='SARL terminal value cost weight')

    # SARL bridge
    sarl_model_path_arg = DeclareLaunchArgument(
        'sarl_model_path',
        default_value='/home/car/RL/CrowdNav/crowd_nav/data/output/rl_model.pth',
        description='Path to trained SARL model weights')
    sarl_rate_hz_arg = DeclareLaunchArgument(
        'sarl_rate_hz', default_value='10.0',
        description='SARL inference rate (Hz)')

    # VLM integration - MASTER CONTROL
    enable_vlm_arg = DeclareLaunchArgument(
        'enable_vlm', default_value='false',
        description='Master VLM control: enables VLM integration, API calls, warmup, and MPC modulation (requires API key when true)')

    # ========== Launch Configurations ==========
    goal_x = LaunchConfiguration('goal_x')
    goal_y = LaunchConfiguration('goal_y')
    goal_tolerance = LaunchConfiguration('goal_tolerance')
    waypoint_spacing = LaunchConfiguration('waypoint_spacing')
    log_directory = LaunchConfiguration('log_directory')
    enable_debug_logging = LaunchConfiguration('enable_debug_logging')
    log_mpc_to_csv = LaunchConfiguration('log_mpc_to_csv')
    num_rollouts = LaunchConfiguration('num_rollouts')
    N = LaunchConfiguration('N')
    dt = LaunchConfiguration('dt')
    w_obstacle = LaunchConfiguration('w_obstacle')
    w_vlm_directional = LaunchConfiguration('w_vlm_directional')
    w_vlm_action = LaunchConfiguration('w_vlm_action')
    w_vlm_scene = LaunchConfiguration('w_vlm_scene')
    w_vlm_personal = LaunchConfiguration('w_vlm_personal')
    w_sarl_attention = LaunchConfiguration('w_sarl_attention')
    w_sarl_terminal = LaunchConfiguration('w_sarl_terminal')
    sarl_model_path = LaunchConfiguration('sarl_model_path')
    sarl_rate_hz = LaunchConfiguration('sarl_rate_hz')
    enable_vlm = LaunchConfiguration('enable_vlm')

    # ========== Info Message ==========
    launch_info = LogInfo(
        msg=[
            '\n',
            '==================================================\n',
            'VLM + SARL Social MPC Navigation\n',
            '==================================================\n',
            'Goal: (', goal_x, ', ', goal_y, ') tolerance: ', goal_tolerance, ' m\n',
            'Waypoint spacing: ', waypoint_spacing, ' m\n',
            'MPC: N=', N, ' dt=', dt, ' rollouts=', num_rollouts, '\n',
            'VLM weights: dir=', w_vlm_directional, ' action=', w_vlm_action,
            ' scene=', w_vlm_scene, ' personal=', w_vlm_personal, '\n',
            'SARL weights: attention=', w_sarl_attention,
            ' terminal=', w_sarl_terminal, '\n',
            'SARL model: ', sarl_model_path, ' @ ', sarl_rate_hz, ' Hz\n',
            'VLM enabled (master control): ', enable_vlm, '\n',
            'Debug logging: ', enable_debug_logging, '\n',
            'Log directory: ', log_directory, '\n',
            '==================================================\n'
        ]
    )

    # ========== Nodes ==========

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

    # Node 2: Ground-Truth Localization (Physics-Based)
    # Uses Gazebo Transport to get physics-based robot pose
    # Publishes map -> odom transform
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
    # Provides batch evaluation service for MPC terminal states
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
    # Subscribes to /sarl/output to inject attention scores into VLM prompts
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

    # Node 6: SARL+VLM MPC Controller
    # Combines SARL attention-weighted social cost with VLM scene-conditioned weights
    mpc_controller_sarl_node = Node(
        package='social_mpc_nav',
        executable='mpc_controller_sarl_node',
        name='mpc_controller_node',
        output='screen',
        parameters=[
            default_mpc_sarl_config,
            {
                'goal_x': goal_x,
                'goal_y': goal_y,
                'goal_tolerance': goal_tolerance,
                'log_directory': log_directory,
                'log_mpc_to_csv': log_mpc_to_csv,
                'enable_debug_logging': enable_debug_logging,
                'num_rollouts': num_rollouts,
                'N': N,
                'dt': dt,
                'w_obstacle': w_obstacle,
                'w_vlm_directional': w_vlm_directional,
                'w_vlm_action': w_vlm_action,
                'w_vlm_scene': w_vlm_scene,
                'w_vlm_personal': w_vlm_personal,
                'w_sarl_attention': w_sarl_attention,
                'w_sarl_terminal': w_sarl_terminal,
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
    # Logs ground truth pedestrian movements from hunav (arena4)
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
    # Visualize robot, map, laser scan, global path, goals, and SARL attention markers
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
        num_rollouts_arg,
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
        w_sarl_terminal_arg,

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
        mpc_controller_sarl_node,
        pedestrian_logger_node,
        rviz_node,
    ])
