#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node


def generate_launch_description():
    # Fix snap package conflicts that cause Gazebo GUI errors
    if 'GTK_PATH' in os.environ:
        del os.environ['GTK_PATH']
    if 'GIO_MODULE_DIR' in os.environ:
        del os.environ['GIO_MODULE_DIR']
    # Get the launch directory
    pkg_ros2_mower = get_package_share_directory('ros2_mower')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    
    # Paths
    world_file = PathJoinSubstitution([pkg_ros2_mower, 'worlds', 'empty.world'])
    urdf_file = PathJoinSubstitution([pkg_ros2_mower, 'urdf', 'mower.urdf.xacro'])
    controller_config = PathJoinSubstitution([pkg_ros2_mower, 'config', 'mower_controllers.yaml'])
    
    # Launch arguments
    world_arg = DeclareLaunchArgument(
        'world',
        default_value=world_file,
        description='Path to the Gazebo world file'
    )
    
    # Start Gazebo server and client
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py'])
        ),
        launch_arguments={
            'gz_args': ['-r ', LaunchConfiguration('world')],  # -r starts sim running
            'on_exit_shutdown': 'true'
        }.items()
    )
    
    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': Command(['xacro ', urdf_file]),
            'use_sim_time': True
        }]
    )
    
    # Spawn the robot
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'robotic_reel_mower',
            '-z', '0.2075'  # chassis_height/2 + wheel_radius = 0.11 + 0.0975
        ],
        output='screen'
    )

    # Joint State Broadcaster Spawner
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--switch-timeout', '10'],
        output='screen'
    )

    # Diff Drive Controller Spawner
    diff_drive_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diff_drive_controller', '--switch-timeout', '10'],
        output='screen'
    )

    # Reel Controller Spawner
    reel_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['reel_controller', '--switch-timeout', '10'],
        output='screen'
    )

    return LaunchDescription([
        world_arg,
        gazebo,
        robot_state_publisher,
        spawn_entity,
        # Delay controller spawning to allow robot to be fully loaded
        TimerAction(
            period=4.0,
            actions=[
                joint_state_broadcaster_spawner,
                diff_drive_controller_spawner,
                reel_controller_spawner
            ]
        )
    ])
