#!/usr/bin/env python3
"""
Launch file to display the robotic reel mower URDF in RViz2.

This launch file starts:
- robot_state_publisher: Publishes robot transforms
- joint_state_publisher_gui: Provides GUI to control joints
- rviz2: Visualizes the robot model
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Generate the launch description for viewing the robot in RViz2."""
    
    # Get package share directory
    pkg_share = FindPackageShare('ros2_mower').find('ros2_mower')
    
    # Define paths
    urdf_path = PathJoinSubstitution([
        FindPackageShare('ros2_mower'),
        'urdf',
        'mower.urdf.xacro'
    ])
    
    # Launch arguments
    gui_arg = DeclareLaunchArgument(
        name='gui',
        default_value='true',
        description='Flag to enable joint_state_publisher_gui'
    )
    
    # Robot description parameter
    robot_description = ParameterValue(
        Command(['xacro ', urdf_path]),
        value_type=str
    )
    
    # Robot State Publisher node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description
        }]
    )
    
    # Joint State Publisher GUI node
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        condition=IfCondition(LaunchConfiguration('gui'))
    )
    
    # RViz2 node
    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', PathJoinSubstitution([
            FindPackageShare('ros2_mower'),
            'rviz',
            'mower_view.rviz'
        ])],
        condition=IfCondition(LaunchConfiguration('gui'))
    )
    
    return LaunchDescription([
        gui_arg,
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz2_node
    ])
