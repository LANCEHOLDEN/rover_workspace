#!/usr/bin/env python3
"""Launch file for waypoint follower node."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'waypoint_file',
            default_value='waypoints.yaml',
            description='Path to waypoints YAML file'
        ),
        DeclareLaunchArgument(
            'goal_tolerance',
            default_value='0.5',
            description='Distance tolerance to consider waypoint reached (meters)'
        ),
        DeclareLaunchArgument(
            'max_linear_vel',
            default_value='0.5',
            description='Maximum linear velocity (m/s)'
        ),
        DeclareLaunchArgument(
            'max_angular_vel',
            default_value='1.0',
            description='Maximum angular velocity (rad/s)'
        ),
        DeclareLaunchArgument(
            'loop',
            default_value='false',
            description='Loop through waypoints continuously'
        ),
        DeclareLaunchArgument(
            'reverse',
            default_value='false',
            description='Run waypoints in reverse order'
        ),

        Node(
            package='waypoint_reader',
            executable='waypoint_follower',
            name='waypoint_follower',
            output='screen',
            parameters=[{
                'waypoint_file': LaunchConfiguration('waypoint_file'),
                'goal_tolerance': LaunchConfiguration('goal_tolerance'),
                'max_linear_vel': LaunchConfiguration('max_linear_vel'),
                'max_angular_vel': LaunchConfiguration('max_angular_vel'),
                'loop': LaunchConfiguration('loop'),
                'reverse': LaunchConfiguration('reverse'),
            }]
        )
    ])
