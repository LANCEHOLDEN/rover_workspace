#!/usr/bin/env python3
"""
rviz.launch.py — Launch RViz for the rover.

Starts rviz2 with ins_rviz2.rviz config.
"""

from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node


def generate_launch_description():

    cleanup = ExecuteProcess(
        cmd=['bash', '-c', 'pkill -9 -f static_transform_publisher; pkill -9 -f rviz2; sleep 1'],
        output='screen',
        name='cleanup',
    )

    rviz = ExecuteProcess(
        cmd=['rviz2', '-d', '/home/lanceholden/rover_workspace/rover_odom.rviz'],
        output='screen',
        name='rviz2',
    )

    return LaunchDescription([
        cleanup,
        TimerAction(period=1.5, actions=[rviz]),
    ])
