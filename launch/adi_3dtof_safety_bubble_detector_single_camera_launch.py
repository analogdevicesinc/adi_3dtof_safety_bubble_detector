"""
Compact Launch File: Default Single Camera

This file has been refactored to use the unified launch system.
It calls the unified launch file with the default orientation preset.

Usage (unchanged):
    ros2 launch adi_3dtof_safety_bubble_detector adi_3dtof_safety_bubble_detector_single_camera_launch.py

For direct usage of the unified launch:
    ros2 launch adi_3dtof_safety_bubble_detector adi_3dtof_safety_bubble_detector_unified_single_camera_launch.py orientation:=default
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('adi_3dtof_safety_bubble_detector'),
                    'launch',
                    'adi_3dtof_safety_bubble_detector_unified_single_camera_launch.py'
                ])
            ]),
            launch_arguments={
                'orientation': 'default'
            }.items()
        )
    ])
