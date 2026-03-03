"""
Compact Launch File: Camera 4 at -67.5° Yaw

This file has been refactored to use the unified launch system.
It calls the unified launch file with the cam4_minus_67_5deg orientation preset.

Usage (unchanged):
    ros2 launch adi_3dtof_safety_bubble_detector adi_3dtof_safety_bubble_detector_single_cam4_minus_67_5_deg_yaw_launch.py
    
    # With arguments:
    ros2 launch adi_3dtof_safety_bubble_detector adi_3dtof_safety_bubble_detector_single_cam4_minus_67_5_deg_yaw_launch.py \
        arg_input_sensor_mode:=3 arg_input_sensor_ip:="192.168.56.1"

For direct usage of the unified launch:
    ros2 launch adi_3dtof_safety_bubble_detector adi_3dtof_safety_bubble_detector_unified_single_camera_launch.py orientation:=cam4_minus_67_5deg
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare arguments that can be passed to this launch file
    arg_input_sensor_mode = DeclareLaunchArgument(
        'arg_input_sensor_mode',
        default_value='0',
        description='Input sensor mode: 0=Camera, 2=FileIO, 3=Network'
    )
    
    arg_input_sensor_ip = DeclareLaunchArgument(
        'arg_input_sensor_ip',
        default_value='192.168.56.1',
        description='IP address for network mode'
    )
    
    arg_input_file_name = DeclareLaunchArgument(
        'arg_input_file_name',
        default_value='',
        description='Input file name (overrides camera orientation config)'
    )
    
    arg_camera_mode = DeclareLaunchArgument(
        'arg_camera_mode',
        default_value='3',
        description='Camera mode: 0=qVGA, 1=VGA, 2=Short Range, 3=Long Range'
    )
    
    # Include the unified launch file and forward all arguments
    unified_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('adi_3dtof_safety_bubble_detector'),
                'launch',
                'adi_3dtof_safety_bubble_detector_unified_single_camera_launch.py'
            ])
        ]),
        launch_arguments={
            'orientation': 'cam4_minus_67_5deg',
            'arg_input_sensor_mode': LaunchConfiguration('arg_input_sensor_mode'),
            'arg_input_sensor_ip': LaunchConfiguration('arg_input_sensor_ip'),
            'arg_input_file_name': LaunchConfiguration('arg_input_file_name'),
            'arg_camera_mode': LaunchConfiguration('arg_camera_mode'),
        }.items()
    )
    
    return LaunchDescription([
        arg_input_sensor_mode,
        arg_input_sensor_ip,
        arg_input_file_name,
        arg_camera_mode,
        unified_launch
    ])


