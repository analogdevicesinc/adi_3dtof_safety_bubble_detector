#!/usr/bin/env python3
"""
Stitch Host Launch File - Multi-Camera Safety Bubble Detector Integration

PURPOSE:
    Launch file for the stitch host node that subscribes to multiple SBD nodes
    running on remote devices/robots and combines their outputs for visualization
    and decision-making.

SUPPORTED CONFIGURATIONS:
    - 1-4 cameras (single camera or multi-camera with time synchronization)
    - Compressed or uncompressed image transport
    - Configurable processing rate

USAGE EXAMPLES:
    # 4 cameras (default - 135°, 67.5°, 0°, -67.5° yaw) with RQT
    ros2 launch adi_3dtof_safety_bubble_detector \\
        adi_3dtof_safety_bubble_detector_host_multiple_cameras_launch.py

    # 2 cameras with compressed transport and 20Hz rate
    ros2 launch adi_3dtof_safety_bubble_detector \\
        adi_3dtof_safety_bubble_detector_host_multiple_cameras_launch.py \\
        arg_camera_prefixes:="[cam0,cam1]" \\
        arg_use_compressed:=true \\
        arg_update_rate_hz:=20.0

    # Single camera (no synchronization overhead), disable RQT
    ros2 launch adi_3dtof_safety_bubble_detector \\
        adi_3dtof_safety_bubble_detector_host_multiple_cameras_launch.py \\
        arg_camera_prefixes:="[cam0]" \\
        arg_launch_rqt:=false

PARAMETERS:
    - arg_camera_prefixes: List of camera namespace prefixes to subscribe to
                          Default: [cam1,cam2,cam3,cam4]
                          Examples: "[cam0]", "[cam0,cam1]", "[front,rear,left,right]"
    
    - arg_use_compressed: Use compressed image transport (reduces bandwidth by ~70-80%)
                         Default: true (recommended for WiFi/remote operation)
                         Set to false for local connections or when latency is critical
    
    - arg_update_rate_hz: Processing rate in Hz (affects timer period)
                         Default: 30.0 Hz
                         Typical range: 10-60 Hz
                         Higher rate = lower latency but more CPU usage
    
    - arg_launch_rqt: Launch RQT GUI with perspective configuration
                     Default: true
                     Set to false to disable RQT GUI if not needed

SUBSCRIBED TOPICS (per camera):
    - /<camera_prefix>/zone_status (ZoneStatusArray)
    - /<camera_prefix>/out_image (Image) or /out_image/compressed (CompressedImage)

PUBLISHED TOPICS:
    - /combo_safety_bubble/zone_status (ZoneStatusArray) - Combined zone detections
    - /combo_safety_bubble/zone_status_json (String) - Combined zone detections in JSON format (no dependencies)
    - /combo_safety_bubble/out_image (Image) - Stitched visualization

NETWORK CONSIDERATIONS:
    - Compressed transport recommended for WiFi: ~5-10 Mbps per camera
    - Uncompressed transport for wired: ~30-50 Mbps per camera
    - For 4 cameras over WiFi: Use compressed + 20-30 Hz rate

Author: Analog Devices, Inc.
Date: 2026-03-03
"""

import os
import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution, TextSubstitution
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition

def generate_launch_description():
    
    # Namespace for the stitch host node
    var_ns_prefix = "adi_3dtof_safety_bubble_detector_stitch"

    # ==========================================================================
    # LAUNCH ARGUMENTS - User-configurable parameters
    # ==========================================================================
    
    # Camera prefixes (1-4 cameras supported)
    # Each prefix corresponds to one SBD node's namespace
    arg_camera_prefixes_desc = DeclareLaunchArgument(
        'arg_camera_prefixes', 
        default_value="[cam1,cam2,cam3,cam4]",
        description='List of camera namespace prefixes (1-4 cameras). Examples: "[cam0]", "[cam0,cam1]", "[cam1,cam2,cam3,cam4]"'
    )
    
    # Compressed image transport (bandwidth optimization)
    arg_use_compressed_desc = DeclareLaunchArgument(
        'arg_use_compressed',
        default_value='true',
        description='Use compressed image transport (true) or raw images (false). Recommended: true for WiFi/remote.'
    )
    
    # Processing rate (Hz)
    arg_update_rate_hz_desc = DeclareLaunchArgument(
        'arg_update_rate_hz',
        default_value='30.0',
        description='Processing rate in Hz (10-60 typical). Higher = lower latency but more CPU. Default: 30.0'
    )
    
    # Launch RQT GUI (optional)
    arg_launch_rqt_desc = DeclareLaunchArgument(
        'arg_launch_rqt',
        default_value='true',
        description='Launch RQT GUI with perspective file (true/false). Default: true'
    )

    # ==========================================================================
    # STITCH HOST NODE - Main processing node
    # ==========================================================================
    adi_3dtof_safety_bubble_detector_node_desc = Node(
                            package='adi_3dtof_safety_bubble_detector',
                            namespace=var_ns_prefix,
                            executable='adi_3dtof_safety_bubble_detector_stitch_host_node',
                            name='adi_3dtof_safety_bubble_detector_stitch_host_node',
                            output="screen",
                            parameters=[{
                                'param_camera_prefixes':  LaunchConfiguration('arg_camera_prefixes'),
                                'param_use_compressed':   LaunchConfiguration('arg_use_compressed'),
                                'param_update_rate_hz':   LaunchConfiguration('arg_update_rate_hz'),
                            }],
                            on_exit=launch.actions.Shutdown()
                        )
    
    # ==========================================================================
    # RQT GUI - Optional Qt-based GUI with perspective file
    # ==========================================================================
    rqt_gui_desc = Node(
                    package='rqt_gui',
                    namespace=var_ns_prefix,
                    executable='rqt_gui',
                    name='rqt_gui',
                    output="screen",
                    arguments=['--perspective-file', [PathJoinSubstitution(([
                                        FindPackageShare('adi_3dtof_safety_bubble_detector'),
                                        'rqt_config',
                                        'adi_3dtof_safety_bubble_detector_rqt.perspective'
                                        ]))]], 
                    condition=IfCondition(LaunchConfiguration('arg_launch_rqt'))
                    )

    # ==========================================================================
    # LAUNCH DESCRIPTION - Assemble all components
    # ==========================================================================
    return LaunchDescription([
        # Launch arguments (user-configurable)
        arg_camera_prefixes_desc,
        arg_use_compressed_desc,
        arg_update_rate_hz_desc,
        arg_launch_rqt_desc,
        
        # Nodes
        adi_3dtof_safety_bubble_detector_node_desc,
        rqt_gui_desc
    ])  

