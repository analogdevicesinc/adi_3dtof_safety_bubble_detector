"""
Unified Single Camera Launch File with Orientation Configuration

This launch file replaces the multiple orientation-specific launch files with a single
configurable launcher that loads camera position and orientation from a YAML file.

Usage:
    # Launch with specific camera orientation preset
    ros2 launch adi_3dtof_safety_bubble_detector adi_3dtof_safety_bubble_detector_unified_single_camera_launch.py \
        orientation:=cam1_135deg
    
    # Launch with custom orientation file
    ros2 launch adi_3dtof_safety_bubble_detector adi_3dtof_safety_bubble_detector_unified_single_camera_launch.py \
        orientation:=cam3_0deg \
        params_file:=config/custom_params.yaml
    
    # Override camera height
    ros2 launch adi_3dtof_safety_bubble_detector adi_3dtof_safety_bubble_detector_unified_single_camera_launch.py \
        orientation:=cam2_67_5deg \
        camera_height:=0.2

Available orientations (from config/camera_orientations.yaml):
    - cam1_135deg      : 135° yaw (rear-left)
    - cam2_67_5deg     : 67.5° yaw (left)
    - cam3_0deg        : 0° yaw (front/center)
    - cam4_minus_67_5deg : -67.5° yaw (right)
    - default          : Center mounted, no rotation
"""

import os
import yaml
import launch
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration


def load_camera_orientation(context, *args, **kwargs):
    """
    Load camera orientation configuration from YAML file.
    This function is called during launch to read the orientation config.
    """
    pkg_share = get_package_share_directory('adi_3dtof_safety_bubble_detector')
    
    # Get launch arguments
    orientation_name = LaunchConfiguration('orientation').perform(context)
    orientations_file = LaunchConfiguration('orientations_file').perform(context)
    camera_height_override = LaunchConfiguration('camera_height').perform(context)
    params_file = LaunchConfiguration('params_file').perform(context)
    enable_rviz = LaunchConfiguration('enable_rviz').perform(context)
    
    # Get overridable input parameters (these override YAML values)
    input_sensor_mode = LaunchConfiguration('arg_input_sensor_mode').perform(context)
    input_sensor_ip = LaunchConfiguration('arg_input_sensor_ip').perform(context)
    input_file_name = LaunchConfiguration('arg_input_file_name').perform(context)
    camera_mode = LaunchConfiguration('arg_camera_mode').perform(context)
    
    # Load orientations from YAML
    with open(orientations_file, 'r') as f:
        config = yaml.safe_load(f)
    
    camera_orientations = config['camera_orientations']
    virtual_camera_config = config['virtual_camera']
    optical_transform = config['optical_to_base_transform']
    
    # Get the selected orientation
    if orientation_name not in camera_orientations:
        available = ', '.join(camera_orientations.keys())
        raise ValueError(
            f"Unknown orientation '{orientation_name}'. "
            f"Available orientations: {available}"
        )
    
    cam_config = camera_orientations[orientation_name]
    
    # Override height if specified
    cam_pos_z = camera_height_override if camera_height_override else str(cam_config['position']['z'])
    
    # Extract values
    namespace = cam_config['name']
    cam_pos_x = str(cam_config['position']['x'])
    cam_pos_y = str(cam_config['position']['y'])
    cam_roll = str(cam_config['orientation']['roll'])
    cam_pitch = str(cam_config['orientation']['pitch'])
    cam_yaw = str(cam_config['orientation']['yaw'])
    
    # Extract input file (for file-based input modes)
    input_file_name_from_config = cam_config.get('input_file', '')
    # Prepend the directory path to match the expected format
    if input_file_name_from_config:
        input_file_from_config = f"src/adi_3dtof_input_video_files/{input_file_name_from_config}"
    else:
        input_file_from_config = ''
    
    # Use launch argument if provided, otherwise use config file value
    final_input_file = input_file_name if input_file_name else input_file_from_config
    
    # Frame names
    cam_base_frame = f"{namespace}_adtf31xx"
    cam_base_frame_optical = f"{namespace}_adtf31xx_optical"
    virtual_camera_frame = virtual_camera_config['name']
    
    # Virtual camera configuration
    vcam_pos_x = str(virtual_camera_config['position']['x'])
    vcam_pos_y = str(virtual_camera_config['position']['y'])
    vcam_pos_z = str(virtual_camera_config['position']['z'])
    vcam_roll = str(virtual_camera_config['orientation']['roll'])
    vcam_pitch = str(virtual_camera_config['orientation']['pitch'])
    vcam_yaw = str(virtual_camera_config['orientation']['yaw'])
    
    # Optical to base transform
    opt_to_base_roll = str(optical_transform['roll'])
    opt_to_base_pitch = str(optical_transform['pitch'])
    opt_to_base_yaw = str(optical_transform['yaw'])
    
    # ========================================================================
    # Safety Bubble Detector Node
    # ========================================================================
    
    detector_node = Node(
        package='adi_3dtof_safety_bubble_detector',
        namespace=namespace,
        executable='adi_3dtof_safety_bubble_detector_node',
        name='adi_3dtof_safety_bubble_detector_node',
        output='screen',
        parameters=[
            params_file,
            {
                # Override frame names for this camera
                'param_camera_link': cam_base_frame,
                'param_optical_camera_link': cam_base_frame_optical,
                'param_virtual_camera_link': virtual_camera_frame,
                # Override input file from launch argument or camera orientation config
                'param_input_file_name_or_ros_topic_prefix_name': final_input_file,
                # Override input sensor mode from launch argument
                'param_input_sensor_mode': int(input_sensor_mode),
                # Override input sensor IP from launch argument
                'param_input_sensor_ip': input_sensor_ip,
                # Override camera mode from launch argument
                'param_camera_mode': int(camera_mode),
            }
        ],
        on_exit=launch.actions.Shutdown()
    )
    
    # ========================================================================
    # Transform: Base to Optical Frame
    # ========================================================================
    
    base_to_optical_tf = Node(
        package='tf2_ros',
        namespace=namespace,
        executable='static_transform_publisher',
        name=f'{cam_base_frame_optical}_tf',
        output='screen',
        arguments=[
            '--x', '0', '--y', '0', '--z', '0',
            '--roll', opt_to_base_roll,
            '--pitch', opt_to_base_pitch,
            '--yaw', opt_to_base_yaw,
            '--frame-id', cam_base_frame,
            '--child-frame-id', cam_base_frame_optical
        ]
    )
    
    # ========================================================================
    # Transform: Map to Camera Base Frame
    # ========================================================================
    
    map_to_base_tf = Node(
        package='tf2_ros',
        namespace=namespace,
        executable='static_transform_publisher',
        name=f'{cam_base_frame}_tf',
        output='screen',
        arguments=[
            '--x', cam_pos_x,
            '--y', cam_pos_y,
            '--z', cam_pos_z,
            '--roll', cam_roll,
            '--pitch', cam_pitch,
            '--yaw', cam_yaw,
            '--frame-id', 'map',
            '--child-frame-id', cam_base_frame
        ]
    )
    
    # ========================================================================
    # Transform: Map to Virtual Camera Frame
    # ========================================================================
    
    map_to_vcam_tf = Node(
        package='tf2_ros',
        namespace=namespace,
        executable='static_transform_publisher',
        name=f'{virtual_camera_frame}_tf',
        output='screen',
        arguments=[
            '--x', vcam_pos_x,
            '--y', vcam_pos_y,
            '--z', vcam_pos_z,
            '--roll', vcam_roll,
            '--pitch', vcam_pitch,
            '--yaw', vcam_yaw,
            '--frame-id', 'map',
            '--child-frame-id', virtual_camera_frame
        ]
    )
    
    # ========================================================================
    # RViz (Optional)
    # ========================================================================
    
    nodes = [detector_node, base_to_optical_tf, map_to_base_tf, map_to_vcam_tf]
    
    if enable_rviz.lower() == 'true':
        rviz_node = Node(
            package='rviz2',
            namespace=namespace,
            executable='rviz2',
            name='rviz2',
            arguments=[
                '-d',
                PathJoinSubstitution([
                    FindPackageShare('adi_3dtof_safety_bubble_detector'),
                    'rviz',
                    'adi_3dtof_safety_bubble_detector.rviz'
                ]).perform(context)
            ]
        )
        nodes.append(rviz_node)
    
    return nodes


def generate_launch_description():
    """
    Generate launch description with camera orientation configuration.
    """
    pkg_share = get_package_share_directory('adi_3dtof_safety_bubble_detector')
    
    # ========================================================================
    # Launch Arguments
    # ========================================================================
    
    arg_orientation = DeclareLaunchArgument(
        'orientation',
        default_value='cam3_0deg',
        description='Camera orientation preset name (cam1_135deg, cam2_67_5deg, cam3_0deg, cam4_minus_67_5deg, default)'
    )
    
    arg_orientations_file = DeclareLaunchArgument(
        'orientations_file',
        default_value=os.path.join(pkg_share, 'config', 'camera_orientations.yaml'),
        description='Path to camera orientations YAML file'
    )
    
    arg_params_file = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(pkg_share, 'config', 'adi_3dtof_safety_bubble_detector_params.yaml'),
        description='Path to node parameters YAML file'
    )
    
    arg_camera_height = DeclareLaunchArgument(
        'camera_height',
        default_value='',
        description='Camera height from ground in meters (overrides orientation config)'
    )
    
    arg_enable_rviz = DeclareLaunchArgument(
        'enable_rviz',
        default_value='false',
        description='Launch RViz for visualization'
    )
    
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
    
    # ========================================================================
    # Launch Description
    # ========================================================================
    
    return LaunchDescription([
        arg_orientation,
        arg_orientations_file,
        arg_params_file,
        arg_camera_height,
        arg_enable_rviz,
        arg_input_sensor_mode,
        arg_input_sensor_ip,
        arg_input_file_name,
        arg_camera_mode,
        OpaqueFunction(function=load_camera_orientation)
    ])
