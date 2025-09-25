import os
import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution, TextSubstitution
from launch.substitutions import LaunchConfiguration, PythonExpression

def generate_launch_description():
    
    var_ns_prefix = "adi_3dtof_safety_bubble_detector_stitch"


    arg_camera_prefixes_desc = DeclareLaunchArgument('arg_camera_prefixes', default_value="[cam1,cam2,cam3,cam4]")

    adi_3dtof_safety_bubble_detector_node_desc = Node(
                            package='adi_3dtof_safety_bubble_detector',
                            namespace=var_ns_prefix,
                            executable='adi_3dtof_safety_bubble_detector_stitch_host_node',
                            name='adi_3dtof_safety_bubble_detector_stitch_host_node',
                            output="screen",
                            parameters=[{
                                'param_camera_prefixes':  LaunchConfiguration('arg_camera_prefixes'),
                            }],
                            on_exit=launch.actions.Shutdown()
                        )
    
    rviz_desc = Node(
                    package='rviz2',
                    namespace=var_ns_prefix,
                    executable='rviz2',
                    name='rviz2',
                    arguments=['-d', [PathJoinSubstitution(([
                                        FindPackageShare('adi_3dtof_safety_bubble_detector'),
                                        'rviz',
                                        'adi_3dtof_safety_bubble_detector_stitch.rviz'
                                        ]))]]
                    )

    return LaunchDescription([
    arg_camera_prefixes_desc,
    adi_3dtof_safety_bubble_detector_node_desc,
    rviz_desc
    ])  

