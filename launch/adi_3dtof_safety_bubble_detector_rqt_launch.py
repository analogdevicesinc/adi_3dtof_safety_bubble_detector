import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():

    # This node launches rqt
    var_ns_prefix_cam1 = "cam1"
    rqt_gui_launch_desc = Node(
                              package='rqt_gui',
                              namespace=var_ns_prefix_cam1,
                              executable='rqt_gui',
                              name='rqt_gui_launch',
                              output="screen",
                              arguments=['--perspective-file', [PathJoinSubstitution(([
                                        FindPackageShare('adi_3dtof_safety_bubble_detector'),
                                        'rqt_config',
                                        'adi_3dtof_safety_bubble_detector_rqt.perspective'
                                        ]))]]
                          )

    return LaunchDescription([
        rqt_gui_launch_desc,
    ])
