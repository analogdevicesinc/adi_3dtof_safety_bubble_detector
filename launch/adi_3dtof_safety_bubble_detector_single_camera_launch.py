import os
import json
import launch
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution, TextSubstitution
from launch.substitutions import LaunchConfiguration, PythonExpression

# configuration file names
config_json_file_names = ['config_crosby_old_modes.json', 'config_crosby_adsd3500_new_modes.json']
config_ini_file_names = ['RawToDepthAdsd3500_qmp.ini', 'RawToDepthAdsd3500_lr-qnative.ini']

package_dir = get_package_share_directory('adi_3dtof_safety_bubble_detector') + "/../../../../src/adi_3dtof_safety_bubble_detector/"

# this function modifies the path of the ini file in configuration json file
def modify_ini_path_in_json_file(config_file_name_of_tof_sdk):
    with open(config_file_name_of_tof_sdk, 'r+') as file:
        data = json.load(file)

    for i in range(len(config_json_file_names)):
        if (config_file_name_of_tof_sdk.rsplit("/", 1)[1] == config_json_file_names[i]):
            modified_file_path = package_dir + "config/" + config_ini_file_names[i]
    
    data["DEPTH_INI"] = modified_file_path

    with open(config_file_name_of_tof_sdk, 'w') as file:
        json.dump(data, file, indent = 4) 


def generate_launch_description():

    # If 'arg_input_sensor_mode' default value is 3, then value of 'arg_input_file_name_or_ros_topic_prefix_name' and 'var_ns_prefix_cam1' should not be same.
    # Adjust the prefix based on the actual value
    var_ns_prefix_cam1 = "cam1"

    #Arguments : these values can be taken via command line
    # Set the camera height from ground
    arg_camera_height_from_ground_in_mtr_desc = DeclareLaunchArgument('arg_camera_height_from_ground_in_mtr', default_value="0.15")

    # Virtual camera height
    arg_virtual_camera_height_z_in_mtr_desc = DeclareLaunchArgument('arg_virtual_camera_height_z_in_mtr', default_value="5.0")

    # Safety bubble radius from the camera
    arg_safety_bubble_radius_in_mtr_desc = DeclareLaunchArgument('arg_safety_bubble_radius_in_mtr', default_value="1.0")

    # Shape of the safety bubble 0 = circular around camera, 1 = Square around camera
    arg_safety_bubble_shape_desc = DeclareLaunchArgument('arg_safety_bubble_shape', default_value="0")

    # Sensitivity of triggering the red flag in case of intrusion (1 highly sensitive, 100 less sensitive, default 10)
    arg_safety_bubble_detection_sensitivity_desc = DeclareLaunchArgument('arg_safety_bubble_detection_sensitivity', default_value="10")

    # Parameters related to input and output modes
    # Input mode
    # 0:Real Time Sensor
    # 2:Rosbag bin
    arg_input_sensor_mode_desc = DeclareLaunchArgument('arg_input_sensor_mode', default_value="0")

    # Output mode
    # 1:Creates ouput avi file
    arg_output_mode_desc = DeclareLaunchArgument('arg_output_mode', default_value="0")

    # Input mode argument
    # if "arg_input_mode" is 2 then "arg_input_file_name_or_ros_topic_prefix_name" represents the file name.
    # Ex:     arg_input_file_name_or_ros_topic_prefix_name_desc = DeclareLaunchArgument('arg_input_file_name_or_ros_topic_prefix_name', default_value=package_dir + "../adi_3dtof_input_video_files/adi_3dtof_height_170mm_yaw_135degrees_cam1.bin")
    # if "arg_input_mode" is 3 then "arg_input_file_name_or_ros_topic_prefix_name" represents the prefix of ros topics.
    # Ex:     arg_input_file_name_or_ros_topic_prefix_name_desc = DeclareLaunchArgument('arg_input_file_name_or_ros_topic_prefix_name', default_value=cam1")
    arg_input_file_name_or_ros_topic_prefix_name_desc = DeclareLaunchArgument('arg_input_file_name_or_ros_topic_prefix_name', default_value=package_dir + "../adi_3dtof_input_video_files/adi_3dtof_height_170mm_yaw_135degrees_cam1.bin")
    
    # Parameters related to ransac floor detection

    # Distance which determines how close the point must be to the RANSAC plane in order to be selected as inlier, default(25mm)
    arg_ransac_distance_threshold_mtr_desc = DeclareLaunchArgument('arg_ransac_distance_threshold_mtr', default_value="0.025")

    # Maximum number of RANSAC iterations which is allowed, default(10)
    arg_ransac_max_iterations_desc = DeclareLaunchArgument('arg_ransac_max_iterations', default_value="10")

    # Enables ransac Floor Detection, default(true)
    arg_enable_ransac_floor_detection_desc = DeclareLaunchArgument('arg_enable_ransac_floor_detection', default_value="true")

    # Parameters related to visualizations

    # Enables Floor paint, default(false)    
    arg_enable_floor_paint_desc = DeclareLaunchArgument('arg_enable_floor_paint', default_value="false")

    # Enables Visualization of safety bubble, default(true)
    arg_enable_safety_bubble_zone_visualization_desc = DeclareLaunchArgument('arg_enable_safety_bubble_zone_visualization', default_value="true")

    # Parameters related to image compressions

    # enables depth and IR compression
    arg_enable_depth_ir_compression_desc = DeclareLaunchArgument('arg_enable_depth_ir_compression', default_value="false")

    # enables output image compression
    arg_enable_output_image_compression_desc = DeclareLaunchArgument('arg_enable_output_image_compression', default_value="true")

    # Parameters related to input image quality

    # For Sensor serial number starting with CR and DV below default values works fine,
    # for sensor serial number starting with AM change the confidence threshold to 25

    # AB threshold value default(10)
    arg_ab_threshold_desc = DeclareLaunchArgument('arg_ab_threshold', default_value="10")

    # Confidence threshold value default(10)
    arg_confidence_threshold_desc = DeclareLaunchArgument('arg_confidence_threshold', default_value="10")

    # Configuration fie name of ToF SDK
    #    "config_crosby_old_modes.json" - Sensor serial number starting with CR/DV - config_json_file_names[0]
    #    "config_crosby_adsd3500_new_modes.json" - Sensor serial number starting with AM - config_json_file_names[1]    
    arg_config_file_name_of_tof_sdk_desc = DeclareLaunchArgument('arg_config_file_name_of_tof_sdk', default_value= package_dir + "config/" + config_json_file_names[0])
    # Frame Type
    #    "qmp" - Sensor serial number starting with CR/DV
    #    "lr-qnative" - Sensor serial number starting with AM
    arg_frame_type_desc = DeclareLaunchArgument('arg_frame_type', default_value="qmp")
 
    # Modifying the path to ini file in json file only if input sensor mode is camera
    if arg_input_sensor_mode_desc.default_value[0].text == '0':
        modify_ini_path_in_json_file(arg_config_file_name_of_tof_sdk_desc.default_value[0].text) 

    # Arguments for virtual camera transformation wrt map when virtual camera looking down(downward Z) 
    # NOTE: These values should not be changed
    var_virtual_camera_base_frame = "virtual_camera_frame"
    var_vcam_parent_frame = "map"
    var_vcam_pos_x = "0.0"
    var_vcam_pos_y = "0.0"
    var_vcam_pos_z = arg_virtual_camera_height_z_in_mtr_desc.default_value[0].text
    var_vcam_roll  = "3.1415927"
    var_vcam_pitch = "0.0"
    var_vcam_yaw   = "-1.5707"

    # Optical Frame: X-Right,Y-Down,Z-Front to Base Frame : X-Front,Y-Left,Z-Up conversion
    # Optical to Device rotation, this are standard values, would never change 
    var_cam_optical_to_base_roll = "-1.57"
    var_cam_optical_to_base_pitch = "0"
    var_cam_optical_to_base_yaw = "-1.57"

    # Parameters for Camera_1 Node
    var_cam1_base_frame_optical = f"{var_ns_prefix_cam1}_adtf31xx_optical"
    var_cam1_base_frame = f"{var_ns_prefix_cam1}_adtf31xx"
    #  map
    #   ^
    #   |
    # camera_device
    #   ^
    #   |
    # camera_optical
    var_cam1_parent_frame = "map"
    var_cam1_child_frame = var_cam1_base_frame_optical

    # Camera position wrt map
    var_cam1_pos_x = "0.0"
    var_cam1_pos_y = "0.0"
    var_cam1_pos_z = LaunchConfiguration('arg_camera_height_from_ground_in_mtr')
    # Side Tilt: Left side down:-ve  Right side down: +ve 
    var_cam1_roll = "0.0"
    # Vertical Tilt, down +ve radian, up -ve radian
    # Restriction on Pitch, Accepted Range (-1.5708, 1.5708)
    # If |Pitch| > 1.57, output from TF transform should be taken as cam_pitch = 3.14 - extracted pitch_angle
    var_cam1_pitch = "0.0"
    var_cam1_yaw = "0.0"

    # adi safety bubble detector node
    adi_3dtof_safety_bubble_detector_node_desc = Node(
                                        package='adi_3dtof_safety_bubble_detector',
                                        namespace=var_ns_prefix_cam1,
                                        executable='adi_3dtof_safety_bubble_detector_node',
                                        name='adi_3dtof_safety_bubble_detector_node',
                                        output="screen",
                                        parameters=[{
                                            'param_camera_link': var_cam1_base_frame,
                                            'param_optical_camera_link': var_cam1_base_frame_optical,
                                            'param_virtual_camera_link': var_virtual_camera_base_frame,
                                            'param_virtual_camera_height': LaunchConfiguration('arg_virtual_camera_height_z_in_mtr'),
                                            'param_safety_zone_radius_in_mtr': LaunchConfiguration('arg_safety_bubble_radius_in_mtr'),
                                            'param_safety_bubble_shape': LaunchConfiguration('arg_safety_bubble_shape'),
                                            'param_safety_bubble_sensitivity': LaunchConfiguration('arg_safety_bubble_detection_sensitivity'),
                                            'param_input_sensor_mode': LaunchConfiguration('arg_input_sensor_mode'),
                                            'param_output_sensor_mode': LaunchConfiguration('arg_output_mode'),
                                            'param_input_file_name_or_ros_topic_prefix_name': LaunchConfiguration('arg_input_file_name_or_ros_topic_prefix_name'),
                                            'param_ransac_distance_threshold_mtr': LaunchConfiguration('arg_ransac_distance_threshold_mtr'),
                                            'param_ransac_max_iterations': LaunchConfiguration('arg_ransac_max_iterations'),
                                            'param_enable_ransac_floor_detection': LaunchConfiguration('arg_enable_ransac_floor_detection'),
                                            'param_enable_floor_paint': LaunchConfiguration('arg_enable_floor_paint'),
                                            'param_enable_safety_bubble_zone_visualization': LaunchConfiguration('arg_enable_safety_bubble_zone_visualization'),
                                            'param_enable_depth_ir_compression': LaunchConfiguration('arg_enable_depth_ir_compression'),
                                            'param_enable_output_image_compression': LaunchConfiguration('arg_enable_output_image_compression'),
                                            'param_ab_threshold': LaunchConfiguration('arg_ab_threshold'),
                                            'param_confidence_threshold': LaunchConfiguration('arg_confidence_threshold'),
                                            'param_config_file_name_of_tof_sdk': LaunchConfiguration('arg_config_file_name_of_tof_sdk'),
                                            'param_frame_type': LaunchConfiguration('arg_frame_type')
                                        }],
                                        #prefix=['valgrind --tool=memcheck --leak-check=full --show-leak-kinds=all --track-origins=yes --verbose --log-file=valgrind.rpt'],
                                        #prefix=['valgrind --tool=memcheck --track-origins=yes --log-file=valgrind.rpt'],
                                        on_exit=launch.actions.Shutdown()
                                    )


    # Transform from base frame to optical
    cam1_base_to_optical_tf_desc = Node(
                                       package='tf2_ros',
                                       namespace=var_ns_prefix_cam1,
                                       executable='static_transform_publisher',
                                       name=f'{var_cam1_base_frame_optical}_tf',
                                       output="screen",
                                       arguments=["--x", "0", "--y", "0", "--z", "0", 
                                                  "--roll", f"{var_cam_optical_to_base_roll}", 
                                                  "--pitch", f"{var_cam_optical_to_base_pitch}", 
                                                  "--yaw", f"{var_cam_optical_to_base_yaw}", 
                                                  "--frame-id", f"{var_cam1_base_frame}",
                                                  "--child-frame-id", f"{var_cam1_child_frame}"]
                                   )

    # Transform from map to base
    cam1_map_to_base_tf_desc = Node(
                                   package='tf2_ros',
                                   namespace=var_ns_prefix_cam1,
                                   executable='static_transform_publisher',
                                   name=f'{var_cam1_base_frame}_tf',
                                   output="screen",
                                   arguments=["--x", f"{var_cam1_pos_x}", 
                                              "--y", f"{var_cam1_pos_y}", 
                                              "--z", PythonExpression(LaunchConfiguration('arg_camera_height_from_ground_in_mtr')), 
                                              "--roll", f"{var_cam1_roll}", 
                                              "--pitch", f"{var_cam1_pitch}", 
                                              "--yaw", f"{var_cam1_yaw}", 
                                              "--frame-id", f"{var_cam1_parent_frame}", 
                                              "--child-frame-id", f"{var_cam1_base_frame}"]
                               )

    # Transform from map to virtual camera
    cam1_map_to_vcam_tf_desc = Node(
                                   package='tf2_ros',
                                   namespace=var_ns_prefix_cam1,
                                   executable='static_transform_publisher',
                                   name=f'{var_virtual_camera_base_frame}_tf',
                                   output="screen",
                                   arguments=["--x", f"{var_vcam_pos_x}", 
                                              "--y", f"{var_vcam_pos_y}", 
                                              "--z", f"{var_vcam_pos_z}", 
                                              "--roll", f"{var_vcam_roll}", 
                                              "--pitch", f"{var_vcam_pitch}", 
                                              "--yaw", f"{var_vcam_yaw}", 
                                              "--frame-id", f"{var_vcam_parent_frame}", 
                                              "--child-frame-id", f"{var_virtual_camera_base_frame}"]
                               )

    # rviz node
    rviz_desc = Node(
                    package='rviz2',
                    namespace=var_ns_prefix_cam1,
                    executable='rviz2',
                    name='rviz2',
                    arguments=['-d', [PathJoinSubstitution(([
                                        FindPackageShare('adi_3dtof_safety_bubble_detector'),
                                        'rviz',
                                        'adi_3dtof_safety_bubble_detector.rviz'
                                        ]))]]
                    )

    return LaunchDescription([
        arg_camera_height_from_ground_in_mtr_desc,
        arg_virtual_camera_height_z_in_mtr_desc,
        arg_safety_bubble_radius_in_mtr_desc,
        arg_safety_bubble_shape_desc,
        arg_safety_bubble_detection_sensitivity_desc,
        arg_input_sensor_mode_desc,
        arg_output_mode_desc,
        arg_input_file_name_or_ros_topic_prefix_name_desc,
        arg_ransac_distance_threshold_mtr_desc,
        arg_ransac_max_iterations_desc,
        arg_enable_ransac_floor_detection_desc,
        arg_enable_floor_paint_desc,
        arg_enable_safety_bubble_zone_visualization_desc,
        arg_enable_depth_ir_compression_desc,
        arg_enable_output_image_compression_desc,
        arg_ab_threshold_desc,
        arg_confidence_threshold_desc,
        arg_config_file_name_of_tof_sdk_desc,
        arg_frame_type_desc,
        cam1_base_to_optical_tf_desc,
        cam1_map_to_base_tf_desc,
        cam1_map_to_vcam_tf_desc,
        adi_3dtof_safety_bubble_detector_node_desc
        #rviz_desc
    ])
