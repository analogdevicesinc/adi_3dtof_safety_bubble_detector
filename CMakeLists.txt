cmake_minimum_required(VERSION 3.5)
project(adi_3dtof_safety_bubble_detector)

#Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_geometry REQUIRED)
find_package(image_transport REQUIRED)
find_package(compressed_depth_image_transport REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions  REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)
find_package(visualization_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate custom messages and services
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ZoneStatus.msg"
  "msg/ZoneStatusArray.msg"
  "srv/ConfigureZone.srv"
  "srv/GetZoneConfig.srv"
  DEPENDENCIES std_msgs
)

# Get typesupport target for linking generated message types
# This is needed for both the main node and stitch_host_node
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} rosidl_typesupport_cpp)

set(SENSOR_CONNECTED TRUE CACHE BOOL "Indicates if the sensor is connected to the platform")

# add include files as per PCL version installed
if(${PCL_VERSION} VERSION_EQUAL 1.10.0)
  include_directories(include_floor_plane_detection/pcl_1_10_0)
elseif(${PCL_VERSION} VERSION_EQUAL 1.12.1)
  include_directories(include_floor_plane_detection/pcl_1_12_1)
endif()

include_directories(
  include
)

# Multi-zone detector library source files
SET(MULTI_ZONE_SRC_FILES src/multi_zone_detector.cpp)

#List of common source files.
SET( COMMON_SRC_FILES src/adi_3dtof_safety_bubble_detector_main.cpp src/adi_3dtof_safety_bubble_detector_algorithm.cpp src/adi_3dtof_safety_bubble_detector_application.cpp src/image_proc_utils.cpp src/output_sensor_file.cpp src/adi_3dtof_safety_bubble_detector_input_thread.cpp src/adi_3dtof_safety_bubble_detector_output_thread.cpp src/input_sensor_file_rosbagbin.cpp src/ros-perception/image_transport_plugins/compressed_depth_image_transport/rvl_codec.cpp ${MULTI_ZONE_SRC_FILES})

if(SENSOR_CONNECTED)
  # compile for platforms with 3D ToF Sensor
  find_package(aditof REQUIRED)
  add_compile_definitions(ENABLE_ADTF31XX_SENSOR)
  add_executable(${PROJECT_NAME}_node ${COMMON_SRC_FILES} src/input_sensor_adtf31xx.cpp)
  ament_target_dependencies(${PROJECT_NAME}_node rclcpp std_msgs sensor_msgs tf2 tf2_ros tf2_geometry_msgs pcl_ros pcl_conversions cv_bridge image_geometry image_transport compressed_depth_image_transport OpenCV aditof visualization_msgs)

  # Link generated message types to main node
  target_link_libraries(${PROJECT_NAME}_node "${cpp_typesupport_target}")



  if(${CMAKE_HOST_SYSTEM_PROCESSOR} MATCHES "aarch64|AARCH64")
    if(${PCL_VERSION} VERSION_EQUAL 1.10.0)
      set(FLOOR_DETECTOR_CORE_LIB ${PROJECT_SOURCE_DIR}/libs/libfloor_plane_detection.a)
    endif()
  elseif (${CMAKE_HOST_SYSTEM_PROCESSOR} MATCHES "x86_64|X86_64")
    if(${PCL_VERSION} VERSION_EQUAL 1.12.1)
      set(FLOOR_DETECTOR_CORE_LIB ${PROJECT_SOURCE_DIR}/libs_linux_x86/libfloor_plane_detection.a)
    endif()
  endif()

  ## Specify libraries to link a library or executable target against
  target_link_libraries(${PROJECT_NAME}_node
    ${Boost_SYSTEM_LIBRARY}
    ${OpenCV_LIBRARIES}
    ${FLOOR_DETECTOR_CORE_LIB}
  )

  ## Mark executables for installation
  install(TARGETS ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
  )
endif()

#This is the executable to be run on the Host to stitch the output images
if(BUILD_SBD_STITCH_HOST_NODE)
  add_executable(adi_3dtof_safety_bubble_detector_stitch_host_node src/adi_3dtof_safety_bubble_detector_stitch_host.cpp src/image_proc_utils.cpp ${MULTI_ZONE_SRC_FILES})

  ament_target_dependencies(adi_3dtof_safety_bubble_detector_stitch_host_node rclcpp std_msgs sensor_msgs tf2 tf2_ros tf2_geometry_msgs pcl_ros cv_bridge image_geometry image_transport compressed_depth_image_transport OpenCV visualization_msgs)

  target_link_libraries(adi_3dtof_safety_bubble_detector_stitch_host_node
     "${cpp_typesupport_target}"
     ${OpenCV_LIBRARIES}
     )

  install(TARGETS adi_3dtof_safety_bubble_detector_stitch_host_node
     DESTINATION lib/${PROJECT_NAME}
  )
endif()

install(DIRECTORY rviz 
DESTINATION share/${PROJECT_NAME})

install(DIRECTORY launch
DESTINATION share/${PROJECT_NAME})

install(DIRECTORY config 
DESTINATION share/${PROJECT_NAME})

install(DIRECTORY rqt_config
DESTINATION share/${PROJECT_NAME})

# Install message and service directories
install(DIRECTORY msg srv
  DESTINATION share/${PROJECT_NAME}
)

# Install scripts
install(PROGRAMS
  scripts/save_active_params.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
