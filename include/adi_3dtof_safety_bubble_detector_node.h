/******************************************************************************
Copyright (c), 2023 - Analog Devices Inc. All Rights Reserved.
This software is PROPRIETARY & CONFIDENTIAL to Analog Devices, Inc.
and its licensors.
******************************************************************************/
#ifndef ADI_3DTOF_SAFETY_BUBBLE_DETECTOR_NODE_H
#define ADI_3DTOF_SAFETY_BUBBLE_DETECTOR_NODE_H

#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/msg/transform_stamped.h>
#include <sensor_msgs/msg/image.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <Eigen/Geometry>
#include <std_msgs/msg/bool.hpp>
#include <utility>

#include "adi_3dtof_safety_bubble_detector_output_info.h"
#include "adtf31xx_sensor_frame_info.h"
#include "floor_plane_detection.h"
#include "image_proc_utils.h"
#include "input_sensor.h"
#include "input_sensor_factory.h"
#include "output_sensor.h"
#include "output_sensor_factory.h"
#include "rclcpp/rclcpp.hpp"
#include <image_geometry/pinhole_camera_model.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <sensor_msgs/distortion_models.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
//#include <pcl_ros/point_cloud.hpp>
#include "ros-perception/image_transport_plugins/compressed_depth_image_transport/compression_common.h"
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <chrono>
#include <mutex>
#include <queue>

using namespace std::chrono_literals;
namespace enc = sensor_msgs::image_encodings;

/**
 * @brief This is main class for this package
 *
 */
class ADI3DToFSafetyBubbleDetector : public rclcpp::Node
{
public:
  /**
   * @brief Construct a new ADI3DToFSafetyBubbleDetector object
   *
   */
  ADI3DToFSafetyBubbleDetector() : Node("adi_3dtof_safety_bubble_detector_node")
  {
    // Get Parameters
    rcl_interfaces::msg::ParameterDescriptor policy_x_description{};
    policy_x_description.read_only = true;

    // ToF camera link
    rcl_interfaces::msg::ParameterDescriptor base_camera_link_descriptor{};
    base_camera_link_descriptor.read_only = true;
    base_camera_link_descriptor.description = "Base camera frame name";
    this->declare_parameter<std::string>(
      "param_camera_link", "adi_camera_link_def", base_camera_link_descriptor);

    // Optical camera link
    rcl_interfaces::msg::ParameterDescriptor optical_camera_link_descriptor{};
    optical_camera_link_descriptor.read_only = true;
    optical_camera_link_descriptor.description = "Optical camera frame name";
    this->declare_parameter<std::string>(
      "param_optical_camera_link", "optical_camera_link", optical_camera_link_descriptor);

    // Virtual camera link
    rcl_interfaces::msg::ParameterDescriptor virtual_camera_link_descriptor{};
    virtual_camera_link_descriptor.read_only = true;
    virtual_camera_link_descriptor.description = "Virtual camera frame name";
    this->declare_parameter<std::string>(
      "param_virtual_camera_link", "virtual_camera_link", virtual_camera_link_descriptor);

    // Camera height
    rcl_interfaces::msg::ParameterDescriptor virtual_camera_height_descriptor{};
    virtual_camera_height_descriptor.read_only = true;
    virtual_camera_height_descriptor.description =
      "Height of the over head camera when finding top view image";
    this->declare_parameter<float>(
      "param_virtual_camera_height", 5.0f, virtual_camera_height_descriptor);

    // Input sensor Mode:Camera/File/ROSBag
    rcl_interfaces::msg::ParameterDescriptor input_sensor_mode_descriptor{};
    input_sensor_mode_descriptor.read_only = true;
    input_sensor_mode_descriptor.description = "Input mode: 0:Camera, 2:FileIO";
    this->declare_parameter<int>("param_input_sensor_mode", 0, input_sensor_mode_descriptor);

    // Output sensor Mode:video,csv
    rcl_interfaces::msg::ParameterDescriptor output_sensor_mode_descriptor{};
    output_sensor_mode_descriptor.read_only = true;
    output_sensor_mode_descriptor.description =
      "Output mode: 0:no output is stored, 1:avi and csv file generated";
    this->declare_parameter<int>("param_output_sensor_mode", 0, output_sensor_mode_descriptor);

    // input file name in FileIO mode or rostopic prefix name
    rcl_interfaces::msg::ParameterDescriptor input_file_name_or_ros_topic_prefix_descriptor{};
    input_file_name_or_ros_topic_prefix_descriptor.read_only = true;
    input_file_name_or_ros_topic_prefix_descriptor.description =
      "Input file name in FileIO mode or rostopic prefix name";
    this->declare_parameter<std::string>(
      "param_input_file_name_or_ros_topic_prefix_name", "no name",
      input_file_name_or_ros_topic_prefix_descriptor);

    // enable option to publish depth and ir compressed image
    rcl_interfaces::msg::ParameterDescriptor enable_depth_ab_compressed_image_descriptor{};
    enable_depth_ab_compressed_image_descriptor.read_only = true;
    enable_depth_ab_compressed_image_descriptor.description =
      "Unchecked: Publishes uncompressed images, Checked: Publishes compressed images";
    this->declare_parameter<bool>(
      "param_enable_depth_ab_compression", false, enable_depth_ab_compressed_image_descriptor);

    // enable option to publish compressed output image.
    rcl_interfaces::msg::ParameterDescriptor enable_output_compressed_image_descriptor{};
    enable_output_compressed_image_descriptor.read_only = true;
    enable_output_compressed_image_descriptor.description =
      "Unchecked: Publishes uncompressed output image, Checked: Publishes compressed output image";
    this->declare_parameter<bool>(
      "param_enable_output_image_compression", true, enable_output_compressed_image_descriptor);

    // Do not change the default values of the below 3 parameters
    // image width
    rcl_interfaces::msg::ParameterDescriptor image_width_descriptor{};
    image_width_descriptor.read_only = true;
    image_width_descriptor.description = "Width of image in pixels";
    this->declare_parameter<int>("param_input_image_width", 1024, image_width_descriptor);

    // image height
    rcl_interfaces::msg::ParameterDescriptor image_height_descriptor{};
    image_height_descriptor.read_only = true;
    image_height_descriptor.description = "Height of image in pixels";
    this->declare_parameter<int>("param_input_image_height", 1024, image_height_descriptor);

    // scale factor to determine some algorithm parameters
    rcl_interfaces::msg::ParameterDescriptor processing_scale_descriptor{};
    processing_scale_descriptor.read_only = true;
    processing_scale_descriptor.description = "processing scale on image";
    this->declare_parameter<int>("param_processing_scale", 2, processing_scale_descriptor);

    // the height of the floor ransac finds
    rcl_interfaces::msg::ParameterDescriptor height_of_the_ransac_floor_descriptor{};
    height_of_the_ransac_floor_descriptor.read_only = true;
    height_of_the_ransac_floor_descriptor.description = "Height of the ransac floor in meters";
    this->declare_parameter<float>(
      "param_ransac_distance_threshold_mtr", 0.025f, height_of_the_ransac_floor_descriptor);

    // maxium iterations ransac is allowed
    rcl_interfaces::msg::ParameterDescriptor max_ransac_iteration_allowed_descriptor{};
    max_ransac_iteration_allowed_descriptor.read_only = true;
    max_ransac_iteration_allowed_descriptor.description = "Maximum iterations in ransac allowed";
    this->declare_parameter<int>(
      "param_ransac_max_iterations", 10, max_ransac_iteration_allowed_descriptor);

    // path of the config file to read from tof sdk
    rcl_interfaces::msg::ParameterDescriptor path_of_the_config_file_descriptor{};
    path_of_the_config_file_descriptor.read_only = true;
    path_of_the_config_file_descriptor.description = "Path of the configuaration files";
    this->declare_parameter<std::string>(
      "param_config_file_name_of_tof_sdk", "no name", path_of_the_config_file_descriptor);
	  
    // Camera mode for the TOF sensor
    rcl_interfaces::msg::ParameterDescriptor camera_mode_descriptor{};
    camera_mode_descriptor.read_only = true;
    camera_mode_descriptor.description = "Camera Mode";
    this->declare_parameter<int>("param_camera_mode", 3, camera_mode_descriptor);

    // ip address of the sensor
    rcl_interfaces::msg::ParameterDescriptor ip_address_of_sensor_descriptor{};
    ip_address_of_sensor_descriptor.read_only = true;
    ip_address_of_sensor_descriptor.description = "IP address of the sensor";
    this->declare_parameter<std::string>(
      "param_input_sensor_ip", "no name", ip_address_of_sensor_descriptor);	
    
    // Safety zone radius
    rcl_interfaces::msg::ParameterDescriptor safety_zone_radius_descriptor{};
    rcl_interfaces::msg::FloatingPointRange safety_zone_radius_range;
    safety_zone_radius_range.set__from_value(0.1).set__to_value(3);
    safety_zone_radius_descriptor.floating_point_range = {safety_zone_radius_range};
    safety_zone_radius_descriptor.description = "Safety bubble radius in meters";
    this->declare_parameter<float>(
      "param_safety_zone_radius_in_mtr", 1.0f, safety_zone_radius_descriptor);

    // Shape of safety bubble. 0 = circular, 1 = square
    rcl_interfaces::msg::ParameterDescriptor safety_bubble_shape_descriptor{};
    safety_bubble_shape_descriptor.description = "0 : Circular, 1 : Square";
    this->declare_parameter<int>("param_safety_bubble_shape", 0, safety_bubble_shape_descriptor);

    // enable option to run ransac floor detection algortihm
    rcl_interfaces::msg::ParameterDescriptor enable_ransac_floor_detection_descriptor{};
    enable_ransac_floor_detection_descriptor.description =
      "Checked: enables ransac floor detection algorithm, Unchecked: disables ransac floor "
      "detection algorithm";
    this->declare_parameter<bool>(
      "param_enable_ransac_floor_detection", true, enable_ransac_floor_detection_descriptor);

    // number of connected pixels to trigger object detection
    rcl_interfaces::msg::ParameterDescriptor safety_bubble_sensitivity_descriptor{};
    rcl_interfaces::msg::IntegerRange safety_bubble_sensitivity_range;
    safety_bubble_sensitivity_range.set__from_value(1).set__to_value(50);
    safety_bubble_sensitivity_descriptor.integer_range = {safety_bubble_sensitivity_range};
    safety_bubble_sensitivity_descriptor.description =
      "total number of connected pixels in an image to detect as a object";
    this->declare_parameter<int>(
      "param_safety_bubble_sensitivity", 10, safety_bubble_sensitivity_descriptor);

    // enable option to visualize for floor paint
    rcl_interfaces::msg::ParameterDescriptor enable_floor_paint_descriptor{};
    enable_floor_paint_descriptor.description =
      "Checked: enables floor paint, Unchecked: disables floor paint";
    this->declare_parameter<bool>("param_enable_floor_paint", false, enable_floor_paint_descriptor);

    // enable option to visualize safety bubble zone
    rcl_interfaces::msg::ParameterDescriptor enable_safety_bubble_zone_visualization_descriptor{};
    enable_safety_bubble_zone_visualization_descriptor.description =
      "Checked: enables safety bubble zone visualization, Unchecked: disables safety bubble zone "
      "visualization";
    this->declare_parameter<bool>(
      "param_enable_safety_bubble_zone_visualization", true,
      enable_safety_bubble_zone_visualization_descriptor);

    // ab threshold to be set to device
    rcl_interfaces::msg::ParameterDescriptor ab_threshold_descriptor{};
    rcl_interfaces::msg::IntegerRange ab_threshold_range;
    ab_threshold_range.set__from_value(1).set__to_value(255);
    ab_threshold_descriptor.integer_range = {ab_threshold_range};
    ab_threshold_descriptor.description = "Set ab threshold value to device";
    this->declare_parameter<int>("param_ab_threshold", 10, ab_threshold_descriptor);

    // confidence threshold to be set to device
    rcl_interfaces::msg::ParameterDescriptor confidence_threshold_descriptor{};
    rcl_interfaces::msg::IntegerRange confidence_threshold_range;
    confidence_threshold_range.set__from_value(1).set__to_value(255);
    confidence_threshold_descriptor.integer_range = {confidence_threshold_range};
    confidence_threshold_descriptor.description = "Set confidence threshold value to device";
    this->declare_parameter<int>("param_confidence_threshold", 10, confidence_threshold_descriptor);

    camera_link_ =
      this->get_parameter("param_camera_link").get_parameter_value().get<std::string>();
    optical_camera_link_ =
      this->get_parameter("param_optical_camera_link").get_parameter_value().get<std::string>();
    virtual_camera_link_ =
      this->get_parameter("param_virtual_camera_link").get_parameter_value().get<std::string>();
    safety_zone_radius_mtr_ =
      this->get_parameter("param_safety_zone_radius_in_mtr").get_parameter_value().get<float>();
    safety_bubble_shape_ =
      this->get_parameter("param_safety_bubble_shape").get_parameter_value().get<int>();
    virtual_camera_height_mtr_ =
      this->get_parameter("param_virtual_camera_height").get_parameter_value().get<float>();
    input_sensor_mode_ =
      this->get_parameter("param_input_sensor_mode").get_parameter_value().get<int>();
    output_sensor_mode_ =
      this->get_parameter("param_output_sensor_mode").get_parameter_value().get<int>();
    input_file_name_or_ros_topic_prefix_name_ =
      this->get_parameter("param_input_file_name_or_ros_topic_prefix_name")
        .get_parameter_value()
        .get<std::string>();
    enable_ransac_floor_detection_ = (this->get_parameter("param_enable_ransac_floor_detection")
                                        .get_parameter_value()
                                        .get<bool>());
    enable_depth_ab_compression_ =
      (this->get_parameter("param_enable_depth_ab_compression").get_parameter_value().get<bool>() ==
       1);
    enable_output_image_compression_ = (this->get_parameter("param_enable_output_image_compression")
                                          .get_parameter_value()
                                          .get<bool>());
    safety_bubble_sensitivity_ =
      this->get_parameter("param_safety_bubble_sensitivity").get_parameter_value().get<int>();
    enable_floor_paint_ =
      (this->get_parameter("param_enable_floor_paint").get_parameter_value().get<bool>());
    enable_safety_bubble_zone_visualization_ =
      (this->get_parameter("param_enable_safety_bubble_zone_visualization")
         .get_parameter_value()
         .get<bool>());
    ransac_distance_threshold_mtr_ =
      this->get_parameter("param_ransac_distance_threshold_mtr").get_parameter_value().get<float>();
    ransac_max_iterations_ =
      this->get_parameter("param_ransac_max_iterations").get_parameter_value().get<int>();
    ab_threshold_ = this->get_parameter("param_ab_threshold").get_parameter_value().get<int>();
    confidence_threshold_ =
      this->get_parameter("param_confidence_threshold").get_parameter_value().get<int>();

    int input_image_width;
    input_image_width =
      this->get_parameter("param_input_image_width").get_parameter_value().get<int>();

    int input_image_height;
    input_image_height =
      this->get_parameter("param_input_image_height").get_parameter_value().get<int>();

    std::string config_file_name_of_tof_sdk;
    config_file_name_of_tof_sdk = this->get_parameter("param_config_file_name_of_tof_sdk")
                                    .get_parameter_value()
                                    .get<std::string>();

    input_sensor_ip_ =
      this->get_parameter("param_input_sensor_ip").get_parameter_value().get<std::string>();          
    int camera_mode = this->get_parameter("param_camera_mode").get_parameter_value().get<int>();    

    fallback_floor_height_offset_mtr_ = 0.1f;
    frame_number_ = 0;
    safety_zone_radius_pixels_ = 0;

    valid_roi_ = {0, 0, 0, 0};

    tunable_params_.ab_threshold = ab_threshold_;
    tunable_params_.confidence_threshold = confidence_threshold_;
    tunable_params_.enable_floor_paint = enable_floor_paint_;
    tunable_params_.enable_ransac_floor_detection = enable_ransac_floor_detection_;
    tunable_params_.enable_safety_bubble_zone_visualization =
      enable_safety_bubble_zone_visualization_;
    tunable_params_.safety_bubble_detection_sensitivity = safety_bubble_sensitivity_;
    tunable_params_.safety_bubble_radius_in_mtr = safety_zone_radius_mtr_;
    tunable_params_.shape_of_safety_bubble = safety_bubble_shape_;

    param_subscriber_ = std::make_shared<rclcpp::ParameterEventHandler>(this);
    callback_handle_ = this->add_on_set_parameters_callback(
      std::bind(&ADI3DToFSafetyBubbleDetector::parametersCallback, this, std::placeholders::_1));

    frame_number_ = 0;

    timer_ =
      this->create_wall_timer(10ms, std::bind(&ADI3DToFSafetyBubbleDetector::timer_callback, this));

    // Get input sensor module
    input_sensor_ = InputSensorFactory::getInputSensor(input_sensor_mode_);

    // Open the sensor
    if (input_sensor_mode_ != 3) {
      // If the mode is not ADTF31xx sensor over Network, then the ip should be set to ""
      input_sensor_ip_.clear();
    }
    input_sensor_->openSensor(
      input_file_name_or_ros_topic_prefix_name_, input_image_width, input_image_height, config_file_name_of_tof_sdk, input_sensor_ip_);    
    if (!input_sensor_->isOpened()) {
      RCLCPP_ERROR(
        this->get_logger(), "Could not open the sensor %s",
        input_file_name_or_ros_topic_prefix_name_.c_str());
      rclcpp::shutdown();
    }

    // Configure the sensor
    input_sensor_->configureSensor(camera_mode);

    image_width_ = input_sensor_->getFrameWidth();
    image_height_ = input_sensor_->getFrameHeight();

    // Get output sensor module
    output_sensor_ = OutputSensorFactory::getOutputSensor(output_sensor_mode_);

    if (output_sensor_ != nullptr) {
      output_sensor_->open(
        input_file_name_or_ros_topic_prefix_name_, image_width_, image_height_, EnableAllOutputs);
    }

    // Create TF listerner instance
    optical_map_tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    optical_map_tf_listener_ =
      std::make_shared<tf2_ros::TransformListener>(*optical_map_tf_buffer_);
    camera_map_tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    camera_map_tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*camera_map_tf_buffer_);
    vcam_tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    vcam_tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*vcam_tf_buffer_);

    // Buffer allocations.
    vcam_depth_frame_8bpp_ = new unsigned char[image_width_ * image_height_];

    // Get intrinsics and extrinsics
    input_sensor_->getIntrinsics(&depth_intrinsics_);
    input_sensor_->getExtrinsics(&depth_extrinsics_);

    populateIdealCameraIntrinsics(&vcam_intrinsics_);

    /*Filtering point cloud based on Z : Select the points with lesser than (safety bubble radius + 0.5m) depth*/
    discard_distance_threshold_mtr_ =
      safety_zone_radius_mtr_ + discard_distance_threshold_delta_mtr_;

    // Get camera link and virtual camera link TF
    getCameraLinksTF();

    // Find the threshold for safety zone
    safety_zone_radius_pixels_ = setZoneRadius();

    safety_bubble_zone_red_mask_ = cv::Mat::zeros(cv::Size(image_width_, image_height_), CV_8UC3);

    if (safety_bubble_shape_ == 0) {
      // Circular Safety Bubble
      safety_bubble_zone_ = cv::Mat::zeros(cv::Size(image_width_, image_height_), CV_8UC1);
      cv::circle(
        safety_bubble_zone_, cv::Point(image_width_ / 2, image_height_ / 2),
        safety_zone_radius_pixels_, 255, -1);
      cv::circle(
        safety_bubble_zone_red_mask_, cv::Point(image_width_ / 2, image_height_ / 2),
        safety_zone_radius_pixels_, cv::Scalar(0, 0, 255), -1);
      cv::circle(
        safety_bubble_zone_red_mask_, cv::Point(image_width_ / 2, image_height_ / 2),
        safety_zone_radius_pixels_, cv::Scalar(0, 0, 255), -1);
    } else if (safety_bubble_shape_ == 1) {
      // Rectangular Safety Bubble
      cv::Point top_left(
        (image_width_ / 2) - safety_zone_radius_pixels_,
        (image_width_ / 2) - safety_zone_radius_pixels_);
      cv::Point bottom_right(
        (image_width_ / 2) + safety_zone_radius_pixels_,
        (image_width_ / 2) + safety_zone_radius_pixels_);

      safety_bubble_zone_ = cv::Mat::zeros(cv::Size(image_width_, image_height_), CV_8UC1);
      cv::rectangle(
        safety_bubble_zone_, top_left, bottom_right, cv::Scalar(255, 255, 255), -1, 8, 0);
      cv::rectangle(
        safety_bubble_zone_red_mask_, top_left, bottom_right, cv::Scalar(0, 0, 255), -1, 8, 0);
    }

    // Output Images
    object_detected_publisher_ = this->create_publisher<std_msgs::msg::Bool>("object_detected", 10);
    if (enable_output_image_compression_ == true) {
      compressed_out_image_publisher_ =
        this->create_publisher<sensor_msgs::msg::CompressedImage>("out_image/compressed", 10);
    } else {
      out_image_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("out_image", 10);
    }

    // Input and Intermediate Debug Images
    if (enable_depth_ab_compression_ == true) {
      compressed_depth_image_publisher_ = this->create_publisher<sensor_msgs::msg::CompressedImage>(
        "depth_image/compressedDepth", 10);
      compressed_ab_image_publisher_ =
        this->create_publisher<sensor_msgs::msg::CompressedImage>("ab_image/compressedDepth", 10);
    } else {
      depth_image_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("depth_image", 10);
      ab_image_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("ab_image", 10);
    }
    // xyz_image_publisher_ = this->advertise<sensor_msgs::PointCloud2>("point_cloud", 10);
    // vcam_depth_image_publisher_ = this->advertise<sensor_msgs::Image>("vcam_depth_image", 10);

    // Camera Infos
    // vcam_info_publisher_ = this->advertise<sensor_msgs::CameraInfo>("virtual_camera_info", 10);
    depth_info_publisher_ = this->create_publisher<sensor_msgs::msg::CameraInfo>("camera_info", 10);

    image_proc_utils_ = new ImageProcUtils(&depth_intrinsics_, image_width_, image_height_);
    floor_plane_detection_ = new FloorPlaneDetection(
      image_width_, image_height_, ransac_distance_threshold_mtr_, ransac_max_iterations_,
      discard_distance_threshold_mtr_, camera_height_mtr_);

    // For File-io, we do not want to miss any frame, so increasing the queue size.
    if ((input_sensor_mode_ != 0) && (input_sensor_mode_ != 3)) {
      max_input_queue_length_ = 100;
      max_output_queue_length_ = 100;
    }

    // setting ab thresold to pulsatrix
    input_sensor_->setABinvalidationThreshold(ab_threshold_);

    // setting confidence threshold to pulsatrix
    input_sensor_->setConfidenceThreshold(confidence_threshold_);
  }

  bool runSafetyBubbleDetection();
  void processOutput();
  void processOutputAbort();
  void readInputAbort();
  void readInput();
  void updateDynamicReconfigureVariablesInputThread();
  void updateDynamicReconfigureVariablesProcessThread();
  ADTF31xxSensorFrameInfo * safetyBubbleDetectorIOThreadGetNextFrame(void);
  ADI3DToFSafetyBubbleDetectorOutputInfo * safetyBubbleDetectorIOThreadGetNextOutputNode(void);

  /**
   * @brief Destroy the ADI3DToFSafetyBubbleDetector object
   *
   */
  ~ADI3DToFSafetyBubbleDetector()
  {
    // Close the input sensor
    input_sensor_->closeSensor();

    delete[] vcam_depth_frame_8bpp_;

    delete image_proc_utils_;
    delete floor_plane_detection_;

    // Close outputs
    if (output_sensor_ != nullptr) {
      output_sensor_->close();
    }
  }

  /**
   * @brief 
   * 
   */
  void timer_callback()
  {
    if (!runSafetyBubbleDetection()) {
      //Raise an exception to shutdown the Node.
      throw std::runtime_error("Error Running Safety bubble detector");
    }
  }

private:
  rclcpp::TimerBase::SharedPtr timer_;
  IInputSensor * input_sensor_;
  OOutputSensor * output_sensor_;
  ImageProcUtils * image_proc_utils_;
  FloorPlaneDetection * floor_plane_detection_;
  std::string camera_link_;
  std::string optical_camera_link_;
  std::string virtual_camera_link_;
  float safety_zone_radius_mtr_;
  int safety_bubble_shape_;
  int safety_bubble_sensitivity_;
  int safety_zone_radius_pixels_;
  float virtual_camera_height_mtr_;
  int input_sensor_mode_;
  int output_sensor_mode_;
  bool enable_depth_ab_compression_;
  bool enable_output_image_compression_;
  bool compute_point_cloud_enable_ = false;
  int image_width_;
  int image_height_;
  int frame_number_;
  int ab_threshold_ = 10;
  int confidence_threshold_ = 10;
  std::string input_file_name_or_ros_topic_prefix_name_;
  std::string input_sensor_ip_;
  std::string output_file_name_;
  std::unique_ptr<tf2_ros::Buffer> vcam_tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> vcam_tf_listener_{nullptr};
  std::unique_ptr<tf2_ros::Buffer> optical_map_tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> optical_map_tf_listener_{nullptr};
  std::unique_ptr<tf2_ros::Buffer> camera_map_tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> camera_map_tf_listener_{nullptr};
  sensor_msgs::msg::CameraInfo cam_info_msg_;
  unsigned short * depth_frame_ = nullptr;
  unsigned short * depth_frame_with_floor_ = nullptr;
  unsigned short * ab_frame_ = nullptr;
  short * xyz_frame_ = nullptr;
  short * rotated_xyz_frame_ = nullptr;
  unsigned short * vcam_depth_frame_ = nullptr;
  unsigned char * vcam_depth_frame_8bpp_ = nullptr;
  unsigned char * vcam_depth_image_floor_pixels_removed_8bpp_ = nullptr;
  unsigned short * vcam_depth_frame_with_floor_ = nullptr;
  //Publishers
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr object_detected_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr out_image_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_image_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr ab_image_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr compressed_out_image_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr compressed_depth_image_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr compressed_ab_image_publisher_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr xyz_image_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr vcam_depth_image_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr depth_info_publisher_;
  //rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr vcam_info_publisher_;
  CameraIntrinsics depth_intrinsics_;
  CameraExtrinsics depth_extrinsics_;
  CameraExtrinsics depth_extrinsics_external_;
  CameraIntrinsics vcam_intrinsics_;
  CameraExtrinsics vcam_extrinsics_;

  bool process_output_thread_abort_ = false;
  bool read_input_thread_abort_ = false;

  std::mutex output_thread_mtx_;
  std::mutex input_thread_mtx_;

  cv::Mat safety_bubble_zone_;
  cv::Mat safety_bubble_zone_red_mask_;
  bool enable_safety_bubble_zone_visualization_;

  unsigned char * compressed_depth_frame_ = nullptr;
  unsigned char * compressed_ab_frame_ = nullptr;
  int compressed_depth_frame_size_ = 0;
  int compressed_ab_frame_size_ = 0;

  /*Floor Detection Variables*/
  /**
   * @brief Variable to choose which floor detection algorithm to run
   * 1. RANSAC based (default)
   * 2. Threshold Y based
   *
   */
  bool enable_ransac_floor_detection_ = true;

  /**
   * @brief Result status of RANSAC floor detection
   * If the status is true then RANSAC result can be considered, otherwise, run the Threshold Y based floor detection.
   *
   */
  bool ransac_floor_detection_status_ = false;

  /**
   * @brief Rotation around X axis in radian
   *
   */
  float camera_roll_rad_ = 0.0f;

  /**
   * @brief Rotation around Y axis in radian
   *
   */
  float camera_pitch_rad_ = 0.0f;

  /**
   * @brief Rotation around Z axis in radian
   *
   */
  float camera_yaw_rad_ = 0.0f;

  /**
   * @brief Flag for camera rotation
   *
   */
  bool camera_tilted_ = false;

  /**
   * @brief distance (25mm) which determines how close the point must be to the RANSAC plane in order to be selected as
   * inlier.
   *
   */
  float ransac_distance_threshold_mtr_ = 0.025f;

  /**
   * @brief Maximum number of RANSAC iterations (10) which is allowed
   *
   */
  int ransac_max_iterations_ = 10;

  /**
   * @brief Small offset value used in filtering point cloud based on depth (Z)
   *
   */
  float discard_distance_threshold_delta_mtr_ = 0.5f;

  /**
   * @brief Threshold to filter the point cloud based on depth (Z)
   * Select the points with depth value lesser than 1500mm from the sensor (safety bubble radius 1.0m + delta 0.5m)
   *
   */
  float discard_distance_threshold_mtr_ = 1.5f;

  /**
   * @brief Camera height, default is 0.1524m (6 inches)
   *
   */
  float camera_height_mtr_ = 0.1524f;

  /**
   * @brief Points which are lower than ground
   *
   */
  int noise_count_ = 0;

  /**
   * @brief Number of iterations RANSAC has taken for floor detection
   *
   */
  int ransac_iterations_ = 0;

  /**
   * @brief Time (ms) RANSAC has taken for floor detection
   *
   */
  float ransac_time_ms_ = 0.0f;

  /**
   * @brief Enable painting the floor pixels in output image for visulation of the floor
   *
   */
  bool enable_floor_paint_ = false;

  /**
   * @brief Floor height with offset for Threshold Y based floor detection.
   *
   */
  float fallback_floor_height_offset_mtr_ = 0.1f;

  /**
   * @brief    This is the scale factor to scale the input image.
               The processing will happen on the scaled down image
               The topics and the output coordinates will correspond
               to the scaled image size.
   *
   */
  int processing_scale_ = 2;

  rclcpp::Time curr_frame_timestamp_ = rclcpp::Clock{}.now();

  std::shared_ptr<rclcpp::ParameterEventHandler> param_subscriber_;
  OnSetParametersCallbackHandle::SharedPtr callback_handle_;

  struct TunableParameters
  {
    int ab_threshold;
    int confidence_threshold;
    double safety_bubble_radius_in_mtr;
    int shape_of_safety_bubble;
    int safety_bubble_detection_sensitivity;
    bool enable_ransac_floor_detection;
    bool enable_floor_paint;
    bool enable_safety_bubble_zone_visualization;
  };

  TunableParameters tunable_params_;

  /**
   * @brief new values from dynamic reconfigure are copied to a struture variable here, actual update to individual
   * parameters happens in updateDynamicReconfigureVariablesInputThread and updateDynamicReconfigureVariablesProcessThread
   * functions.
   *
   * @param config Config parameters present in GUI
   * @param level
   */
  rcl_interfaces::msg::SetParametersResult parametersCallback(
    const std::vector<rclcpp::Parameter> & parameters)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    result.reason = "success";
    // Copy the parameters vector to a local variable.
    for (const auto & param : parameters) {
      if (param.get_name() == "param_ab_threshold") {
        if (tunable_params_.ab_threshold != param.as_int()) {
          tunable_params_.ab_threshold = param.as_int();
          RCLCPP_INFO(
            this->get_logger(), "The value of param_ab_threshold is changed to %s",
            param.value_to_string().c_str());
        }
      }

      if (param.get_name() == "param_confidence_threshold") {
        if (tunable_params_.confidence_threshold != param.as_int()) {
          tunable_params_.confidence_threshold = param.as_int();
          RCLCPP_INFO(
            this->get_logger(), "The value of param_confidence_threshold is changed to %s",
            param.value_to_string().c_str());
        }
      }

      if (param.get_name() == "param_safety_zone_radius_in_mtr") {
        if (tunable_params_.safety_bubble_radius_in_mtr != param.as_double()) {
          tunable_params_.safety_bubble_radius_in_mtr = param.as_double();
          RCLCPP_INFO(
            this->get_logger(), "The value of safety bubble radius is changed to %s",
            param.value_to_string().c_str());
        }
      }

      if (param.get_name() == "param_safety_bubble_shape") {
        if (tunable_params_.shape_of_safety_bubble != param.as_int()) {
          tunable_params_.shape_of_safety_bubble = param.as_int();
          RCLCPP_INFO(
            this->get_logger(), "The value of param_safety_bubble_shape is changed to %s",
            param.value_to_string().c_str());
        }
      }

      if (param.get_name() == "param_safety_bubble_sensitivity") {
        if (tunable_params_.safety_bubble_detection_sensitivity != param.as_int()) {
          tunable_params_.safety_bubble_detection_sensitivity = param.as_int();
          RCLCPP_INFO(
            this->get_logger(), "The value of param_safety_bubble_sensitivity is changed to %s",
            param.value_to_string().c_str());
        }
      }

      if (param.get_name() == "param_enable_ransac_floor_detection") {
        if (tunable_params_.enable_ransac_floor_detection != param.as_bool()) {
          tunable_params_.enable_ransac_floor_detection = param.as_bool();
          RCLCPP_INFO(
            this->get_logger(), "The value of param_enable_ransac_floor_detection is changed to %s",
            param.value_to_string().c_str());
        }
      }

      if (param.get_name() == "param_enable_floor_paint") {
        if (tunable_params_.enable_floor_paint != param.as_bool()) {
          tunable_params_.enable_floor_paint = param.as_bool();
          RCLCPP_INFO(
            this->get_logger(), "The value of param_enable_floor_paint is changed to %s",
            param.value_to_string().c_str());
        }
      }

      if (param.get_name() == "param_enable_safety_bubble_zone_visualization") {
        if (tunable_params_.enable_safety_bubble_zone_visualization != param.as_bool()) {
          tunable_params_.enable_safety_bubble_zone_visualization = param.as_bool();
          RCLCPP_INFO(
            this->get_logger(),
            "The value of param_enable_safety_bubble_zone_visualization is changed to %s",
            param.value_to_string().c_str());
        }
      }
    }
    return result;
  }

  /**
   * @brief updates the parameter of input image based on dynamic reconfigure.
   *
   */
  void updateTunableParameters()
  {
    // setting AB threshold and confidence threshold values if they are changed.
    if (ab_threshold_ != tunable_params_.ab_threshold) {
      ab_threshold_ = tunable_params_.ab_threshold;
      RCLCPP_INFO(this->get_logger(), "Changed AB threshold value is %d", ab_threshold_);
      input_sensor_->setABinvalidationThreshold(ab_threshold_);
    }

    if (confidence_threshold_ != tunable_params_.confidence_threshold) {
      confidence_threshold_ = tunable_params_.confidence_threshold;
      RCLCPP_INFO(
        this->get_logger(), "Changed Confidence threshold value is %d", confidence_threshold_);
      input_sensor_->setConfidenceThreshold(confidence_threshold_);
    }

    if (safety_zone_radius_mtr_ != tunable_params_.safety_bubble_radius_in_mtr) {
      safety_zone_radius_mtr_ = tunable_params_.safety_bubble_radius_in_mtr;
      RCLCPP_INFO(
        this->get_logger(), "Changed safety bubble radius in meter value is %lf",
        safety_zone_radius_mtr_);
    }

    if (safety_bubble_shape_ != tunable_params_.shape_of_safety_bubble) {
      safety_bubble_shape_ = tunable_params_.shape_of_safety_bubble;
      RCLCPP_INFO(
        this->get_logger(), "Changed shape of safety bubble value is %d", safety_bubble_shape_);
    }

    if (safety_bubble_sensitivity_ != tunable_params_.safety_bubble_detection_sensitivity) {
      safety_bubble_sensitivity_ = tunable_params_.safety_bubble_detection_sensitivity;
      RCLCPP_INFO(
        this->get_logger(), "Changed Safety bubble detection sensitivity value is %d",
        safety_bubble_sensitivity_);
    }

    if (enable_ransac_floor_detection_ != tunable_params_.enable_ransac_floor_detection) {
      enable_ransac_floor_detection_ = tunable_params_.enable_ransac_floor_detection;
      RCLCPP_INFO(
        this->get_logger(), "Enable bit for ransac floor detection is changed to %d",
        enable_ransac_floor_detection_);
    }

    if (enable_floor_paint_ != tunable_params_.enable_floor_paint) {
      enable_floor_paint_ = tunable_params_.enable_floor_paint;
      RCLCPP_INFO(
        this->get_logger(), "Enable bit for floor paint is changed to  %d", enable_floor_paint_);
    }

    if (
      enable_safety_bubble_zone_visualization_ !=
      tunable_params_.enable_safety_bubble_zone_visualization) {
      enable_safety_bubble_zone_visualization_ =
        tunable_params_.enable_safety_bubble_zone_visualization;
      RCLCPP_INFO(
        this->get_logger(), "Enable bit for safety bubble zone visualization is changed to %d",
        enable_safety_bubble_zone_visualization_);
    }
  }

  int max_input_queue_length_ = 1;

  std::queue<ADTF31xxSensorFrameInfo *> input_frames_queue_;

  int max_output_queue_length_ = 5;

  std::queue<ADI3DToFSafetyBubbleDetectorOutputInfo *> output_node_queue_;

  bool error_in_frame_read_ = false;

  /**
   * @brief    ROI details after transformation
   */
  ADIImageROI valid_roi_;

  void getCameraLinksTF();

  void convertDepthCamToVirtualCamFrame();

  void fillAndPublishCameraInfo(
    const std::string & frame_id,
    const rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr publisher);

  void publishImageAsRosMsg(
    cv::Mat img, const std::string & encoding_type, std::string frame_id,
    const rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher);

  void publishCompressedImageAsRosMsg(
    cv::Mat img, const std::string & encoding_type, std::string frame_id,
    const rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr publisher);

  void publishPointCloud(short * xyz_frame);

  void publishImageAndCameraInfo(
    unsigned short * depth_frame, unsigned short * ab_frame, unsigned short * vcam_depth_frame,
    short * xyz_frame);

  void publishImageAndCameraInfo(
    unsigned char * compressed_depth_frame, int compressed_depth_frame_size,
    unsigned char * compressed_ab_frame, int compressed_ab_frame_size);

  int setZoneRadius();

  bool safetyBubbleDetection();

  cv::Mat generateVisualizationImage(
    unsigned char * vcam_depth_image_floor_pixels_removed_8bpp,
    unsigned short * vcam_depth_frame_with_floor, bool object_detected);

  void safetyBubbleDetectorIOThreadPushOutputNode(
    ADI3DToFSafetyBubbleDetectorOutputInfo * new_output_node);

  void publishRVLCompressedImageAsRosMsg(
    unsigned char * compressed_img, int compressed_img_size, const std::string & encoding_type,
    std::string frame_id,
    const rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr publisher);

  /**
   * @brief populates virual view camera intrinsics
   *
   * @param camera_intrinsics pointer to camera intrinsics
   */
  void populateIdealCameraIntrinsics(CameraIntrinsics * camera_intrinsics)
  {

    camera_intrinsics->camera_matrix[0] = depth_intrinsics_.camera_matrix[0];
    camera_intrinsics->camera_matrix[1] = 0.0f;
    camera_intrinsics->camera_matrix[2] = depth_intrinsics_.camera_matrix[2];
    camera_intrinsics->camera_matrix[3] = 0.0f;
    camera_intrinsics->camera_matrix[4] = depth_intrinsics_.camera_matrix[4];
    camera_intrinsics->camera_matrix[5] = depth_intrinsics_.camera_matrix[5];
    camera_intrinsics->camera_matrix[6] = 0.0f;
    camera_intrinsics->camera_matrix[7] = 0.0f;
    camera_intrinsics->camera_matrix[8] = 1.0f;
    camera_intrinsics->distortion_coeffs[0] = 0.0f;
    camera_intrinsics->distortion_coeffs[1] = 0.0f;
    camera_intrinsics->distortion_coeffs[2] = 0.0f;
    camera_intrinsics->distortion_coeffs[3] = 0.0f;
    camera_intrinsics->distortion_coeffs[4] = 0.0f;
    camera_intrinsics->distortion_coeffs[5] = 0.0f;
    camera_intrinsics->distortion_coeffs[6] = 0.0f;
    camera_intrinsics->distortion_coeffs[7] = 0.0f;

    return;
  }
};

#endif
