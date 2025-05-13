/******************************************************************************
Copyright (c), 2023 - Analog Devices Inc. All Rights Reserved.
This software is PROPRIETARY & CONFIDENTIAL to Analog Devices, Inc.
and its licensors.
******************************************************************************/
#ifndef ADI_3DTOF_SAFETY_BUBBLE_DETECTOR_NODE_H
#define ADI_3DTOF_SAFETY_BUBBLE_DETECTOR_NODE_H

#include "input_sensor.h"
#include "input_sensor_factory.h"
#include "image_proc_utils.h"
#include "output_sensor.h"
#include "output_sensor_factory.h"
#include "floor_plane_detection.h"
#include "adtf31xx_sensor_frame_info.h"
#include "adi_3dtof_safety_bubble_detector_output_info.h"
#include <ros/ros.h>
#include <Eigen/Geometry>
#include <utility>
#include <std_msgs/Bool.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/buffer.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <eigen_conversions/eigen_msg.h>
#include <image_geometry/pinhole_camera_model.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <sensor_msgs/distortion_models.h>
#include <tf2/LinearMath/Quaternion.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <sensor_msgs/PointCloud2.h>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <queue>
#include <dynamic_reconfigure/server.h>
#include <adi_3dtof_safety_bubble_detector/SafetyBubbleDetectorParamsConfig.h>
#include <compressed_depth_image_transport/compression_common.h>

namespace enc = sensor_msgs::image_encodings;

/**
 * @brief This is main class for this package
 *
 */
class ADI3DToFSafetyBubbleDetector : public ros::NodeHandle
{
public:
  /**
   * @brief Construct a new ADI3DToFSafetyBubbleDetector object
   *
   */
  ADI3DToFSafetyBubbleDetector()
  {
    dynamic_reconfigure_callbacktype_ =
        boost::bind(&ADI3DToFSafetyBubbleDetector::dynamicallyReconfigureVariables, this, _1, _2);

    server_.setCallback(dynamic_reconfigure_callbacktype_);

    ros::NodeHandle nh("~");

    // Get Parameters
    // ToF camera link
    std::string camera_link;
    nh.param<std::string>("param_camera_link", camera_link, "adi_camera_link");

    std::string optical_camera_link;
    nh.param<std::string>("param_optical_camera_link", optical_camera_link, "optical_camera_link");

    // Virtual camera link
    std::string virtual_camera_link;
    nh.param<std::string>("param_virtual_camera_link", virtual_camera_link, "virtual_camera_link");

    // Safety zone radius
    float safety_zone_radius_mtr;
    nh.param<float>("param_safety_zone_radius_in_mtr", safety_zone_radius_mtr, 1.0f);

    // Shape of safety bubble. 0 = circular, 1 = square
    int safety_bubble_shape;
    nh.param<int>("param_safety_bubble_shape", safety_bubble_shape, 0);

    // Camera height
    float virtual_camera_height_mtr;
    nh.param<float>("param_virtual_camera_height", virtual_camera_height_mtr, 5.0f);

    // Input sensor Mode:Camera/File/ROSBag
    int input_sensor_mode;
    nh.param<int>("param_input_sensor_mode", input_sensor_mode, 0);

    // Output sensor Mode:video,csv
    int output_sensor_mode;
    nh.param<int>("param_output_sensor_mode", output_sensor_mode, 0);

    // input file name in FileIO mode or rostopic prefix name
    std::string input_file_name_or_ros_topic_prefix_name;
    nh.param<std::string>("param_input_file_name_or_ros_topic_prefix_name", input_file_name_or_ros_topic_prefix_name,
                          "no name");

    // enable option to run ransac floor detection algortihm
    int enable_ransac_floor_detection;
    nh.param<int>("param_enable_ransac_floor_detection", enable_ransac_floor_detection, 1);

    // enable option to publish depth and ir compressed images
    int enable_depth_ab_compression;
    nh.param<int>("param_enable_depth_ab_compression", enable_depth_ab_compression, 0);

    // enable option to publish compressed output image.
    int enable_output_image_compression;
    nh.param<int>("param_enable_output_image_compression", enable_output_image_compression, 1);

    // number of connected pixels to trigger object detection
    int safety_bubble_sensitivity;
    nh.param<int>("param_safety_bubble_sensitivity", safety_bubble_sensitivity, 10);

    // enable option to visualize for floor paint
    int enable_floor_paint;
    nh.param<int>("param_enable_floor_paint", enable_floor_paint, 0);

    // enable option to visualize safety bubble zone
    int enable_safety_bubble_zone_visualization;
    nh.param<int>("param_enable_safety_bubble_zone_visualization", enable_safety_bubble_zone_visualization, 1);

    // Do not change the default values of the below 3 parameters
    // image width
    int input_image_width;
    nh.param<int>("param_input_image_width", input_image_width, 1024);

    // image height
    int input_image_height;
    nh.param<int>("param_input_image_height", input_image_height, 1024);

    // scale factor to determine some algorithm parameters
    int processing_scale;
    nh.param<int>("param_processing_scale", processing_scale, 2);

    // the height of the floor ransac finds
    float ransac_distance_threshold_mtr;
    nh.param<float>("param_ransac_distance_threshold_mtr", ransac_distance_threshold_mtr, 0.025f);

    // maxium iterations ransac is allowed
    int ransac_max_iterations;
    nh.param<int>("param_ransac_max_iterations", ransac_max_iterations, 10);

    // ab threshold to be set to device
    int ab_threshold;
    nh.param<int>("param_ab_threshold", ab_threshold, 10);

    // confidence threshold to be set to device
    int confidence_threshold;
    nh.param<int>("param_confidence_threshold", confidence_threshold, 10);

    // path of the config file to read from tof sdk
    std::string config_file_name_of_tof_sdk;
    nh.param<std::string>("param_config_file_name_of_tof_sdk", config_file_name_of_tof_sdk, "no name");

    // camera mode supported in TOF SDK
    int camera_mode;
    nh.param<int>("param_camera_mode", camera_mode, 4);

    std::string input_sensor_ip;
    nh.param<std::string>("param_input_sensor_ip", input_sensor_ip, "no name");

    camera_link_ = std::move(camera_link);
    optical_camera_link_ = std::move(optical_camera_link);
    virtual_camera_link_ = std::move(virtual_camera_link);
    safety_zone_radius_mtr_ = safety_zone_radius_mtr;
    safety_bubble_shape_ = safety_bubble_shape;
    safety_bubble_sensitivity_ = safety_bubble_sensitivity;
    input_sensor_mode_ = input_sensor_mode;
    output_sensor_mode_ = output_sensor_mode;
    input_file_name_or_ros_topic_prefix_name_ = std::move(input_file_name_or_ros_topic_prefix_name);
    input_sensor_ip_ = std::move(input_sensor_ip);
    fallback_floor_height_offset_mtr_ = 0.1f;
    virtual_camera_height_mtr_ = virtual_camera_height_mtr;
    frame_number_ = 0;
    safety_zone_radius_pixels_ = 0;
    enable_ransac_floor_detection_ = (enable_ransac_floor_detection == 1) ? true : false;
    enable_floor_paint_ = (enable_floor_paint == 1) ? true : false;
    enable_depth_ab_compression_ = (enable_depth_ab_compression == 1) ? true : false;
    enable_output_image_compression_ = (enable_output_image_compression == 1) ? true : false;
    enable_safety_bubble_zone_visualization_ = (enable_safety_bubble_zone_visualization == 1) ? true : false;
    ransac_distance_threshold_mtr_ = ransac_distance_threshold_mtr;
    ransac_max_iterations_ = ransac_max_iterations;
    ab_threshold_ = ab_threshold;
    confidence_threshold_ = confidence_threshold;

    valid_roi_ = { 0 };

    // Get input sensor module
    input_sensor_ = InputSensorFactory::getInputSensor(input_sensor_mode_);

    // Open the sensor
    if (input_sensor_mode_ != 3)
    {
      // If the mode is not ADTF31xx sensor over Network, then the ip should be set to ""
      input_sensor_ip_.clear();
    }

    // Open the sensor
    input_sensor_->openSensor(input_file_name_or_ros_topic_prefix_name_, input_image_width, input_image_height,
                              config_file_name_of_tof_sdk, input_sensor_ip_);
    if (!input_sensor_->isOpened())
    {
      ROS_ERROR("Could not open the sensor %s", input_file_name_or_ros_topic_prefix_name_.c_str());
      shutDownAllNodes();
    }

    // Configure the sensor
    input_sensor_->configureSensor(camera_mode);
    image_width_ = input_sensor_->getFrameWidth();
    image_height_ = input_sensor_->getFrameHeight();

    // Get output sensor module
    output_sensor_ = OutputSensorFactory::getOutputSensor(output_sensor_mode_);

    if (output_sensor_ != nullptr)
    {
      output_sensor_->open(input_file_name_or_ros_topic_prefix_name_, image_width_, image_height_, EnableAllOutputs);
    }

    // Create TF listerner instance
    optical_map_tf_listener_ = new tf2_ros::TransformListener(optical_map_tf_buffer_);
    camera_map_tf_listener_ = new tf2_ros::TransformListener(camera_map_tf_buffer_);
    vcam_tf_listener_ = new tf2_ros::TransformListener(vcam_tf_buffer_);

    // Buffer allocations.
    vcam_depth_frame_8bpp_ = new unsigned char[image_width_ * image_height_];

    // Get intrinsics and extrinsics
    input_sensor_->getIntrinsics(&depth_intrinsics_);
    input_sensor_->getExtrinsics(&depth_extrinsics_);

    populateIdealCameraIntrinsics(&vcam_intrinsics_);

    /*Filtering point cloud based on Z : Select the points with lesser than (safety bubble radius + 0.5m) depth*/
    discard_distance_threshold_mtr_ = safety_zone_radius_mtr_ + discard_distance_threshold_delta_mtr_;

    // Get camera link and virtual camera link TF
    getCameraLinksTF();

    // Find the threshold for safety zone
    safety_zone_radius_pixels_ = setZoneRadius();

    safety_bubble_zone_red_mask_ = cv::Mat::zeros(cv::Size(image_width_, image_height_), CV_8UC3);

    if (safety_bubble_shape_ == 0)
    {
      // Circular Safety Bubble
      safety_bubble_zone_ = cv::Mat::zeros(cv::Size(image_width_, image_height_), CV_8UC1);
      cv::circle(safety_bubble_zone_, cv::Point(image_width_ / 2, image_height_ / 2), safety_zone_radius_pixels_, 255,
                 -1);
      cv::circle(safety_bubble_zone_red_mask_, cv::Point(image_width_ / 2, image_height_ / 2),
                 safety_zone_radius_pixels_, cv::Scalar(0, 0, 255), -1);
      cv::circle(safety_bubble_zone_red_mask_, cv::Point(image_width_ / 2, image_height_ / 2),
                 safety_zone_radius_pixels_, cv::Scalar(0, 0, 255), -1);
    }
    else if (safety_bubble_shape_ == 1)
    {
      // Rectangular Safety Bubble
      cv::Point top_left((image_width_ / 2) - safety_zone_radius_pixels_,
                         (image_width_ / 2) - safety_zone_radius_pixels_);
      cv::Point bottom_right((image_width_ / 2) + safety_zone_radius_pixels_,
                             (image_width_ / 2) + safety_zone_radius_pixels_);

      safety_bubble_zone_ = cv::Mat::zeros(cv::Size(image_width_, image_height_), CV_8UC1);
      cv::rectangle(safety_bubble_zone_, top_left, bottom_right, cv::Scalar(255, 255, 255), -1, 8, 0);
      cv::rectangle(safety_bubble_zone_red_mask_, top_left, bottom_right, cv::Scalar(0, 0, 255), -1, 8, 0);
    }

    // Output Images
    object_detected_publisher_ = this->advertise<std_msgs::Bool>("object_detected", 10);
    if (enable_output_image_compression_ == true)
    {
      out_image_publisher_ = this->advertise<sensor_msgs::CompressedImage>("out_image/compressed", 10);
    }
    else
    {
      out_image_publisher_ = this->advertise<sensor_msgs::Image>("out_image", 10);
    }

    // Input and Intermediate Debug Images
    if (enable_depth_ab_compression == true)
    {
      depth_image_publisher_ = this->advertise<sensor_msgs::CompressedImage>("depth_image/compressedDepth", 10);
      ab_image_publisher_ = this->advertise<sensor_msgs::CompressedImage>("ab_image/compressedDepth", 10);
    }
    else
    {
      depth_image_publisher_ = this->advertise<sensor_msgs::Image>("depth_image", 10);
      ab_image_publisher_ = this->advertise<sensor_msgs::Image>("ab_image", 10);
    }
    // xyz_image_publisher_ = this->advertise<sensor_msgs::PointCloud2>("point_cloud", 10);
    // vcam_depth_image_publisher_ = this->advertise<sensor_msgs::Image>("vcam_depth_image", 10);

    // Camera Infos
    // vcam_info_publisher_ = this->advertise<sensor_msgs::CameraInfo>("virtual_camera_info", 10);
    depth_info_publisher_ = this->advertise<sensor_msgs::CameraInfo>("camera_info", 10);

    image_proc_utils_ = new ImageProcUtils(&depth_intrinsics_, image_width_, image_height_);
    floor_plane_detection_ =
        new FloorPlaneDetection(image_width_, image_height_, ransac_distance_threshold_mtr_, ransac_max_iterations_,
                                discard_distance_threshold_mtr_, camera_height_mtr_);

    // For File-io, we do not want to miss any frame, so increasing the queue size.
    if ((input_sensor_mode_ != 0) && (input_sensor_mode_ != 3) && (input_sensor_mode_ != 4))
    {
      max_input_queue_length_ = 100;
      max_output_queue_length_ = 100;
    }

    // setting ab thresold to pulsatrix
    input_sensor_->setABinvalidationThreshold(ab_threshold_);

    // setting confidence threshold to pulsatrix
    input_sensor_->setConfidenceThreshold(confidence_threshold_);

    // Initially setting dynamic reconfigure values to same as launch file
    initSettingsForDynamicReconfigure();
  }

  void shutDownAllNodes();

  bool runSafetyBubbleDetection();

  void processOutput();

  void processOutputAbort();

  void readInputAbort();

  void readInput();

  void updateDynamicReconfigureVariablesInputThread();

  void updateDynamicReconfigureVariablesProcessThread();

  ADTF31xxSensorFrameInfo* safetyBubbleDetectorIOThreadGetNextFrame(void);

  ADI3DToFSafetyBubbleDetectorOutputInfo* safetyBubbleDetectorIOThreadGetNextOutputNode(void);

  void dynamicallyReconfigureVariables(adi_3dtof_safety_bubble_detector::SafetyBubbleDetectorParamsConfig& config,
                                       uint32_t level);

  /**
   * @brief Destroy the ADI3DToFSafetyBubbleDetector object
   *
   */
  ~ADI3DToFSafetyBubbleDetector()
  {
    // Close the input sensor
    input_sensor_->closeSensor();

    delete optical_map_tf_listener_;
    delete camera_map_tf_listener_;
    delete vcam_tf_listener_;
    delete[] vcam_depth_frame_8bpp_;

    delete image_proc_utils_;
    delete floor_plane_detection_;

    // Close outputs
    if (output_sensor_ != nullptr)
    {
      output_sensor_->close();
    }
  }

private:
  IInputSensor* input_sensor_;
  OOutputSensor* output_sensor_;
  ImageProcUtils* image_proc_utils_;
  FloorPlaneDetection* floor_plane_detection_;
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
  tf2_ros::Buffer vcam_tf_buffer_;
  tf2_ros::TransformListener* vcam_tf_listener_;
  tf2_ros::Buffer optical_map_tf_buffer_;
  tf2_ros::Buffer camera_map_tf_buffer_;
  tf2_ros::TransformListener* optical_map_tf_listener_;
  tf2_ros::TransformListener* camera_map_tf_listener_;
  sensor_msgs::CameraInfo cam_info_msg_;
  unsigned short* depth_frame_ = nullptr;
  unsigned short* depth_frame_with_floor_ = nullptr;
  unsigned short* ab_frame_ = nullptr;
  short* xyz_frame_ = nullptr;
  short* rotated_xyz_frame_ = nullptr;
  unsigned short* vcam_depth_frame_ = nullptr;
  unsigned char* vcam_depth_frame_8bpp_ = nullptr;
  unsigned char* vcam_depth_image_floor_pixels_removed_8bpp_ = nullptr;
  unsigned short* vcam_depth_frame_with_floor_ = nullptr;
  ros::Publisher object_detected_publisher_;
  ros::Publisher out_image_publisher_;
  ros::Publisher depth_image_publisher_;
  ros::Publisher ab_image_publisher_;
  ros::Publisher xyz_image_publisher_;
  ros::Publisher vcam_depth_image_publisher_;
  ros::Publisher depth_info_publisher_;
  ros::Publisher vcam_info_publisher_;
  CameraIntrinsics depth_intrinsics_;
  CameraExtrinsics depth_extrinsics_;
  CameraExtrinsics depth_extrinsics_external_;
  CameraIntrinsics vcam_intrinsics_;
  CameraExtrinsics vcam_extrinsics_;

  bool process_output_thread_abort_ = false;
  bool read_input_thread_abort_ = false;

  boost::mutex output_thread_mtx_;
  boost::mutex input_thread_mtx_;

  cv::Mat safety_bubble_zone_;
  cv::Mat safety_bubble_zone_red_mask_;
  bool enable_safety_bubble_zone_visualization_;

  unsigned char* compressed_depth_frame_ = nullptr;
  unsigned char* compressed_ab_frame_ = nullptr;
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

  ros::Time curr_frame_timestamp_ = ros::Time::now();

  int max_input_queue_length_ = 1;

  std::queue<ADTF31xxSensorFrameInfo*> input_frames_queue_;

  int max_output_queue_length_ = 5;

  std::queue<ADI3DToFSafetyBubbleDetectorOutputInfo*> output_node_queue_;

  bool error_in_frame_read_ = false;

  dynamic_reconfigure::Server<adi_3dtof_safety_bubble_detector::SafetyBubbleDetectorParamsConfig> server_;

  dynamic_reconfigure::Server<adi_3dtof_safety_bubble_detector::SafetyBubbleDetectorParamsConfig>::CallbackType
      dynamic_reconfigure_callbacktype_;

  adi_3dtof_safety_bubble_detector::SafetyBubbleDetectorParamsConfig dynamic_reconfigure_config_;

  /**
   * @brief    ROI details after transformation
   */
  ADIImageROI valid_roi_;

  void initSettingsForDynamicReconfigure();

  void getCameraLinksTF();

  void convertDepthCamToVirtualCamFrame();

  void fillAndPublishCameraInfo(const std::string& frame_id, const ros::Publisher& publisher);

  void publishImageAsRosMsg(cv::Mat img, const std::string& encoding_type, std::string frame_id,
                            const ros::Publisher& publisher, bool enable_image_compression);

  void publishPointCloud(short* xyz_frame);

  void publishImageAndCameraInfo(unsigned short* depth_frame, unsigned short* ab_frame,
                                 unsigned short* vcam_depth_frame, short* xyz_frame);

  void publishImageAndCameraInfo(unsigned char* compressed_depth_frame, int compressed_depth_frame_size,
                                 unsigned char* compressed_ab_frame, int compressed_ab_frame_size);

  int setZoneRadius();

  bool safetyBubbleDetection();

  cv::Mat generateVisualizationImage(unsigned char* vcam_depth_image_floor_pixels_removed_8bpp,
                                     unsigned short* vcam_depth_frame_with_floor, bool object_detected);

  void safetyBubbleDetectorIOThreadPushOutputNode(ADI3DToFSafetyBubbleDetectorOutputInfo* new_output_node);

  void publishRVLCompressedImageAsRosMsg(unsigned char* compressed_img, int compressed_img_size,
                                         const std::string& encoding_type, std::string frame_id,
                                         const ros::Publisher& publisher);

  /**
   * @brief populates virual view camera intrinsics
   *
   * @param camera_intrinsics pointer to camera intrinsics
   */
  void populateIdealCameraIntrinsics(CameraIntrinsics* camera_intrinsics)
  {
    // The hardcoded values used below correspond to
    // 1024x1024 image. Hence scale the values as per
    // actual image resolution.
    float scale = 1024 / image_width_;

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
