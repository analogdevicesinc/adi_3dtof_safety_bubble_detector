/******************************************************************************
Copyright (c), 2023 - Analog Devices Inc. All Rights Reserved.
This software is PROPRIETARY & CONFIDENTIAL to Analog Devices, Inc.
and its licensors.
******************************************************************************/
#include <chrono>

#include "adi_3dtof_safety_bubble_detector_node.h"
#include "adtf31xx_sensor_frame_info.h"
#include "module_profile.h"

using namespace std::chrono_literals;
namespace enc = sensor_msgs::image_encodings;

/**
 * @brief Updates algorithm parameters based on dynamic reconfigure.
 *        This function handles algorithm-specific parameters (not sensor capture parameters).
 *        Sensor parameters (AB threshold, confidence threshold) are handled in the input thread.
 */

void ADI3DToFSafetyBubbleDetector::updateDynamicReconfigureVariablesProcessThread()
{
  // Update safety bubble sensitivity if changed
  if (safety_bubble_sensitivity_ != tunable_params_.safety_bubble_detection_sensitivity) {
    safety_bubble_sensitivity_ = tunable_params_.safety_bubble_detection_sensitivity;
    RCLCPP_INFO(this->get_logger(), "Changed Safety bubble detection sensitivity value is %d",
                safety_bubble_sensitivity_);
  }

  // Update RANSAC floor detection flag if changed
  if (enable_ransac_floor_detection_ != tunable_params_.enable_ransac_floor_detection) {
    enable_ransac_floor_detection_ = tunable_params_.enable_ransac_floor_detection;
    RCLCPP_INFO(this->get_logger(), "Enable bit for ransac floor detection is changed to %d",
                enable_ransac_floor_detection_);
  }

  // Update floor paint flag if changed
  if (enable_floor_paint_ != tunable_params_.enable_floor_paint) {
    enable_floor_paint_ = tunable_params_.enable_floor_paint;
    RCLCPP_INFO(this->get_logger(), "Enable bit for floor paint is changed to %d", enable_floor_paint_);
  }

  // Update floor height threshold if changed
  if (floor_height_threshold_mtr_ != tunable_params_.floor_height_threshold_mtr) {
    floor_height_threshold_mtr_ = tunable_params_.floor_height_threshold_mtr;
    RCLCPP_INFO(this->get_logger(), "Floor height threshold is changed to %.3f meters", floor_height_threshold_mtr_);
  }

  // Update safety bubble zone visualization flag if changed
  if (enable_safety_bubble_zone_visualization_ != tunable_params_.enable_safety_bubble_zone_visualization) {
    enable_safety_bubble_zone_visualization_ = tunable_params_.enable_safety_bubble_zone_visualization;
    RCLCPP_INFO(this->get_logger(), "Enable bit for safety bubble zone visualization is changed to %d",
                enable_safety_bubble_zone_visualization_);
  }

  // Update multi-zone configuration from tunable parameters
  auto& zones = multi_zone_config_.getZones();
  if (zones.size() >= 3) {
    bool zones_changed = false;  // Local flag to track zone changes

    // Zone 1
    if (zones[0].shape != (tunable_params_.zone1_shape == 0 ? adi_3dtof_safety_bubble_detector::ZoneShape::CIRCLE : adi_3dtof_safety_bubble_detector::ZoneShape::RECTANGLE) ||
        zones[0].radius_mtr != static_cast<float>(tunable_params_.zone1_size_z_mtr) ||
        zones[0].x_dim_mtr != static_cast<float>(tunable_params_.zone1_size_x_mtr) ||
        zones[0].z_dim_mtr != static_cast<float>(tunable_params_.zone1_size_z_mtr) ||
        zones[0].enabled != tunable_params_.zone1_enabled) {
      zones[0].shape = tunable_params_.zone1_shape == 0 ? adi_3dtof_safety_bubble_detector::ZoneShape::CIRCLE : adi_3dtof_safety_bubble_detector::ZoneShape::RECTANGLE;
      zones[0].radius_mtr = static_cast<float>(tunable_params_.zone1_size_z_mtr);  // Radius stored in size_z
      zones[0].x_dim_mtr = static_cast<float>(tunable_params_.zone1_size_x_mtr);
      zones[0].z_dim_mtr = static_cast<float>(tunable_params_.zone1_size_z_mtr);
      zones[0].enabled = tunable_params_.zone1_enabled;
      zones_changed = true;
      RCLCPP_INFO(this->get_logger(), "Zone 1 updated: shape=%d, z=%.3f m (radius), x=%.3f m, enabled=%d",
                  tunable_params_.zone1_shape, tunable_params_.zone1_size_z_mtr,
                  tunable_params_.zone1_size_x_mtr, tunable_params_.zone1_enabled);
    }

    // Zone 2
    if (zones[1].shape != (tunable_params_.zone2_shape == 0 ? adi_3dtof_safety_bubble_detector::ZoneShape::CIRCLE : adi_3dtof_safety_bubble_detector::ZoneShape::RECTANGLE) ||
        zones[1].radius_mtr != static_cast<float>(tunable_params_.zone2_size_z_mtr) ||
        zones[1].x_dim_mtr != static_cast<float>(tunable_params_.zone2_size_x_mtr) ||
        zones[1].z_dim_mtr != static_cast<float>(tunable_params_.zone2_size_z_mtr) ||
        zones[1].enabled != tunable_params_.zone2_enabled) {
      zones[1].shape = tunable_params_.zone2_shape == 0 ? adi_3dtof_safety_bubble_detector::ZoneShape::CIRCLE : adi_3dtof_safety_bubble_detector::ZoneShape::RECTANGLE;
      zones[1].radius_mtr = static_cast<float>(tunable_params_.zone2_size_z_mtr);  // Radius stored in size_z
      zones[1].x_dim_mtr = static_cast<float>(tunable_params_.zone2_size_x_mtr);
      zones[1].z_dim_mtr = static_cast<float>(tunable_params_.zone2_size_z_mtr);
      zones[1].enabled = tunable_params_.zone2_enabled;
      zones_changed = true;
      RCLCPP_INFO(this->get_logger(), "Zone 2 updated: shape=%d, z=%.3f m (radius), x=%.3f m, enabled=%d",
                  tunable_params_.zone2_shape, tunable_params_.zone2_size_z_mtr,
                  tunable_params_.zone2_size_x_mtr, tunable_params_.zone2_enabled);
    }

    // Zone 3
    if (zones[2].shape != (tunable_params_.zone3_shape == 0 ? adi_3dtof_safety_bubble_detector::ZoneShape::CIRCLE : adi_3dtof_safety_bubble_detector::ZoneShape::RECTANGLE) ||
        zones[2].radius_mtr != static_cast<float>(tunable_params_.zone3_size_z_mtr) ||
        zones[2].x_dim_mtr != static_cast<float>(tunable_params_.zone3_size_x_mtr) ||
        zones[2].z_dim_mtr != static_cast<float>(tunable_params_.zone3_size_z_mtr) ||
        zones[2].enabled != tunable_params_.zone3_enabled) {
      zones[2].shape = tunable_params_.zone3_shape == 0 ? adi_3dtof_safety_bubble_detector::ZoneShape::CIRCLE : adi_3dtof_safety_bubble_detector::ZoneShape::RECTANGLE;
      zones[2].radius_mtr = static_cast<float>(tunable_params_.zone3_size_z_mtr);  // Radius stored in size_z
      zones[2].x_dim_mtr = static_cast<float>(tunable_params_.zone3_size_x_mtr);
      zones[2].z_dim_mtr = static_cast<float>(tunable_params_.zone3_size_z_mtr);
      zones[2].enabled = tunable_params_.zone3_enabled;
      zones_changed = true;
      RCLCPP_INFO(this->get_logger(), "Zone 3 updated: shape=%d, z=%.3f m (radius), x=%.3f m, enabled=%d",
                  tunable_params_.zone3_shape, tunable_params_.zone3_size_z_mtr,
                  tunable_params_.zone3_size_x_mtr, tunable_params_.zone3_enabled);
    }

    // Check if zones changed and reinitialize MultiZoneDetector if needed
    if (zones_changed && multi_zone_detector_) {
      multi_zone_detector_->initialize(multi_zone_config_);
      
      // Invalidate visualization cache so it gets rebuilt with new zone config
      visualization_cache_valid_ = false;
      
      RCLCPP_INFO(this->get_logger(), "MultiZoneDetector reinitialized due to zone parameter changes");
    }
  }
}

/**
 *
 * @brief This function is the entry point to the safety bubble algorithm
 *
 *
 */
bool ADI3DToFSafetyBubbleDetector::runSafetyBubbleDetection()
{
  // nullptr checks
  if (
    (camera_map_tf_listener_ == nullptr) || (optical_map_tf_listener_ == nullptr) ||
    (vcam_tf_listener_ == nullptr) || (vcam_depth_frame_8bpp_ == nullptr)) {
    return false;
  }

  // Update dynamic reconfigure values.
  updateDynamicReconfigureVariablesProcessThread();

  // Get frame from sensor.
  PROFILE_FUNCTION_START(SafetyBubble_GetNextFrame)
  ADTF31xxSensorFrameInfo * inframe = nullptr;
  try {
    inframe = safetyBubbleDetectorIOThreadGetNextFrame();
  } catch (const std::exception & e) {
    std::cerr << e.what() << '\n';
  }

  if (inframe == nullptr) {
    return false;
  }

  RCLCPP_INFO_STREAM(
    this->get_logger(), "adi_3dtof_safety_bubble_detector::Running loop : " << frame_number_);

  depth_frame_ = inframe->getDepthFrame();
  ab_frame_ = inframe->getIRFrame();
  xyz_frame_ = inframe->getXYZFrame();
  compressed_depth_frame_ = inframe->getCompressedDepthFrame();
  compressed_ab_frame_ = inframe->getCompressedIRFrame();
  compressed_depth_frame_size_ = inframe->getCompressedDepthFrameSize();
  compressed_ab_frame_size_ = inframe->getCompressedIRFrameSize();
  // Set global timestamp
  curr_frame_timestamp_ = inframe->getFrameTimestamp();

  short * ransac_input_xyz_frame = inframe->getXYZFrame();
  if (camera_tilted_) {
    ransac_input_xyz_frame = inframe->getRotatedXYZFrame();
  }

  if ((depth_frame_ == nullptr) || (ab_frame_ == nullptr)) {
    return false;
  }
  PROFILE_FUNCTION_END(SafetyBubble_GetNextFrame)

  ADI3DToFSafetyBubbleDetectorOutputInfo * new_output_frame =
    new ADI3DToFSafetyBubbleDetectorOutputInfo(image_width_, image_height_);
  if (new_output_frame == nullptr) {
    return false;
  }

  // Assign pointers
  vcam_depth_frame_ = new_output_frame->vcam_depth_frame_;
  vcam_depth_image_floor_pixels_removed_8bpp_ =
    new_output_frame->vcam_depth_image_floor_pixels_removed_8bpp_;
  vcam_depth_frame_with_floor_ = new_output_frame->vcam_depth_frame_with_floor_;
  depth_frame_with_floor_ = new_output_frame->depth_frame_with_floor_;

  // nullptr checks
  if (
    (new_output_frame->depth_frame_ == nullptr) || (new_output_frame->ab_frame_ == nullptr) ||
    (new_output_frame->xyz_frame_ == nullptr)) {
    return false;
  }

  if (
    (vcam_depth_frame_ == nullptr) || (vcam_depth_image_floor_pixels_removed_8bpp_ == nullptr) ||
    (vcam_depth_frame_with_floor_ == nullptr) || (depth_frame_with_floor_ == nullptr)) {
    return false;
  }

  PROFILE_FUNCTION_START(SafetyBubble_RUN)

  // Store the original depth frame
  memcpy(
    depth_frame_with_floor_, depth_frame_, image_width_ * image_height_ * sizeof(depth_frame_[0]));

  // Floor Removal using modified PCL RANSAC
  // Default values for RANSAC debug outputs
  ransac_iterations_ = 0;
  noise_count_ = 0;
  ransac_time_ms_ = 0;
  FilterFlag filter_flag = RemoveFloorFromDepthImage;

  // int64 e1 = cv::getTickCount();
  PROFILE_FUNCTION_START(SafetyBubble_FLOOR_DETECTION)
  if (enable_ransac_floor_detection_) {
    // Process
    ransac_floor_detection_status_ = floor_plane_detection_->detectFloorUsingEnhancedRANSAC(
      &depth_frame_[0], &xyz_frame_[0], filter_flag, nullptr, nullptr, ransac_input_xyz_frame,
      inframe->getModifiedXYZFrame(), inframe->getModifiedXYZFrameSize(),
      inframe->getNearbyObjectsFoundFlag());
#if 0
    if (ransac_floor_detection_status_)
    {
      ransac_iterations_ = floor_plane_detection_->getRansacIterations();
      noise_count_ = floor_plane_detection_->getNoiseCount();
    }
#endif
  }

  if ((!enable_ransac_floor_detection_) || (!ransac_floor_detection_status_)) {
    std::cout << "Running traditional floor detection algorithm.\n" << std::endl;
    ransac_floor_detection_status_ = false;
  }
  PROFILE_FUNCTION_END(SafetyBubble_FLOOR_DETECTION)
  // int64 e2 = cv::getTickCount();
  // ransac_time_ms_ = ((e2 - e1) * 1000.0f) / cv::getTickFrequency();

  // Transform to Virtual cam view.
  PROFILE_FUNCTION_START(SafetyBubble_TRANSFORM2VCAM)
  convertDepthCamToVirtualCamFrame();
  PROFILE_FUNCTION_END(SafetyBubble_TRANSFORM2VCAM)

  // Perform detection.
  PROFILE_FUNCTION_START(SafetyBubble_SAFETYBUBBLEDETECTION)
  last_detection_result_ = safetyBubbleDetection();
  PROFILE_FUNCTION_END(SafetyBubble_SAFETYBUBBLEDETECTION)

  // Publish zone status immediately after detection
  // Zone status includes per-zone detection information
  publishZoneStatus(last_detection_result_);

  // The generate visualization and publish topics are run in a different thread,
  // so, copy the buffers needed by these functions to a queue
  if (new_output_frame != nullptr) {
    new_output_frame->frame_number_ = frame_number_;
    new_output_frame->detection_result_ = last_detection_result_;  // Pass detection results via queue
    new_output_frame->ransac_floor_detection_status_ = ransac_floor_detection_status_;
    new_output_frame->ransac_iterations_ = ransac_iterations_;
    new_output_frame->noise_count_ = noise_count_;
    new_output_frame->ransac_time_ms_ = ransac_time_ms_;
    new_output_frame->compressed_depth_frame_size_ = compressed_depth_frame_size_;
    new_output_frame->compressed_ab_frame_size_ = compressed_ab_frame_size_;
    memcpy(
      new_output_frame->depth_frame_, depth_frame_,
      image_width_ * image_height_ * sizeof(depth_frame_[0]));
    memcpy(
      new_output_frame->ab_frame_, ab_frame_, image_width_ * image_height_ * sizeof(ab_frame_[0]));
    memcpy(
      new_output_frame->compressed_depth_frame_, compressed_depth_frame_,
      2 * image_width_ * image_height_ * sizeof(compressed_depth_frame_[0]));
    memcpy(
      new_output_frame->compressed_ab_frame_, compressed_ab_frame_,
      2 * image_width_ * image_height_ * sizeof(compressed_ab_frame_[0]));
    memcpy(
      new_output_frame->xyz_frame_, xyz_frame_,
      3 * image_width_ * image_height_ * sizeof(xyz_frame_[0]));
    // Push
    safetyBubbleDetectorIOThreadPushOutputNode(new_output_frame);
  }

  frame_number_++;

  // dispose the frame
  delete inframe;

  PROFILE_FUNCTION_END(SafetyBubble_RUN)
  return true;
}

/**
 * @brief This function gives TF of base frame to virtual camera frame.
 *
 */
void ADI3DToFSafetyBubbleDetector::getCameraLinksTF()
{
  // Get RPY
  geometry_msgs::msg::TransformStamped camera_map_transform;
  try {
    camera_map_transform =
      camera_map_tf_buffer_->lookupTransform("map", camera_link_, tf2::TimePointZero, 5s);
  } catch (tf2::TransformException & ex) {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Camera to Map TF2 Error: %s\n", ex.what());
    return;
  }

  // Camera Tilt
  // Ref : https://en.wikipedia.org/wiki/Rotation_matrix
  tf2::Quaternion camera_map_qt(
    camera_map_transform.transform.rotation.x, camera_map_transform.transform.rotation.y,
    camera_map_transform.transform.rotation.z, camera_map_transform.transform.rotation.w);
  tf2::Matrix3x3 camera_map_rotation_matrix(camera_map_qt);
  double roll, pitch, yaw;
  /*
  Input Pitch should be always in (-1.57, 1.57).
  For the |pitch| > 1.57, please consider taking (3.14 +/- pitch) as output here.
  For input pitch |1.57|, output will be  wrong because cosβ = 0
  */
  camera_map_rotation_matrix.getRPY(roll, pitch, yaw);
  camera_roll_rad_ = roll;
  camera_pitch_rad_ = pitch;
  camera_yaw_rad_ = yaw;

  camera_tilted_ = false;

  // Yaw is neglected as it does not affect the algorithm
  if (
    (std::fabs(camera_roll_rad_) >= std::numeric_limits<float>::epsilon()) ||
    (std::fabs(camera_pitch_rad_) >= std::numeric_limits<float>::epsilon())) {
    camera_tilted_ = true;
  }

  // Get reverse rotation matrix
  geometry_msgs::msg::TransformStamped optical_map_transform;
  try {
    optical_map_transform =
      optical_map_tf_buffer_->lookupTransform("map", optical_camera_link_, tf2::TimePointZero, 5s);
  } catch (tf2::TransformException & ex) {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Optical to Map TF2 Error: %s\n", ex.what());
    return;
  }

  tf2::Quaternion optical_map_qt(
    optical_map_transform.transform.rotation.x, optical_map_transform.transform.rotation.y,
    optical_map_transform.transform.rotation.z, optical_map_transform.transform.rotation.w);
  tf2::Matrix3x3 cam_rotation_matrix(optical_map_qt);

  depth_extrinsics_external_.translation_matrix[0] = optical_map_transform.transform.translation.x;
  depth_extrinsics_external_.translation_matrix[1] = optical_map_transform.transform.translation.y;
  depth_extrinsics_external_.translation_matrix[2] = optical_map_transform.transform.translation.z;

  // Rotation Matrix
  float optical_map_rotation_matrix[9];
  int j = 0;
  for (int i = 0; i < 3; i++) {
    for (int k = 0; k < 3; k++) {
      optical_map_rotation_matrix[j++] = cam_rotation_matrix.getRow(i)[k];
    }
  }

  // Point Cloud reference : forward -> +Z, Right -> +X, Down -> +Y
  // ROS reference : forward -> +X, Left -> +Y, Up -> +Z
  // Multiplying with constant matrix which rotates the point cloud in Z -90 and X -90 degrees
  // Final Rotation Matrix = reference_reverse_rotation_matrix * optical_map_rotation_matrix
  float reference_reverse_rotation_matrix[9] = {0, -1, 0, 0, 0, -1, 1, 0, 0};
  image_proc_utils_->matrixMultiplication(
    &reference_reverse_rotation_matrix[0], 3, 3, &optical_map_rotation_matrix[0], 3, 3,
    &depth_extrinsics_external_.rotation_matrix[0]);

  /*Camera height from ground*/
  camera_height_mtr_ = optical_map_transform.transform.translation.z;

  geometry_msgs::msg::TransformStamped vcam_transform;
  try {
    vcam_transform = vcam_tf_buffer_->lookupTransform(
      virtual_camera_link_, optical_camera_link_, tf2::TimePointZero, 1000ms);
  } catch (tf2::TransformException & ex) {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "VCAM TF2 Error: %s\n", ex.what());
    return;
  }

  vcam_extrinsics_.translation_matrix[0] = vcam_transform.transform.translation.x;
  vcam_extrinsics_.translation_matrix[1] = vcam_transform.transform.translation.y;
  vcam_extrinsics_.translation_matrix[2] = vcam_transform.transform.translation.z;

  tf2::Quaternion vcam_qt(
    vcam_transform.transform.rotation.x, vcam_transform.transform.rotation.y,
    vcam_transform.transform.rotation.z, vcam_transform.transform.rotation.w);
  tf2::Matrix3x3 vcam_rotation_matrix(vcam_qt);

  int m = 0;
  for (int i = 0; i < 3; i++) {
    for (int k = 0; k < 3; k++) {
      vcam_extrinsics_.rotation_matrix[m++] = vcam_rotation_matrix.getRow(i)[k];
    }
  }
}

/**
 * @brief This function converts depth image from base frame to virtual camera frame.
 *
 */
void ADI3DToFSafetyBubbleDetector::convertDepthCamToVirtualCamFrame()
{
  ADIImage in_img;
  in_img.bpp = 16;
  in_img.roi = nullptr;
  in_img.width = image_width_;
  in_img.height = image_height_;

  ADIImage out_img = in_img;

  // Convert Translation vector values to mm.
  CameraExtrinsics vcam_extrinsics = vcam_extrinsics_;
  for (float & i : vcam_extrinsics.translation_matrix) {
    i *= 1000;
  }

  if (enable_floor_paint_) {
    if (ransac_floor_detection_status_) {
      in_img.data = depth_frame_with_floor_;
    } else {
      in_img.data = depth_frame_;
    }

    out_img.data = vcam_depth_frame_with_floor_;
    memset(vcam_depth_frame_with_floor_, 0, image_width_ * image_height_ * 2);
    image_proc_utils_->transformFrame(
      &in_img, &out_img, &depth_intrinsics_, &depth_extrinsics_, &vcam_intrinsics_,
      &vcam_extrinsics, &valid_roi_, compute_point_cloud_enable_, xyz_frame_);
  }

  in_img.data = depth_frame_;
  out_img.data = vcam_depth_frame_;
  memset(vcam_depth_frame_, 0, image_width_ * image_height_ * 2);

  if (ransac_floor_detection_status_) {
    image_proc_utils_->transformFrame(
      &in_img, &out_img, &depth_intrinsics_, &depth_extrinsics_, &vcam_intrinsics_,
      &vcam_extrinsics, &valid_roi_, compute_point_cloud_enable_, xyz_frame_);
  } else {
    image_proc_utils_->transformFrameWithFloorRemoval(
      &in_img, &out_img, &depth_intrinsics_, &depth_extrinsics_, &vcam_intrinsics_,
      &vcam_extrinsics, &valid_roi_, floor_height_threshold_mtr_, virtual_camera_height_mtr_,
      xyz_frame_, compute_point_cloud_enable_);
  }
}

/**
 *@brief Calculates and stores pixels_per_meter_ for zone size calculations.
 *       This is used by MultiZoneDetector to convert zone dimensions
 *       from meters/cm to pixels.
 *
 *@return Zone radius in pixels for zone 1 (for backward compatibility).
 */
/**
 *@brief This function detects objects in all safety zones using MultiZoneDetector.
 *
 *@return MultiZoneDetectionResult containing per-zone detection status.
 */
adi_3dtof_safety_bubble_detector::MultiZoneDetectionResult ADI3DToFSafetyBubbleDetector::safetyBubbleDetection()
{
  // init
  memset(
    vcam_depth_image_floor_pixels_removed_8bpp_, 0,
    image_width_ * image_height_ * sizeof(vcam_depth_image_floor_pixels_removed_8bpp_[0]));

  ADIImage in_img;
  in_img.data = vcam_depth_frame_;
  in_img.bpp = 16;
  in_img.width = image_width_;
  in_img.height = image_height_;
  in_img.roi = &valid_roi_;

  ADIImage out_img = in_img;

  // convert to 8 bit image.
  int scale_factor = 8192;
  out_img.data = vcam_depth_image_floor_pixels_removed_8bpp_;
  out_img.bpp = 8;
  ImageProcUtils::convertTo8BppImage(&in_img, &out_img, scale_factor);

  // Use MultiZoneDetector for multi-zone detection
  if (multi_zone_detector_) {
    // Update sensitivity in detector
    multi_zone_detector_->setSensitivity(safety_bubble_sensitivity_);
    
    // Convert ADIImageROI to cv::Rect
    cv::Rect roi(valid_roi_.x, valid_roi_.y, valid_roi_.width, valid_roi_.height);
    
    // Perform multi-zone detection
    auto result = multi_zone_detector_->detectZones(
      vcam_depth_image_floor_pixels_removed_8bpp_, roi);
    
    return result;
  }

  // Fallback: Return empty result if detector not initialized
  adi_3dtof_safety_bubble_detector::MultiZoneDetectionResult empty_result;
  empty_result.timestamp = rclcpp::Clock().now();
  return empty_result;
}
