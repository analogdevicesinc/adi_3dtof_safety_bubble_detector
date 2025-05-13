/******************************************************************************
Copyright (c), 2023 - Analog Devices Inc. All Rights Reserved.
This software is PROPRIETARY & CONFIDENTIAL to Analog Devices, Inc.
and its licensors.
******************************************************************************/
#include "module_profile.h"
#include "adtf31xx_sensor_frame_info.h"
#include "adi_3dtof_safety_bubble_detector_node.h"
#include <boost/chrono.hpp>
#include <boost/thread/thread.hpp>

namespace enc = sensor_msgs::image_encodings;

/**
 * @brief Updates the parameters of algorithm based on dynamic reconfigure.
 *
 */

void ADI3DToFSafetyBubbleDetector::updateDynamicReconfigureVariablesProcessThread()
{
  if ((safety_zone_radius_mtr_ != dynamic_reconfigure_config_.safety_bubble_radius_in_mtr) ||
      (safety_bubble_shape_ != dynamic_reconfigure_config_.shape_of_safety_bubble))
  {
    safety_zone_radius_mtr_ = dynamic_reconfigure_config_.safety_bubble_radius_in_mtr;
    safety_zone_radius_pixels_ = setZoneRadius();
    if (dynamic_reconfigure_config_.shape_of_safety_bubble == 0)
    {
      // Circular Safety Bubble
      safety_bubble_zone_ = cv::Mat::zeros(cv::Size(image_width_, image_height_), CV_8UC1);
      safety_bubble_zone_red_mask_ = cv::Mat::zeros(cv::Size(image_width_, image_height_), CV_8UC3);
      cv::circle(safety_bubble_zone_, cv::Point(image_width_ / 2, image_height_ / 2), safety_zone_radius_pixels_, 255,
                 -1);
      cv::circle(safety_bubble_zone_red_mask_, cv::Point(image_width_ / 2, image_height_ / 2),
                 safety_zone_radius_pixels_, cv::Scalar(0, 0, 255), -1);
    }
    else if (dynamic_reconfigure_config_.shape_of_safety_bubble == 1)
    {
      // Rectangular Safety Bubble
      cv::Point top_left((image_width_ / 2) - safety_zone_radius_pixels_,
                         (image_height_ / 2) - safety_zone_radius_pixels_);
      cv::Point bottom_right((image_width_ / 2) + safety_zone_radius_pixels_,
                             (image_height_ / 2) + safety_zone_radius_pixels_);

      safety_bubble_zone_ = cv::Mat::zeros(cv::Size(image_width_, image_height_), CV_8UC1);
      safety_bubble_zone_red_mask_ = cv::Mat::zeros(cv::Size(image_width_, image_height_), CV_8UC3);
      cv::rectangle(safety_bubble_zone_, top_left, bottom_right, cv::Scalar(255, 255, 255), -1, 8, 0);
      cv::rectangle(safety_bubble_zone_red_mask_, top_left, bottom_right, cv::Scalar(0, 0, 255), -1, 8, 0);
    }
  }

  safety_bubble_shape_ = dynamic_reconfigure_config_.shape_of_safety_bubble;
  safety_bubble_sensitivity_ = dynamic_reconfigure_config_.safety_bubble_detection_sensitivity;
  enable_ransac_floor_detection_ = dynamic_reconfigure_config_.enable_ransac_floor_detection;
  enable_floor_paint_ = dynamic_reconfigure_config_.enable_floor_paint;
  enable_safety_bubble_zone_visualization_ = dynamic_reconfigure_config_.enable_safety_bubble_zone_visualization;
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
  if ((camera_map_tf_listener_ == nullptr) || (optical_map_tf_listener_ == nullptr) || (vcam_tf_listener_ == nullptr) ||
      (vcam_depth_frame_8bpp_ == nullptr))
  {
    return false;
  }

  // Update dynamic reconfigure values.
  updateDynamicReconfigureVariablesProcessThread();

  // Get frame from sensor.
  PROFILE_FUNCTION_START(SafetyBubble_GetNextFrame)
  ADTF31xxSensorFrameInfo* inframe = nullptr;
  try
  {
    inframe = safetyBubbleDetectorIOThreadGetNextFrame();
  }
  catch (const std::exception& e)
  {
    std::cerr << e.what() << '\n';
  }

  if (inframe == nullptr)
  {
    return false;
  }

  depth_frame_ = inframe->getDepthFrame();
  ab_frame_ = inframe->getIRFrame();
  xyz_frame_ = inframe->getXYZFrame();
  compressed_depth_frame_ = inframe->getCompressedDepthFrame();
  compressed_ab_frame_ = inframe->getCompressedIRFrame();
  compressed_depth_frame_size_ = inframe->getCompressedDepthFrameSize();
  compressed_ab_frame_size_ = inframe->getCompressedIRFrameSize();
  // Set global timestamp
  curr_frame_timestamp_ = inframe->getFrameTimestamp();

  short* ransac_input_xyz_frame = inframe->getXYZFrame();
  if (camera_tilted_)
  {
    ransac_input_xyz_frame = inframe->getRotatedXYZFrame();
  }

  if ((depth_frame_ == nullptr) || (ab_frame_ == nullptr))
  {
    return false;
  }
  PROFILE_FUNCTION_END(SafetyBubble_GetNextFrame)

  ADI3DToFSafetyBubbleDetectorOutputInfo* new_output_frame =
      new ADI3DToFSafetyBubbleDetectorOutputInfo(image_width_, image_height_);
  if (new_output_frame == nullptr)
  {
    return false;
  }

  // Assign pointers
  vcam_depth_frame_ = new_output_frame->vcam_depth_frame_;
  vcam_depth_image_floor_pixels_removed_8bpp_ = new_output_frame->vcam_depth_image_floor_pixels_removed_8bpp_;
  vcam_depth_frame_with_floor_ = new_output_frame->vcam_depth_frame_with_floor_;
  depth_frame_with_floor_ = new_output_frame->depth_frame_with_floor_;

  // nullptr checks
  if ((new_output_frame->depth_frame_ == nullptr) || (new_output_frame->ab_frame_ == nullptr) ||
      (new_output_frame->xyz_frame_ == nullptr))
  {
    return false;
  }

  if ((vcam_depth_frame_ == nullptr) || (vcam_depth_image_floor_pixels_removed_8bpp_ == nullptr) ||
      (vcam_depth_frame_with_floor_ == nullptr) || (depth_frame_with_floor_ == nullptr))
  {
    return false;
  }

  PROFILE_FUNCTION_START(SafetyBubble_RUN)

  // Get camera link and virtual camera link TF
  getCameraLinksTF();

  // Store the original depth frame
  memcpy(depth_frame_with_floor_, depth_frame_, image_width_ * image_height_ * sizeof(depth_frame_[0]));

  // Floor Removal using modified PCL RANSAC
  // Default values for RANSAC debug outputs
  ransac_iterations_ = 0;
  noise_count_ = 0;
  ransac_time_ms_ = 0;
  FilterFlag filter_flag = RemoveFloorFromDepthImage;

  // int64 e1 = cv::getTickCount();
  PROFILE_FUNCTION_START(SafetyBubble_FLOOR_DETECTION)
  if (enable_ransac_floor_detection_)
  {
    // Process
    ransac_floor_detection_status_ = floor_plane_detection_->detectFloorUsingEnhancedRANSAC(
        &depth_frame_[0], &xyz_frame_[0], filter_flag, nullptr, nullptr, ransac_input_xyz_frame,
        inframe->getModifiedXYZFrame(), inframe->getModifiedXYZFrameSize(), inframe->getNearbyObjectsFoundFlag());
#if 0
    if (ransac_floor_detection_status_)
    {
      ransac_iterations_ = floor_plane_detection_->getRansacIterations();
      noise_count_ = floor_plane_detection_->getNoiseCount();
    }
#endif
  }

  if ((!enable_ransac_floor_detection_) || (!ransac_floor_detection_status_))
  {
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
  bool object_detected = safetyBubbleDetection();
  PROFILE_FUNCTION_END(SafetyBubble_SAFETYBUBBLEDETECTION)

  // Publish output flag here,
  // this ensures that we publish the detection flag immediately without waiting for any further processing.
  // Other optional messages ar published in an other thread.
  std_msgs::Bool obj_detect;
  obj_detect.data = object_detected;
  object_detected_publisher_.publish(obj_detect);

  // The generate visualization and publish topics are run in a different thread,
  // so, copy the buffers needed by these functions to a queue
  if (new_output_frame != nullptr)
  {
    new_output_frame->frame_number_ = frame_number_;
    new_output_frame->object_detected_ = object_detected;
    new_output_frame->ransac_floor_detection_status_ = ransac_floor_detection_status_;
    new_output_frame->ransac_iterations_ = ransac_iterations_;
    new_output_frame->noise_count_ = noise_count_;
    new_output_frame->ransac_time_ms_ = ransac_time_ms_;
    new_output_frame->compressed_depth_frame_size_ = compressed_depth_frame_size_;
    new_output_frame->compressed_ab_frame_size_ = compressed_ab_frame_size_;
    memcpy(new_output_frame->depth_frame_, depth_frame_, image_width_ * image_height_ * sizeof(depth_frame_[0]));
    memcpy(new_output_frame->ab_frame_, ab_frame_, image_width_ * image_height_ * sizeof(ab_frame_[0]));
    memcpy(new_output_frame->compressed_depth_frame_, compressed_depth_frame_,
           2 * image_width_ * image_height_ * sizeof(compressed_depth_frame_[0]));
    memcpy(new_output_frame->compressed_ab_frame_, compressed_ab_frame_,
           2 * image_width_ * image_height_ * sizeof(compressed_ab_frame_[0]));
    memcpy(new_output_frame->xyz_frame_, xyz_frame_, 3 * image_width_ * image_height_ * sizeof(xyz_frame_[0]));
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
  geometry_msgs::TransformStamped camera_map_transform;
  try
  {
    camera_map_transform =
        camera_map_tf_buffer_.lookupTransform("map", camera_link_, ros::Time(0.0f), ros::Duration(1.0f));
  }
  catch (tf2::TransformException& ex)
  {
    ROS_ERROR("Camera to Map TF2 Error: %s\n", ex.what());
    return;
  }

  // Camera Tilt
  // Ref : https://en.wikipedia.org/wiki/Rotation_matrix
  tf2::Quaternion camera_map_qt(camera_map_transform.transform.rotation.x, camera_map_transform.transform.rotation.y,
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
  if ((std::fabs(camera_roll_rad_) >= std::numeric_limits<float>::epsilon()) ||
      (std::fabs(camera_pitch_rad_) >= std::numeric_limits<float>::epsilon()))
  {
    camera_tilted_ = true;
  }

  // Get reverse rotation matrix
  geometry_msgs::TransformStamped optical_map_transform;
  try
  {
    optical_map_transform =
        optical_map_tf_buffer_.lookupTransform("map", optical_camera_link_, ros::Time(0.0f), ros::Duration(1.0f));
  }
  catch (tf2::TransformException& ex)
  {
    ROS_ERROR("Optical to Map TF2 Error: %s\n", ex.what());
    return;
  }

  tf2::Quaternion optical_map_qt(optical_map_transform.transform.rotation.x, optical_map_transform.transform.rotation.y,
                                 optical_map_transform.transform.rotation.z,
                                 optical_map_transform.transform.rotation.w);
  tf2::Matrix3x3 cam_rotation_matrix(optical_map_qt);

  depth_extrinsics_external_.translation_matrix[0] = optical_map_transform.transform.translation.x;
  depth_extrinsics_external_.translation_matrix[1] = optical_map_transform.transform.translation.y;
  depth_extrinsics_external_.translation_matrix[2] = optical_map_transform.transform.translation.z;

  // Rotation Matrix
  float optical_map_rotation_matrix[9];
  int j = 0;
  for (int i = 0; i < 3; i++)
  {
    for (int k = 0; k < 3; k++)
    {
      optical_map_rotation_matrix[j++] = cam_rotation_matrix.getRow(i)[k];
    }
  }

  // Point Cloud reference : forward -> +Z, Right -> +X, Down -> +Y
  // ROS reference : forward -> +X, Left -> +Y, Up -> +Z
  // Multiplying with constant matrix which rotates the point cloud in Z -90 and X -90 degrees
  // Final Rotation Matrix = reference_reverse_rotation_matrix * optical_map_rotation_matrix
  float reference_reverse_rotation_matrix[9] = { 0, -1, 0, 0, 0, -1, 1, 0, 0 };
  image_proc_utils_->matrixMultiplication(&reference_reverse_rotation_matrix[0], 3, 3, &optical_map_rotation_matrix[0],
                                          3, 3, &depth_extrinsics_external_.rotation_matrix[0]);

  /*Camera height from ground*/
  camera_height_mtr_ = optical_map_transform.transform.translation.z;

  geometry_msgs::TransformStamped vcam_transform;
  try
  {
    vcam_transform = vcam_tf_buffer_.lookupTransform(virtual_camera_link_, optical_camera_link_, ros::Time(0.0f),
                                                     ros::Duration(1.0f));
  }
  catch (tf2::TransformException& ex)
  {
    ROS_ERROR("VCAM TF2 Error: %s\n", ex.what());
    return;
  }

  vcam_extrinsics_.translation_matrix[0] = vcam_transform.transform.translation.x;
  vcam_extrinsics_.translation_matrix[1] = vcam_transform.transform.translation.y;
  vcam_extrinsics_.translation_matrix[2] = vcam_transform.transform.translation.z;

  tf2::Quaternion vcam_qt(vcam_transform.transform.rotation.x, vcam_transform.transform.rotation.y,
                          vcam_transform.transform.rotation.z, vcam_transform.transform.rotation.w);
  tf2::Matrix3x3 vcam_rotation_matrix(vcam_qt);

  int m = 0;
  for (int i = 0; i < 3; i++)
  {
    for (int k = 0; k < 3; k++)
    {
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
  for (float& i : vcam_extrinsics.translation_matrix)
  {
    i *= 1000;
  }

  if (enable_floor_paint_)
  {
    if (ransac_floor_detection_status_)
    {
      in_img.data = depth_frame_with_floor_;
    }
    else
    {
      in_img.data = depth_frame_;
    }

    out_img.data = vcam_depth_frame_with_floor_;
    memset(vcam_depth_frame_with_floor_, 0, image_width_ * image_height_ * 2);
    image_proc_utils_->transformFrame(&in_img, &out_img, &depth_intrinsics_, &depth_extrinsics_, &vcam_intrinsics_,
                                      &vcam_extrinsics, &valid_roi_, compute_point_cloud_enable_, xyz_frame_);
  }

  in_img.data = depth_frame_;
  out_img.data = vcam_depth_frame_;
  memset(vcam_depth_frame_, 0, image_width_ * image_height_ * 2);

  if (ransac_floor_detection_status_)
  {
    image_proc_utils_->transformFrame(&in_img, &out_img, &depth_intrinsics_, &depth_extrinsics_, &vcam_intrinsics_,
                                      &vcam_extrinsics, &valid_roi_, compute_point_cloud_enable_, xyz_frame_);
  }
  else
  {
    image_proc_utils_->transformFrameWithFloorRemoval(
        &in_img, &out_img, &depth_intrinsics_, &depth_extrinsics_, &vcam_intrinsics_, &vcam_extrinsics, &valid_roi_,
        fallback_floor_height_offset_mtr_, virtual_camera_height_mtr_, xyz_frame_, compute_point_cloud_enable_);
  }
}

/**
 *@brief Sets the number of pixels for a particular in Zone radius.
 *
 *@return Zone radius in pixels.
 */
int ADI3DToFSafetyBubbleDetector::setZoneRadius()
{
  // Obtain focal lengths from intrinsic camera matrix, K.

  float focal_length_x = vcam_intrinsics_.camera_matrix[0];
  float focal_length_y = vcam_intrinsics_.camera_matrix[4];
  float focal_length = (focal_length_x + focal_length_y) / 2.0f;

  // Calculate horizontal Field-of-View (radians), length opposite FoV angle, and number of pixels-per-metre.
  float fov = 2.0f * atan2(image_width_ / 2.0f, focal_length);
  float fov_in_meters = 2.0f * virtual_camera_height_mtr_ * tan(fov / 2.0f);
  float pixels_per_meter = image_width_ / fov_in_meters;

  // Set number of pixels for zone radius based on pre-defined radius in metres.
  return ((int)(pixels_per_meter * safety_zone_radius_mtr_));
}

/**
 *@brief This function detects objects in safety bubble.
 *
 *@return true if object is present in safety bubble.
 *@return false if object is absent in safety bubble.
 */
bool ADI3DToFSafetyBubbleDetector::safetyBubbleDetection()
{
  // init
  memset(vcam_depth_image_floor_pixels_removed_8bpp_, 0,
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

  // Mask with safety zone and find number of non-zero pixels.
  cv::Mat m_vcam_final_image;
  m_vcam_final_image =
      cv::Mat(cv::Size(image_width_, image_height_), CV_8UC1, vcam_depth_image_floor_pixels_removed_8bpp_);

  // Create ROI images for anding.
  cv::Mat m_vcam_final_image_roi =
      m_vcam_final_image(cv::Rect(valid_roi_.x, valid_roi_.y, valid_roi_.width, valid_roi_.height));

  cv::Mat safety_bubble_zone_roi =
      safety_bubble_zone_(cv::Rect(valid_roi_.x, valid_roi_.y, valid_roi_.width, valid_roi_.height));

  cv::Mat masked_image_roi;
  cv::bitwise_and(m_vcam_final_image_roi, safety_bubble_zone_roi, masked_image_roi);

  bool object_detected = false;

#if 1
  // CCL
  // Binarize
  cv::Mat masked_image_roi_bin;
  cv::threshold(masked_image_roi, masked_image_roi_bin, 0, 255, cv::THRESH_BINARY);

  // Perform cc
  cv::Mat labels, stats, centroids;
  int num_lables =
      cv::connectedComponentsWithStats(masked_image_roi_bin, labels, stats, centroids, 4, CV_16U, cv::CCL_WU);
  // First label is background, so ignore
  for (int i = 1; i < num_lables; i++)
  {    
    int blob_area = stats.at<int>(i, cv::CC_STAT_AREA);
    if (blob_area > safety_bubble_sensitivity_)
    {
      object_detected = true;
      break;
    }
  }
#else
  int rows = valid_roi_.height;
  int cols = valid_roi_.width;
  if (!masked_image_roi.empty())
  {
    unsigned char* ptr = (unsigned char*)masked_image_roi.data;
    for (int i = 0; i < (rows * cols); i++)
    {
      if (*ptr++ != 0)
      {
        object_detected = true;
        break;
      }
    }
  }
#endif

  return object_detected;
}
