/******************************************************************************
Copyright (c), 2023 - Analog Devices Inc. All Rights Reserved.
This software is PROPRIETARY & CONFIDENTIAL to Analog Devices, Inc.
and its licensors.
******************************************************************************/
#include "adi_3dtof_safety_bubble_detector_node.h"

/**
 * @brief This image fills and publishes the camera information
 *
 * @param frame_id  frame_id of camera_info
 * @param publisher This is Ros publisher
 */
void ADI3DToFSafetyBubbleDetector::fillAndPublishCameraInfo(const std::string& frame_id,
                                                            const ros::Publisher& publisher)
{
  cam_info_msg_.header.seq = input_sensor_->getFrameCounter();
  cam_info_msg_.header.stamp = curr_frame_timestamp_;
  cam_info_msg_.header.frame_id = frame_id;

  cam_info_msg_.width = image_width_;
  cam_info_msg_.height = image_height_;

  cam_info_msg_.distortion_model = sensor_msgs::distortion_models::RATIONAL_POLYNOMIAL;

  cam_info_msg_.K.fill(0.0f);
  cam_info_msg_.K[0] = depth_intrinsics_.camera_matrix[0];
  cam_info_msg_.K[2] = depth_intrinsics_.camera_matrix[2];
  cam_info_msg_.K[4] = depth_intrinsics_.camera_matrix[4];
  cam_info_msg_.K[5] = depth_intrinsics_.camera_matrix[5];
  cam_info_msg_.K[8] = 1.0f;

  cam_info_msg_.P.fill(0.0);
  cam_info_msg_.P[0] = depth_intrinsics_.camera_matrix[0];
  cam_info_msg_.P[2] = depth_intrinsics_.camera_matrix[2];
  cam_info_msg_.P[3] = depth_extrinsics_.translation_matrix[0];
  cam_info_msg_.P[5] = depth_intrinsics_.camera_matrix[4];
  cam_info_msg_.P[6] = depth_intrinsics_.camera_matrix[5];
  cam_info_msg_.P[7] = depth_extrinsics_.translation_matrix[1];
  cam_info_msg_.P[10] = 1.0f;
  cam_info_msg_.P[11] = depth_extrinsics_.translation_matrix[2];

  cam_info_msg_.D.resize(0);
  for (float distortion_coeff : depth_intrinsics_.distortion_coeffs)
  {
    cam_info_msg_.D.push_back(distortion_coeff);
  }

  cam_info_msg_.R.fill(0.0f);
  for (int i = 0; i < 9; i++)
  {
    cam_info_msg_.R[i] = depth_extrinsics_.rotation_matrix[i];
  }

  cam_info_msg_.binning_x = 0;
  cam_info_msg_.binning_y = 0;
  cam_info_msg_.roi.do_rectify = false;
  cam_info_msg_.roi.height = 0;
  cam_info_msg_.roi.width = 0;
  cam_info_msg_.roi.x_offset = 0;
  cam_info_msg_.roi.y_offset = 0;

  publisher.publish(cam_info_msg_);
}

/**
 * @brief This function publishes a image(of cv::Mat() type) as Ros message.
 *
 * @param img Input image
 * @param encoding_type number of bits used to represent one pixel of image.
 * @param frame_id frame id of image
 * @param publisher ROS publisher handle
 * @param enable_image_compression boolean decides image compression
 */
void ADI3DToFSafetyBubbleDetector::publishImageAsRosMsg(cv::Mat img, const std::string& encoding_type,
                                                        std::string frame_id, const ros::Publisher& publisher,
                                                        bool enable_image_compression)
{
  cv_bridge::CvImagePtr cv_ptr(new cv_bridge::CvImage);

  cv_ptr->encoding = encoding_type;
  cv_ptr->header.seq = input_sensor_->getFrameCounter();
  cv_ptr->header.stamp = curr_frame_timestamp_;
  cv_ptr->header.frame_id = std::move(frame_id);
  cv_ptr->image = std::move(img);

  if (enable_image_compression)
  {
    // We have to send compressed image.
    publisher.publish(cv_ptr->toCompressedImageMsg());
  }
  else
  {
    publisher.publish(cv_ptr->toImageMsg());
  }
}

/**
 * @brief This function publishes the point cloud
 *
 * @param xyz_frame Buffer containing the xyz values in interleaved format
 *
 * Note: Assumes that cam_info_msg_ is already populated
 */
void ADI3DToFSafetyBubbleDetector::publishPointCloud(short* xyz_frame)
{
  sensor_msgs::PointCloud2::Ptr pointcloud_msg(new sensor_msgs::PointCloud2);

  pointcloud_msg->header.seq = input_sensor_->getFrameCounter();
  pointcloud_msg->header.stamp = curr_frame_timestamp_;
  pointcloud_msg->header.frame_id = optical_camera_link_;
  pointcloud_msg->width = image_width_;
  pointcloud_msg->height = image_height_;
  pointcloud_msg->is_dense = false;
  pointcloud_msg->is_bigendian = false;

  // XYZ data from sensor.
  // This data is in 16 bpp format.
  short* xyz_sensor_buf;
  xyz_sensor_buf = xyz_frame;
  sensor_msgs::PointCloud2Modifier pcd_modifier(*pointcloud_msg);
  pcd_modifier.setPointCloud2FieldsByString(1, "xyz");

  sensor_msgs::PointCloud2Iterator<float> iter_x(*pointcloud_msg, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(*pointcloud_msg, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(*pointcloud_msg, "z");
  for (int i = 0; i < image_height_; i++)
  {
    for (int j = 0; j < image_width_; j++)
    {
      *iter_x = (float)(*xyz_sensor_buf++) / 1000.0f;
      *iter_y = (float)(*xyz_sensor_buf++) / 1000.0f;
      *iter_z = (float)(*xyz_sensor_buf++) / 1000.0f;
      ++iter_x;
      ++iter_y;
      ++iter_z;
    }
  }

  // Publisher
  xyz_image_publisher_.publish(pointcloud_msg);
}

/**
 * @brief This function publishes depth image , ir image, point-cloud and camera info.
 *
 * @param depth_frame - Pointer to the depth frame buffer
 * @param ab_frame - Pointer to the ir frame buffer
 * @param vcam_depth_frame - Pointer to the Virtual camera buffer
 * @param xyz_frame - Pointer to the xyz frame buffer
 */
void ADI3DToFSafetyBubbleDetector::publishImageAndCameraInfo(unsigned short* depth_frame, unsigned short* ab_frame,
                                                             unsigned short* vcam_depth_frame, short* /*xyz_frame*/)
{
  // Publish image as Ros message
  cv::Mat m_disp_image_depth, temp_depth, m_disp_image_ir, m_disp_virtual_depth;

  // convert to 16 bit depth and IR image of CV format.
  m_disp_image_depth = cv::Mat(image_height_, image_width_, CV_16UC1, depth_frame);
  m_disp_image_ir = cv::Mat(image_height_, image_width_, CV_16UC1, ab_frame);
  m_disp_virtual_depth = cv::Mat(image_height_, image_width_, CV_16UC1, vcam_depth_frame);

  fillAndPublishCameraInfo(optical_camera_link_, depth_info_publisher_);
  publishImageAsRosMsg(m_disp_image_depth, "mono16", optical_camera_link_, depth_image_publisher_, false);
  publishImageAsRosMsg(m_disp_image_ir, "mono16", optical_camera_link_, ab_image_publisher_, false);

  // fillAndPublishCameraInfo(virtual_camera_link_, vcam_info_publisher_);
  // publishImageAsRosMsg(m_disp_virtual_depth, "mono16", virtual_camera_link_, vcam_depth_image_publisher_,
  // false);

  // publishPointCloud(xyz_frame);
}

/**
 * @brief This function publishes depth image , ir image, and camera info.
 * The images are assumed to be compressed in RVL encoder.
 *
 * @param compressed_depth_frame - Pointer to the depth frame buffer
 * @param compressed_depth_frame_size - Size of the compressed buffer
 * @param compressed_ab_frame - Pointer to the ir frame buffer
 * @param compressed_ab_frame_size - Size of the compressed buffer
 */
void ADI3DToFSafetyBubbleDetector::publishImageAndCameraInfo(unsigned char* compressed_depth_frame,
                                                             int compressed_depth_frame_size,
                                                             unsigned char* compressed_ab_frame,
                                                             int compressed_ab_frame_size)
{
  fillAndPublishCameraInfo(optical_camera_link_, depth_info_publisher_);

  publishRVLCompressedImageAsRosMsg(compressed_depth_frame, compressed_depth_frame_size, "mono16", optical_camera_link_,
                                    depth_image_publisher_);

  publishRVLCompressedImageAsRosMsg(compressed_ab_frame, compressed_ab_frame_size, "mono16", optical_camera_link_,
                                    ab_image_publisher_);
}

/**
 * @brief Generates the output image for visualization.
 *
 * @param vcam_depth_image_floor_pixels_removed_8bpp - Vcam image with floor pixels removed
 * @param vcam_depth_frame_with_floor - Original vcam image
 * @param object_detected - Object detected flag
 * @return Visualization output(cv::Mat() type)
 */
cv::Mat ADI3DToFSafetyBubbleDetector::generateVisualizationImage(
    unsigned char* vcam_depth_image_floor_pixels_removed_8bpp, unsigned short* vcam_depth_frame_with_floor,
    bool object_detected)
{
  // Object to indicate the red zone inside the safety bubble.
  cv::Mat red_zone = cv::Mat::zeros(cv::Size(image_width_, image_height_), CV_8UC1);

  // Object to indicate the green zone inside the safety bubble.
  cv::Mat green_zone = cv::Mat::zeros(cv::Size(image_width_, image_height_), CV_8UC1);

  cv::Mat m_vcam_final_image =
      cv::Mat(cv::Size(image_width_, image_height_), CV_8UC1, vcam_depth_image_floor_pixels_removed_8bpp);

  cv::Mat m_vcam_final_image_roi =
      m_vcam_final_image(cv::Rect(valid_roi_.x, valid_roi_.y, valid_roi_.width, valid_roi_.height));
  cv::Mat safety_bubble_zone_roi =
      safety_bubble_zone_(cv::Rect(valid_roi_.x, valid_roi_.y, valid_roi_.width, valid_roi_.height));
  cv::Mat red_zone_roi = red_zone(cv::Rect(valid_roi_.x, valid_roi_.y, valid_roi_.width, valid_roi_.height));
  cv::Mat green_zone_roi = green_zone(cv::Rect(valid_roi_.x, valid_roi_.y, valid_roi_.width, valid_roi_.height));

  cv::bitwise_and(m_vcam_final_image_roi, safety_bubble_zone_roi, red_zone_roi);
  cv::bitwise_xor(m_vcam_final_image_roi, red_zone_roi, green_zone_roi);

  // Fill Blue channel with zero pixels.
  cv::Mat blue_channel = cv::Mat::zeros(cv::Size(image_width_, image_height_), CV_8UC1);

  if (enable_floor_paint_)
  {
    // OR with floor pixels to make the visualization better
    // Get separate masks of pixels inside and outside safety bubble.
    int scale_factor = 8192;
    ADIImage in_img;
    in_img.data = vcam_depth_frame_with_floor;
    in_img.bpp = 16;
    in_img.width = image_width_;
    in_img.height = image_height_;
    in_img.roi = nullptr;
    ADIImage out_img = in_img;
    out_img.data = vcam_depth_frame_8bpp_;
    out_img.bpp = 8;
    ImageProcUtils::convertTo8BppImage(&in_img, &out_img, scale_factor);
    m_vcam_final_image = cv::Mat(image_height_, image_width_, CV_8UC1, vcam_depth_frame_8bpp_);
    for (int i = 0; i < image_height_; i++)
    {
      for (int j = 0; j < image_width_; j++)
      {
        if (m_vcam_final_image.at<unsigned char>(i, j) != 0)
        {
          if ((green_zone.at<unsigned char>(i, j) == 0) && (red_zone.at<unsigned char>(i, j) == 0))
          {
            green_zone.at<unsigned char>(i, j) = 128;
            red_zone.at<unsigned char>(i, j) = 128;
            blue_channel.at<unsigned char>(i, j) = 128;
          }
        }
      }
    }
  }
  // push 3 zones
  // no object, objects within safety zone, objects outside the safety zone.
  std::vector<cv::Mat> channels;
  channels.push_back(blue_channel);
  channels.push_back(green_zone);
  channels.push_back(red_zone);

  cv::Mat out_visualization_image;
  cv::merge(channels, out_visualization_image);

  // A Box for indicating the detection status(green:empty, red/occupied)
  cv::Rect box = cv::Rect(8, 10, 20, 20);
  cv::Scalar color = cv::Scalar(0, 255, 0);
  if (object_detected)
  {
    color = cv::Scalar(0, 0, 255);
  }
  cv::rectangle(out_visualization_image, box, color, -1, 8, 0);

  if (enable_safety_bubble_zone_visualization_)
  {
    double alpha = 0.12;
    // blend the overlay with the source image
    cv::addWeighted(safety_bubble_zone_red_mask_, alpha, out_visualization_image, 1 - alpha, 0,
                    out_visualization_image);
  }
  return out_visualization_image;
}

/**
 *@brief This function shuts down all the active nodes
 *
 */
void ADI3DToFSafetyBubbleDetector::shutDownAllNodes()
{
  int status = system("rosnode kill -a");
  if (status < 0)
  {
    ROS_INFO_STREAM("Error in \"rosnode kill -a\": " << status);
  }
  ros::shutdown();
}

/**
 * @brief This function publishes a image(of cv::Mat() type) as Ros message.
 *
 * @param compressed_img compressed image
 * @param compressed_img_size size of compressed image
 * @param encoding_type data type of image pixels
 * @param frame_id frame id of ros message
 * @param publisher ros image publisher
 */
void ADI3DToFSafetyBubbleDetector::publishRVLCompressedImageAsRosMsg(unsigned char* compressed_img,
                                                                     int compressed_img_size,
                                                                     const std::string& encoding_type,
                                                                     std::string frame_id,
                                                                     const ros::Publisher& publisher)
{
  cv_bridge::CvImagePtr cv_ptr(new cv_bridge::CvImage);
  sensor_msgs::CompressedImage::Ptr compressed_payload_ptr(new sensor_msgs::CompressedImage());

  compressed_payload_ptr->format = encoding_type + ";compressedDepth rvl";
  compressed_payload_ptr->header.seq = input_sensor_->getFrameCounter();
  compressed_payload_ptr->header.stamp = curr_frame_timestamp_;
  compressed_payload_ptr->header.frame_id = std::move(frame_id);
  compressed_payload_ptr->data.resize(compressed_img_size + 8 + sizeof(compressed_depth_image_transport::ConfigHeader));

  // Image compression configuration
  compressed_depth_image_transport::ConfigHeader compressionConfiguration{};
  compressionConfiguration.format = compressed_depth_image_transport::INV_DEPTH;

  // Header of compressed image of image transport
  float depthQuantization = 0;
  float maximumDepth = 1;

  // Inverse depth quantization parameters
  float depthQuantizationA = depthQuantization * (depthQuantization + 1.0f);
  float depthQuantizationB = 1.0f - depthQuantizationA / maximumDepth;

  // Add coding parameters to header
  compressionConfiguration.depthParam[0] = depthQuantizationA;
  compressionConfiguration.depthParam[1] = depthQuantizationB;

  memcpy(&compressed_payload_ptr->data[0], &compressionConfiguration, sizeof(compressed_depth_image_transport::ConfigHeader));
  memcpy(&compressed_payload_ptr->data[0] + sizeof(compressed_depth_image_transport::ConfigHeader), &image_width_,
         sizeof(int));
  memcpy(&compressed_payload_ptr->data[4] + sizeof(compressed_depth_image_transport::ConfigHeader), &image_height_,
         sizeof(int));

  memcpy(&compressed_payload_ptr->data[8] + sizeof(compressed_depth_image_transport::ConfigHeader), compressed_img,
         compressed_img_size);

  publisher.publish(compressed_payload_ptr);
}

/**
 * @brief new values from dynamic reconfigure are copied to a struture variable here, actual update to individual
 * parameters happens in updateDynamicReconfigureVariablesInputThread and updateDynamicReconfigureVariablesProcessThread
 * functions.
 *
 * @param config Config parameters present in GUI
 * @param level Bit mask related to dynamic reconfigure callback
 */
void ADI3DToFSafetyBubbleDetector::dynamicallyReconfigureVariables(
    adi_3dtof_safety_bubble_detector::SafetyBubbleDetectorParamsConfig& config, uint32_t /*level*/)
{
  // update all the values in dynamic reconfigure.
  dynamic_reconfigure_config_ = config;

  ROS_INFO(
      "Changed Configuration variables enable safety_bubble_shape, safety_bubble_sensitivity, "
      "enable_ransac_floor_detection, enable_floor_paint"
      "enable_safety_bubble_zone_visualization %d %d %d %d %d",
      dynamic_reconfigure_config_.shape_of_safety_bubble,
      dynamic_reconfigure_config_.safety_bubble_detection_sensitivity,
      dynamic_reconfigure_config_.enable_ransac_floor_detection, dynamic_reconfigure_config_.enable_floor_paint,
      dynamic_reconfigure_config_.enable_safety_bubble_zone_visualization);
}

/**
 * @brief Initialize members of dynamic_reconfigure_config_ structure with the values of parameters which are set via
 * launch file, so that launch file takes precedence over dynamic reconfigure.
 *
 */

void ADI3DToFSafetyBubbleDetector::initSettingsForDynamicReconfigure()
{
  dynamic_reconfigure_config_.safety_bubble_radius_in_mtr = safety_zone_radius_mtr_;
  dynamic_reconfigure_config_.shape_of_safety_bubble = safety_bubble_shape_;
  dynamic_reconfigure_config_.safety_bubble_detection_sensitivity = safety_bubble_sensitivity_;
  dynamic_reconfigure_config_.enable_ransac_floor_detection = enable_ransac_floor_detection_;
  dynamic_reconfigure_config_.enable_floor_paint = enable_floor_paint_;
  dynamic_reconfigure_config_.enable_safety_bubble_zone_visualization = enable_safety_bubble_zone_visualization_;
  dynamic_reconfigure_config_.ab_threshold = ab_threshold_;
  dynamic_reconfigure_config_.confidence_threshold = confidence_threshold_;
}