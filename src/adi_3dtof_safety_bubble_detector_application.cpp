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
void ADI3DToFSafetyBubbleDetector::fillAndPublishCameraInfo(
  const std::string & frame_id,
  const rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr publisher)
{
  cam_info_msg_.header.stamp = curr_frame_timestamp_;
  cam_info_msg_.header.frame_id = frame_id;

  cam_info_msg_.width = image_width_;
  cam_info_msg_.height = image_height_;

  cam_info_msg_.distortion_model = sensor_msgs::distortion_models::RATIONAL_POLYNOMIAL;

  cam_info_msg_.k.fill(0.0f);
  cam_info_msg_.k[0] = depth_intrinsics_.camera_matrix[0];
  cam_info_msg_.k[2] = depth_intrinsics_.camera_matrix[2];
  cam_info_msg_.k[4] = depth_intrinsics_.camera_matrix[4];
  cam_info_msg_.k[5] = depth_intrinsics_.camera_matrix[5];
  cam_info_msg_.k[8] = 1.0f;

  cam_info_msg_.p.fill(0.0);
  cam_info_msg_.p[0] = depth_intrinsics_.camera_matrix[0];
  cam_info_msg_.p[2] = depth_intrinsics_.camera_matrix[2];
  cam_info_msg_.p[3] = depth_extrinsics_.translation_matrix[0];
  cam_info_msg_.p[5] = depth_intrinsics_.camera_matrix[4];
  cam_info_msg_.p[6] = depth_intrinsics_.camera_matrix[5];
  cam_info_msg_.p[7] = depth_extrinsics_.translation_matrix[1];
  cam_info_msg_.p[10] = 1.0f;
  cam_info_msg_.p[11] = depth_extrinsics_.translation_matrix[2];

  cam_info_msg_.d.resize(0);
  for (float distortion_coeff : depth_intrinsics_.distortion_coeffs) {
    cam_info_msg_.d.push_back(distortion_coeff);
  }

  cam_info_msg_.r.fill(0.0f);
  for (int i = 0; i < 9; i++) {
    cam_info_msg_.r[i] = depth_extrinsics_.rotation_matrix[i];
  }

  cam_info_msg_.binning_x = 0;
  cam_info_msg_.binning_y = 0;
  cam_info_msg_.roi.do_rectify = false;
  cam_info_msg_.roi.height = 0;
  cam_info_msg_.roi.width = 0;
  cam_info_msg_.roi.x_offset = 0;
  cam_info_msg_.roi.y_offset = 0;

  publisher->publish(cam_info_msg_);
}

/**
 * @brief This function publishes a image(of cv::Mat() type) as Ros message.
 *
 * @param img Input image
 * @param encoding_type number of bits used to represent one pixel of image.
 * @param frame_id frame id of image
 * @param publisher ROS publisher handle
 */
void ADI3DToFSafetyBubbleDetector::publishImageAsRosMsg(
  cv::Mat img, const std::string & encoding_type, std::string frame_id,
  const rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher)
{
  cv_bridge::CvImagePtr cv_ptr(new cv_bridge::CvImage);

  cv_ptr->encoding = encoding_type;
  cv_ptr->header.stamp = curr_frame_timestamp_;
  cv_ptr->header.frame_id = std::move(frame_id);
  cv_ptr->image = std::move(img);

  publisher->publish(*cv_ptr->toImageMsg());
}

/**
 * @brief This function publishes a image(of cv::Mat() type) as Ros message.
 *
 * @param img Input image
 * @param encoding_type number of bits used to represent one pixel of image.
 * @param frame_id frame id of image
 * @param publisher ROS publisher handle
 */
void ADI3DToFSafetyBubbleDetector::publishCompressedImageAsRosMsg(
  cv::Mat img, const std::string & encoding_type, std::string frame_id,
  const rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr publisher)
{
  cv_bridge::CvImagePtr cv_ptr(new cv_bridge::CvImage);

  cv_ptr->encoding = encoding_type;
  cv_ptr->header.stamp = curr_frame_timestamp_;
  cv_ptr->header.frame_id = std::move(frame_id);
  cv_ptr->image = std::move(img);

  publisher->publish(*cv_ptr->toCompressedImageMsg());
}

#ifdef ENA_POINTCLOUD_PUBLISH
/**
 * @brief This function publishes the point cloud
 *
 * @param xyz_frame Buffer containing the xyz values in interleaved format
 *
 * Note: Assumes that cam_info_msg_ is already populated
 */
void ADI3DToFSafetyBubbleDetector::publishPointCloud(short * xyz_frame)
{
  sensor_msgs::msg::PointCloud2::Ptr pointcloud_msg(new sensor_msgs::msg::PointCloud2);

  pointcloud_msg->header.stamp = curr_frame_timestamp_;
  pointcloud_msg->header.frame_id = optical_camera_link_;
  pointcloud_msg->width = image_width_;
  pointcloud_msg->height = image_height_;
  pointcloud_msg->is_dense = false;
  pointcloud_msg->is_bigendian = false;

  // XYZ data from sensor.
  // This data is in 16 bpp format.
  short * xyz_sensor_buf;
  xyz_sensor_buf = xyz_frame;
  sensor_msgs::PointCloud2Modifier pcd_modifier(*pointcloud_msg);
  pcd_modifier.setPointCloud2FieldsByString(1, "xyz");

  sensor_msgs::PointCloud2Iterator<float> iter_x(*pointcloud_msg, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(*pointcloud_msg, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(*pointcloud_msg, "z");
  for (int i = 0; i < image_height_; i++) {
    for (int j = 0; j < image_width_; j++) {
      *iter_x = (float)(*xyz_sensor_buf++) / 1000.0f;
      *iter_y = (float)(*xyz_sensor_buf++) / 1000.0f;
      *iter_z = (float)(*xyz_sensor_buf++) / 1000.0f;
      ++iter_x;
      ++iter_y;
      ++iter_z;
    }
  }

  // Publisher
  xyz_image_publisher_->publish(*pointcloud_msg);
}
#endif /* ENA_POINTCLOUD_PUBLISH */

/**
 * @brief This function publishes depth image , ir image, point-cloud and camera info.
 *
 * @param depth_frame - Pointer to the depth frame buffer
 * @param ab_frame - Pointer to the ir frame buffer
 * @param vcam_depth_frame - Pointer to the Virtual camera buffer
 * @param xyz_frame - Pointer to the xyz frame buffer
 */
void ADI3DToFSafetyBubbleDetector::publishImageAndCameraInfo(
  unsigned short * depth_frame, unsigned short * ab_frame, unsigned short * vcam_depth_frame,
  short * /*xyz_frame*/)
{
  // Publish image as Ros message
  cv::Mat m_disp_image_depth, temp_depth, m_disp_image_ir, m_disp_virtual_depth;

  // convert to 16 bit depth and IR image of CV format.
  m_disp_image_depth = cv::Mat(image_height_, image_width_, CV_16UC1, depth_frame);
  m_disp_image_ir = cv::Mat(image_height_, image_width_, CV_16UC1, ab_frame);
  m_disp_virtual_depth = cv::Mat(image_height_, image_width_, CV_16UC1, vcam_depth_frame);

  fillAndPublishCameraInfo(optical_camera_link_, depth_info_publisher_);
  publishImageAsRosMsg(m_disp_image_depth, "mono16", optical_camera_link_, depth_image_publisher_);
  publishImageAsRosMsg(m_disp_image_ir, "mono16", optical_camera_link_, ab_image_publisher_);

#ifdef ENA_POINTCLOUD_PUBLISH
  publishPointCloud(xyz_frame);
#endif /* ENA_POINTCLOUD_PUBLISH */
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
void ADI3DToFSafetyBubbleDetector::publishImageAndCameraInfo(
  unsigned char * compressed_depth_frame, int compressed_depth_frame_size,
  unsigned char * compressed_ab_frame, int compressed_ab_frame_size)
{
  fillAndPublishCameraInfo(optical_camera_link_, depth_info_publisher_);

  publishRVLCompressedImageAsRosMsg(
    compressed_depth_frame, compressed_depth_frame_size, "mono16", optical_camera_link_,
    compressed_depth_image_publisher_);

  publishRVLCompressedImageAsRosMsg(
    compressed_ab_frame, compressed_ab_frame_size, "mono16", optical_camera_link_,
    compressed_ab_image_publisher_);
}

/**
 * @brief Rebuilds the visualization cache with current zone configuration.
 * 
 * This function pre-computes static visualization elements that don't change
 * between frames, significantly improving performance for embedded processors.
 * Called when zone configuration changes via dynamic reconfigure.
 * 
 * Cached elements:
 * - Zone background image with fills and status boxes
 * - Non-overlapping zone masks for fast object overlay
 * - Cumulative mask for outside-zone detection
 */
void ADI3DToFSafetyBubbleDetector::rebuildVisualizationCache()
{
  using namespace adi_3dtof_safety_bubble_detector;

  std::lock_guard<std::mutex> lock(visualization_cache_mutex_);

  // Get current zone configuration
  const auto& zones = multi_zone_config_.getZones();
  int num_zones = static_cast<int>(zones.size());

  // Lambda to get zone BGR color (bright colors for detected objects)
  auto getZoneBGRColor = [](int zone_idx) -> cv::Scalar {
    if (zone_idx == 0) {
      return cv::Scalar(0, 0, 255);  // Red (BGR format)
    } else if (zone_idx == 1) {
      return cv::Scalar(0, 255, 255);  // Yellow
    } else {
      return cv::Scalar(0, 255, 0);  // Green
    }
  };

  // Lambda to get zone fill color (dim colors for zone background)
  auto getZoneFillColor = [](int zone_idx) -> cv::Scalar {
    if (zone_idx == 0) {
      return cv::Scalar(0, 0, 80);  // Dark red
    } else if (zone_idx == 1) {
      return cv::Scalar(0, 80, 80);  // Dark yellow
    } else {
      return cv::Scalar(0, 80, 0);  // Dark green
    }
  };

  cv::Point center(image_width_ / 2, image_height_ / 2);

  // Create cumulative masks for each zone
  std::vector<cv::Mat> cumulative_masks(num_zones);
  for (int z = 0; z < num_zones; z++) {
    cumulative_masks[z] = cv::Mat::zeros(cv::Size(image_width_, image_height_), CV_8UC1);

    const ZoneConfig& zone = zones[z];

    if (zone.shape == ZoneShape::CIRCLE) {
      int radius_pixels = static_cast<int>(zone.radius_mtr * pixels_per_meter_);
      cv::circle(cumulative_masks[z], center, radius_pixels, 255, -1);
    } else {
      int half_width_pixels = static_cast<int>(zone.x_dim_mtr * pixels_per_meter_);
      int half_height_pixels = static_cast<int>(zone.z_dim_mtr * pixels_per_meter_);
      cv::Point top_left(center.x - half_width_pixels, center.y - half_height_pixels);
      cv::Point bottom_right(center.x + half_width_pixels, center.y + half_height_pixels);
      cv::rectangle(cumulative_masks[z], top_left, bottom_right, 255, -1);
    }
  }

  // Each zone is independent from origin - store full zone masks
  cached_zone_masks_.resize(num_zones);
  for (int z = 0; z < num_zones; z++) {
    cached_zone_masks_[z] = cumulative_masks[z].clone();
  }

  // Cache union of all enabled zones for outside-zone detection
  cached_cumulative_mask_ = cv::Mat::zeros(cv::Size(image_width_, image_height_), CV_8UC1);
  for (int z = 0; z < num_zones; z++) {
    if (zones[z].enabled) {
      cv::bitwise_or(cached_cumulative_mask_, cumulative_masks[z], cached_cumulative_mask_);
    }
  }

  // Create background image with zone fills
  zone_background_image_ = cv::Mat(image_height_, image_width_, CV_8UC3, cv::Scalar(0, 0, 0));

  // Draw zone fills (from outer to inner so inner zones appear on top)
  if (enable_safety_bubble_zone_visualization_) {
    for (int z = num_zones - 1; z >= 0; z--) {
      if (!zones[z].enabled) continue;

      cv::Scalar fill_color = getZoneFillColor(z);
      
      // Use copyTo with mask for fast fill instead of per-pixel loop
      cv::Mat fill_layer(image_height_, image_width_, CV_8UC3, fill_color);
      fill_layer.copyTo(zone_background_image_, cached_zone_masks_[z]);
    }
  }

  // Draw status indicator boxes in top-left corner on background
  int box_size = 20;
  int box_spacing = 5;
  int start_x = 8;
  int start_y = 10;

  for (int z = 0; z < num_zones; z++) {
    cv::Rect box(start_x + z * (box_size + box_spacing), start_y, box_size, box_size);
    cv::Scalar zone_color = getZoneBGRColor(z);
    cv::Scalar fill_color = getZoneFillColor(z);

    if (!zones[z].enabled) {
      // Disabled zone: gray box with X
      cv::rectangle(zone_background_image_, box, cv::Scalar(80, 80, 80), -1);
      cv::line(zone_background_image_, box.tl(), box.br(), cv::Scalar(128, 128, 128), 2);
    } else {
      // Enabled but not detected: filled with dim color, bright outline
      cv::rectangle(zone_background_image_, box, fill_color, -1);
      cv::rectangle(zone_background_image_, box, zone_color, 2);
    }
  }

  // Add zone labels (1, 2, 3, ...) below status boxes on background
  for (int z = 0; z < num_zones; z++) {
    cv::Point text_pos(start_x + z * (box_size + box_spacing) + 5, start_y + box_size + 15);
    cv::putText(
      zone_background_image_, std::to_string(z + 1), text_pos, cv::FONT_HERSHEY_SIMPLEX, 0.4,
      cv::Scalar(255, 255, 255), 1);
  }

  visualization_cache_valid_ = true;
}

/**
 * @brief Generates the output image for visualization with multi-zone support.
 *
 * This function creates a color-coded visualization where:
 * - Zone 1 (innermost/danger): Red - objects detected within this zone
 * - Zone 2 (warning): Yellow - objects detected within this zone  
 * - Zone 3 (outer/safe): Green - objects detected within this zone
 * - Objects outside all zones are shown in gray
 *
 * Zone configuration is loaded via dynamic reconfigure with 3 zones configured.
 * Each zone can have independent shape (circle/rectangle) and dimensions.
 *
 * @param vcam_depth_image_floor_pixels_removed_8bpp - Vcam image with floor pixels removed
 * @param vcam_depth_frame_with_floor - Original vcam image
 * @param detection_result - Multi-zone detection results passed from output queue
 * @return Visualization output(cv::Mat() type)
 */
cv::Mat ADI3DToFSafetyBubbleDetector::generateVisualizationImage(
  unsigned char * vcam_depth_image_floor_pixels_removed_8bpp,
  unsigned short * vcam_depth_frame_with_floor,
  const adi_3dtof_safety_bubble_detector::MultiZoneDetectionResult& detection_result)
{
  using namespace adi_3dtof_safety_bubble_detector;

  cv::Mat m_vcam_final_image = cv::Mat(
    cv::Size(image_width_, image_height_), CV_8UC1, vcam_depth_image_floor_pixels_removed_8bpp);

  // Define zone color getters (used in both visualization modes)
  auto getZoneBGRColor = [](int zone_idx) -> cv::Scalar {
    if (zone_idx == 0) {
      return cv::Scalar(0, 0, 255);  // Red (danger)
    } else if (zone_idx == 1) {
      return cv::Scalar(0, 255, 255);  // Yellow (warning)
    } else {
      return cv::Scalar(0, 255, 0);  // Green (safe)
    }
  };

  auto getZoneFillColor = [](int zone_idx) -> cv::Scalar {
    if (zone_idx == 0) {
      return cv::Scalar(0, 0, 80);  // Dark red
    } else if (zone_idx == 1) {
      return cv::Scalar(0, 80, 80);  // Dark yellow
    } else {
      return cv::Scalar(0, 80, 0);  // Dark green
    }
  };

  // If zone visualization is disabled, return simple depth visualization with status indicators
  if (!enable_safety_bubble_zone_visualization_) {
    cv::Mat simple_output;
    cv::cvtColor(m_vcam_final_image, simple_output, cv::COLOR_GRAY2BGR);
    
    // Still apply floor paint if enabled (independent parameter)
    if (enable_floor_paint_) {
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
      cv::Mat floor_image = cv::Mat(image_height_, image_width_, CV_8UC1, vcam_depth_frame_8bpp_);
      
      cv::Mat floor_mask = (floor_image > 0);
      cv::Mat floor_color(image_height_, image_width_, CV_8UC3, cv::Scalar(100, 100, 100));
      floor_color.copyTo(simple_output, floor_mask);
    }
    
    // Always draw status indicator boxes (top-left corner) even when zone visualization is off
    const std::vector<ZoneConfig> & zones = multi_zone_config_.getZones();
    const int num_zones = static_cast<int>(zones.size());
    int box_size = 20;
    int box_spacing = 5;
    int start_x = 8;
    int start_y = 10;

    for (int z = 0; z < num_zones; z++) {
      cv::Rect box(start_x + z * (box_size + box_spacing), start_y, box_size, box_size);
      cv::Scalar zone_color = getZoneBGRColor(z);
      cv::Scalar fill_color = getZoneFillColor(z);

      if (!zones[z].enabled) {
        // Disabled zone: gray box with X
        cv::rectangle(simple_output, box, cv::Scalar(80, 80, 80), -1);
        cv::line(simple_output, box.tl(), box.br(), cv::Scalar(128, 128, 128), 2);
        cv::line(simple_output, cv::Point(box.x + box.width, box.y), 
                 cv::Point(box.x, box.y + box.height), cv::Scalar(128, 128, 128), 2);
      } else {
        // Check detection status from result
        bool detected = (z < static_cast<int>(detection_result.zone_detected.size())) 
                        && detection_result.zone_detected[z];
        
        if (detected) {
          // Detected: filled box with bright zone color
          cv::rectangle(simple_output, box, zone_color, -1);
        } else {
          // Not detected: filled with dim color, bright outline
          cv::rectangle(simple_output, box, fill_color, -1);
          cv::rectangle(simple_output, box, zone_color, 2);
        }
      }
    }

    // Add zone labels (1, 2, 3, ...) below status boxes
    for (int z = 0; z < num_zones; z++) {
      cv::Point text_pos(start_x + z * (box_size + box_spacing) + 5, start_y + box_size + 15);
      cv::putText(
        simple_output, std::to_string(z + 1), text_pos, cv::FONT_HERSHEY_SIMPLEX, 0.4,
        cv::Scalar(255, 255, 255), 1);
    }
    
    return simple_output;
  }

  // Rebuild cache if invalid
  if (!visualization_cache_valid_) {
    rebuildVisualizationCache();
  }

  // Get zones from multi-zone configuration
  const std::vector<ZoneConfig> & zones = multi_zone_config_.getZones();
  const int num_zones = static_cast<int>(zones.size());

  if (num_zones == 0) {
    RCLCPP_WARN_ONCE(this->get_logger(), "No zones configured, returning empty visualization");
    return cv::Mat::zeros(cv::Size(image_width_, image_height_), CV_8UC3);
  }

  // PERFORMANCE OPTIMIZED: Start with depth image base
  cv::Mat out_visualization_image;
  cv::cvtColor(m_vcam_final_image, out_visualization_image, cv::COLOR_GRAY2BGR);

  // LAYER 1: Paint floor pixels FIRST (if enabled) - this goes underneath everything
  if (enable_floor_paint_) {
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
    cv::Mat floor_image = cv::Mat(image_height_, image_width_, CV_8UC1, vcam_depth_frame_8bpp_);

    // Paint floor pixels with gray color wherever floor is detected (single copyTo)
    cv::Mat floor_mask = (floor_image > 0);
    cv::Mat floor_color(image_height_, image_width_, CV_8UC3, cv::Scalar(100, 100, 100));
    floor_color.copyTo(out_visualization_image, floor_mask);
  }

  // LAYER 2: Blend cached zone fills with 50% transparency, keep status boxes opaque
  {
    std::lock_guard<std::mutex> lock(visualization_cache_mutex_);
    
    // Combine all zone masks to identify zone fill regions
    cv::Mat zone_fills_mask = cv::Mat::zeros(image_height_, image_width_, CV_8UC1);
    for (const auto& mask : cached_zone_masks_) {
      cv::bitwise_or(zone_fills_mask, mask, zone_fills_mask);
    }
    
    // Create UI mask for status boxes and labels (top-left corner, ~100 pixels width x 50 height)
    cv::Mat ui_mask = cv::Mat::zeros(image_height_, image_width_, CV_8UC1);
    cv::Rect ui_region(0, 0, 100, 50);
    ui_mask(ui_region).setTo(255);
    
    // Separate zone fills mask from UI region
    cv::Mat zone_only_mask;
    cv::subtract(zone_fills_mask, ui_mask, zone_only_mask);
    
    // PERFORMANCE OPTIMIZATION: Find bounding box of zones and blend only that region
    cv::Rect zone_bbox = cv::boundingRect(zone_fills_mask);
    if (zone_bbox.area() > 0) {
      // Extract ROIs for only the zone bounding box region (reduces processing by 50-75%)
      cv::Mat base_roi = out_visualization_image(zone_bbox);
      cv::Mat overlay_roi = zone_background_image_(zone_bbox);
      cv::Mat mask_roi = zone_only_mask(zone_bbox);
      
      // Blend with 50% transparency only within zone bounding box
      cv::Mat blended_roi;
      cv::addWeighted(base_roi, 0.5, overlay_roi, 0.5, 0, blended_roi);
      blended_roi.copyTo(base_roi, mask_roi);
    }
    
    // Copy UI elements (status boxes) with full opacity
    cv::Mat ui_content_mask;
    cv::cvtColor(zone_background_image_, ui_content_mask, cv::COLOR_BGR2GRAY);
    ui_content_mask = (ui_content_mask > 0);
    cv::bitwise_and(ui_content_mask, ui_mask, ui_content_mask);
    zone_background_image_.copyTo(out_visualization_image, ui_content_mask);
  }

  // Apply ROI if valid
  cv::Rect roi = cv::Rect(valid_roi_.x, valid_roi_.y, valid_roi_.width, valid_roi_.height);
  if (roi.width <= 0 || roi.height <= 0 || roi.x + roi.width > image_width_ || roi.y + roi.height > image_height_) {
    roi = cv::Rect(0, 0, image_width_, image_height_);
  }

  cv::Mat vcam_roi = m_vcam_final_image(roi);

  // Track detection status for each zone
  std::vector<bool> zone_detected(num_zones, false);

  // Get detection status from passed detection_result parameter
  for (int z = 0; z < num_zones; z++) {
    if (z < static_cast<int>(detection_result.zone_detected.size())) {
      zone_detected[z] = detection_result.zone_detected[z];
    }
  }

  // LAYER 3: Overlay detected objects with bright colors (outermost first, innermost last so
  // the innermost zone color wins when multiple zones are triggered by the same object)
  for (int z = num_zones - 1; z >= 0; z--) {
    if (!zones[z].enabled || !zone_detected[z]) continue;

    cv::Scalar zone_color = getZoneBGRColor(z);
    cv::Mat zone_color_layer(image_height_, image_width_, CV_8UC3, zone_color);

    // Fast masked copy: only copy where mask is non-zero AND detected in vcam
    cv::Mat combined_mask;
    
    {
      std::lock_guard<std::mutex> lock(visualization_cache_mutex_);
      cv::Mat zone_mask_roi_local = cached_zone_masks_[z](roi);
      combined_mask = cv::Mat::zeros(roi.size(), CV_8UC1);
      cv::bitwise_and(vcam_roi, zone_mask_roi_local, combined_mask);
    }

    // Create full-size mask with ROI
    cv::Mat full_combined_mask = cv::Mat::zeros(cv::Size(image_width_, image_height_), CV_8UC1);
    combined_mask.copyTo(full_combined_mask(roi));

    // Fast vectorized copy using mask
    zone_color_layer.copyTo(out_visualization_image, full_combined_mask);
  }

  // LAYER 4: Handle pixels outside all enabled zones using cached masks
  {
    std::lock_guard<std::mutex> lock(visualization_cache_mutex_);
    
    // Build mask of enabled zones only
    cv::Mat enabled_zones_mask = cv::Mat::zeros(cv::Size(image_width_, image_height_), CV_8UC1);
    for (int z = 0; z < num_zones; z++) {
      if (zones[z].enabled) {
        cv::bitwise_or(enabled_zones_mask, cached_zone_masks_[z], enabled_zones_mask);
      }
    }
    
    cv::Mat outside_enabled_zones;
    cv::bitwise_not(enabled_zones_mask, outside_enabled_zones);
    
    cv::Mat outside_roi = outside_enabled_zones(roi);
    cv::Mat detected_outside;
    cv::bitwise_and(vcam_roi, outside_roi, detected_outside);

    // Create full-size mask
    cv::Mat full_outside_mask = cv::Mat::zeros(cv::Size(image_width_, image_height_), CV_8UC1);
    detected_outside.copyTo(full_outside_mask(roi));

    // Fast vectorized fill
    cv::Mat gray_layer(image_height_, image_width_, CV_8UC3, cv::Scalar(180, 180, 180));
    gray_layer.copyTo(out_visualization_image, full_outside_mask);
  }

  // LAYER 5: Draw status indicator boxes at top-left corner (always on top)
  int box_size = 20;
  int box_spacing = 5;
  int start_x = 8;
  int start_y = 10;

  for (int z = 0; z < num_zones; z++) {
    cv::Rect box(start_x + z * (box_size + box_spacing), start_y, box_size, box_size);
    cv::Scalar zone_color = getZoneBGRColor(z);
    cv::Scalar fill_color = getZoneFillColor(z);

    if (!zones[z].enabled) {
      // Disabled zone: gray box with X
      cv::rectangle(out_visualization_image, box, cv::Scalar(80, 80, 80), -1);
      cv::line(out_visualization_image, box.tl(), box.br(), cv::Scalar(128, 128, 128), 2);
      cv::line(out_visualization_image, cv::Point(box.x + box.width, box.y), 
               cv::Point(box.x, box.y + box.height), cv::Scalar(128, 128, 128), 2);
    } else if (zone_detected[z]) {
      // Detected: filled box with bright zone color
      cv::rectangle(out_visualization_image, box, zone_color, -1);
    } else {
      // Not detected: filled with dim color, bright outline
      cv::rectangle(out_visualization_image, box, fill_color, -1);
      cv::rectangle(out_visualization_image, box, zone_color, 2);
    }
  }

  // Add zone labels (1, 2, 3, ...) below status boxes
  for (int z = 0; z < num_zones; z++) {
    cv::Point text_pos(start_x + z * (box_size + box_spacing) + 5, start_y + box_size + 15);
    cv::putText(
      out_visualization_image, std::to_string(z + 1), text_pos, cv::FONT_HERSHEY_SIMPLEX, 0.4,
      cv::Scalar(255, 255, 255), 1);
  }

  return out_visualization_image;
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
void ADI3DToFSafetyBubbleDetector::publishRVLCompressedImageAsRosMsg(
  unsigned char * compressed_img, int compressed_img_size, const std::string & encoding_type,
  std::string frame_id,
  const rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr publisher)
{
  cv_bridge::CvImagePtr cv_ptr(new cv_bridge::CvImage);
  sensor_msgs::msg::CompressedImage::SharedPtr compressed_payload_ptr(
    new sensor_msgs::msg::CompressedImage());

  compressed_payload_ptr->format = encoding_type + ";compressedDepth rvl";
  compressed_payload_ptr->header.stamp = curr_frame_timestamp_;
  compressed_payload_ptr->header.frame_id = std::move(frame_id);
  compressed_payload_ptr->data.resize(
    compressed_img_size + 8 + sizeof(compressed_depth_image_transport::ConfigHeader));

  // Image compression configuration
  compressed_depth_image_transport::ConfigHeader compression_configuration{};
  compression_configuration.format = compressed_depth_image_transport::INV_DEPTH;

  // Header of compressed image of image transport
  float depth_quantization = 0;
  float maximum_depth = 1;

  // Inverse depth quantization parameters
  float depth_quantization_a = depth_quantization * (depth_quantization + 1.0f);
  float depth_quantization_b = 1.0f - depth_quantization_a / maximum_depth;

  // Add coding parameters to header
  compression_configuration.depthParam[0] = depth_quantization_a;
  compression_configuration.depthParam[1] = depth_quantization_b;

  memcpy(
    &compressed_payload_ptr->data[0], &compression_configuration,
    sizeof(compressed_depth_image_transport::ConfigHeader));
  memcpy(
    &compressed_payload_ptr->data[0] + sizeof(compressed_depth_image_transport::ConfigHeader),
    &image_width_, sizeof(int));
  memcpy(
    &compressed_payload_ptr->data[4] + sizeof(compressed_depth_image_transport::ConfigHeader),
    &image_height_, sizeof(int));

  memcpy(
    &compressed_payload_ptr->data[8] + sizeof(compressed_depth_image_transport::ConfigHeader),
    compressed_img, compressed_img_size);

  publisher->publish(*compressed_payload_ptr);
}

/**
 * @brief Publishes zone status array based on detection results
 *
 * @param detection_result Multi-zone detection result from MultiZoneDetector
 */
void ADI3DToFSafetyBubbleDetector::publishZoneStatus(
  const adi_3dtof_safety_bubble_detector::MultiZoneDetectionResult& detection_result)
{
  if (!zone_status_publisher_) {
    return;
  }

  adi_3dtof_safety_bubble_detector::msg::ZoneStatusArray zone_status_msg;
  zone_status_msg.header.stamp = detection_result.timestamp;
  zone_status_msg.header.frame_id = optical_camera_link_;
  
  const auto& zones = multi_zone_config_.getZones();
  zone_status_msg.num_zones = static_cast<int>(zones.size());

  for (size_t i = 0; i < zones.size(); ++i) {
    adi_3dtof_safety_bubble_detector::msg::ZoneStatus zone_status;
    zone_status.id = zones[i].id;
    zone_status.enabled = zones[i].enabled;
    zone_status.detected = (i < detection_result.zone_detected.size()) ? 
                            detection_result.zone_detected[i] : false;
    
    // Set shape
    zone_status.shape = (zones[i].shape == adi_3dtof_safety_bubble_detector::ZoneShape::CIRCLE) ? 
                        "circle" : "rectangle";
    
    // Set dimensions based on shape
    if (zones[i].shape == adi_3dtof_safety_bubble_detector::ZoneShape::CIRCLE) {
      zone_status.radius_mtr = zones[i].radius_mtr;
      zone_status.x_dim_mtr = 0.0f;
      zone_status.z_dim_mtr = 0.0f;
    } else {
      zone_status.radius_mtr = 0.0f;
      zone_status.x_dim_mtr = zones[i].x_dim_mtr * 2.0f;  // x_dim_mtr is half-dimension
      zone_status.z_dim_mtr = zones[i].z_dim_mtr * 2.0f;  // z_dim_mtr is half-dimension
    }
    
    // Set color (4 floats for RGBA)
    zone_status.color[0] = zones[i].color_rgba[0];
    zone_status.color[1] = zones[i].color_rgba[1];
    zone_status.color[2] = zones[i].color_rgba[2];
    zone_status.color[3] = zones[i].color_rgba[3];

    zone_status_msg.zones.push_back(zone_status);
  }

  zone_status_publisher_->publish(zone_status_msg);
}
