/******************************************************************************
Copyright (c), 2023 - Analog Devices Inc. All Rights Reserved.
This software is PROPRIETARY & CONFIDENTIAL to Analog Devices, Inc.
and its licensors.
******************************************************************************/

#include "multi_zone_detector.hpp"

#include <cmath>
#include <iomanip>
#include <sstream>

#include <opencv2/imgproc.hpp>

namespace adi_3dtof_safety_bubble_detector
{

MultiZoneDetector::MultiZoneDetector(
  int image_width, int image_height,
  float virtual_camera_height_mtr,
  float focal_length_x, float focal_length_y)
: image_width_(image_width),
  image_height_(image_height),
  virtual_camera_height_mtr_(virtual_camera_height_mtr),
  focal_length_x_(focal_length_x),
  focal_length_y_(focal_length_y)
{
  // Calculate pixels per meter based on virtual camera height and focal length
  float focal_length = (focal_length_x + focal_length_y) / 2.0f;
  float fov = 2.0f * std::atan2(image_width / 2.0f, focal_length);
  float fov_in_meters = 2.0f * virtual_camera_height_mtr * std::tan(fov / 2.0f);
  pixels_per_meter_ = static_cast<float>(image_width) / fov_in_meters;

  // Initialize with default 3-zone configuration
  config_.setNumZones(3);
}

MultiZoneDetector::~MultiZoneDetector()
{
  // Cleanup handled by smart pointers and RAII
}

bool MultiZoneDetector::initialize(const MultiZoneConfig & config)
{
  std::lock_guard<std::mutex> lock(config_mutex_);

  config_ = config;
  regenerateZoneMasks();
  initialized_ = true;

  return true;
}

bool MultiZoneDetector::loadConfigFromYaml(const std::string & filepath)
{
  std::lock_guard<std::mutex> lock(config_mutex_);

  if (!config_.loadFromYaml(filepath)) {
    return false;
  }

  regenerateZoneMasks();
  initialized_ = true;

  return true;
}

bool MultiZoneDetector::saveConfigToYaml(const std::string & filepath) const
{
  std::lock_guard<std::mutex> lock(config_mutex_);
  return config_.saveToYaml(filepath);
}

void MultiZoneDetector::setNumZones(int num_zones)
{
  std::lock_guard<std::mutex> lock(config_mutex_);
  config_.setNumZones(num_zones);
  regenerateZoneMasks();
}

bool MultiZoneDetector::updateZone(int zone_id, const ZoneConfig & zone_config)
{
  std::lock_guard<std::mutex> lock(config_mutex_);

  if (!config_.updateZone(zone_id, zone_config)) {
    return false;
  }

  regenerateZoneMasks();
  return true;
}

void MultiZoneDetector::regenerateZoneMasks()
{
  // Note: Caller should hold config_mutex_

  zone_masks_.clear();
  exclusive_zone_masks_.clear();

  const auto & zones = config_.getZones();

  // Create cumulative masks for each zone
  for (const auto & zone : zones) {
    zone_masks_.push_back(createZoneMask(zone));
  }

  // Create exclusive masks (zone area minus inner zones)
  for (size_t i = 0; i < zones.size(); ++i) {
    exclusive_zone_masks_.push_back(createExclusiveZoneMask(i));
  }

  // Update combined visualization masks
  updateCombinedMasks();
}

cv::Mat MultiZoneDetector::createZoneMask(const ZoneConfig & zone) const
{
  cv::Mat mask = cv::Mat::zeros(cv::Size(image_width_, image_height_), CV_8UC1);

  int center_x = image_width_ / 2;
  int center_y = image_height_ / 2;

  // Always create the geometric mask, even if zone is disabled
  // This ensures that exclusive masks for outer zones correctly exclude inner zones
  // The enabled flag is checked during detection, not during mask generation
  
  if (zone.shape == ZoneShape::CIRCLE) {
    int radius_pixels = radiusMtrToPixels(zone.radius_mtr);
    cv::circle(mask, cv::Point(center_x, center_y), radius_pixels, 255, -1);
  } else {
    // Rectangle: x_dim_mtr and z_dim_mtr are already half-dimensions (distance from center)
    int half_width, half_height;
    dimsMtrToPixels(zone.x_dim_mtr, zone.z_dim_mtr, half_width, half_height);

    cv::Point top_left(center_x - half_width, center_y - half_height);
    cv::Point bottom_right(center_x + half_width, center_y + half_height);
    cv::rectangle(mask, top_left, bottom_right, 255, -1);
  }

  return mask;
}

cv::Mat MultiZoneDetector::createExclusiveZoneMask(size_t zone_idx) const
{
  if (zone_idx >= zone_masks_.size()) {
    return cv::Mat();
  }

  cv::Mat exclusive_mask = zone_masks_[zone_idx].clone();

  // Subtract all inner zone masks
  for (size_t i = 0; i < zone_idx; ++i) {
    cv::bitwise_and(exclusive_mask, ~zone_masks_[i], exclusive_mask);
  }

  return exclusive_mask;
}

void MultiZoneDetector::updateCombinedMasks()
{
  combined_zone_mask_ = cv::Mat::zeros(cv::Size(image_width_, image_height_), CV_8UC1);
  colored_zone_image_ = cv::Mat::zeros(cv::Size(image_width_, image_height_), CV_8UC3);

  const auto & zones = config_.getZones();

  // Draw from outermost to innermost so inner zones overlay outer ones
  for (int i = static_cast<int>(zones.size()) - 1; i >= 0; --i) {
    const auto & zone = zones[i];
    if (!zone.enabled) continue;

    // Update binary mask
    cv::bitwise_or(combined_zone_mask_, zone_masks_[i], combined_zone_mask_);

    // Create colored overlay
    cv::Scalar color = zoneColorToScalar(zone.color_rgba);
    cv::Mat zone_colored;
    cv::cvtColor(zone_masks_[i], zone_colored, cv::COLOR_GRAY2BGR);

    // Apply color with alpha blending
    for (int y = 0; y < image_height_; ++y) {
      for (int x = 0; x < image_width_; ++x) {
        if (zone_masks_[i].at<uchar>(y, x) > 0) {
          cv::Vec3b & pixel = colored_zone_image_.at<cv::Vec3b>(y, x);
          float alpha = zone.color_rgba[3];
          pixel[0] = static_cast<uchar>(color[0] * alpha + pixel[0] * (1.0f - alpha));
          pixel[1] = static_cast<uchar>(color[1] * alpha + pixel[1] * (1.0f - alpha));
          pixel[2] = static_cast<uchar>(color[2] * alpha + pixel[2] * (1.0f - alpha));
        }
      }
    }
  }
}

cv::Mat MultiZoneDetector::getZoneMask(int zone_id) const
{
  std::lock_guard<std::mutex> lock(config_mutex_);

  if (zone_id < 1 || zone_id > static_cast<int>(zone_masks_.size())) {
    return cv::Mat();
  }

  return zone_masks_[zone_id - 1].clone();
}

cv::Scalar MultiZoneDetector::zoneColorToScalar(const std::array<float, 4> & color) const
{
  // OpenCV uses BGR order
  return cv::Scalar(
    color[2] * 255.0f,  // B
    color[1] * 255.0f,  // G
    color[0] * 255.0f   // R
  );
}

int MultiZoneDetector::radiusMtrToPixels(float radius_mtr) const
{
  return static_cast<int>(radius_mtr * pixels_per_meter_);
}

void MultiZoneDetector::dimsMtrToPixels(
  float x_dim_mtr, float z_dim_mtr,
  int & width_pixels, int & height_pixels) const
{
  // x_dim_mtr and z_dim_mtr are half-dimensions (distance from center)
  // Convert to pixels directly as they already represent half-dimensions
  width_pixels = static_cast<int>(x_dim_mtr * pixels_per_meter_);
  height_pixels = static_cast<int>(z_dim_mtr * pixels_per_meter_);
}

MultiZoneDetectionResult MultiZoneDetector::detectZones(
  const unsigned char * vcam_depth_8bpp,
  const cv::Rect & roi)
{
  std::lock_guard<std::mutex> lock(config_mutex_);

  MultiZoneDetectionResult result;
  result.timestamp = rclcpp::Clock().now();

  const auto & zones = config_.getZones();
  result.zone_detected.resize(zones.size(), false);

  if (!initialized_ || vcam_depth_8bpp == nullptr) {
    return result;
  }

  // Create cv::Mat wrapper around input data
  cv::Mat depth_image(cv::Size(image_width_, image_height_), CV_8UC1,
                      const_cast<unsigned char *>(vcam_depth_8bpp));

  // Validate ROI
  cv::Rect valid_roi = roi;
  if (valid_roi.width == 0 || valid_roi.height == 0) {
    valid_roi = cv::Rect(0, 0, image_width_, image_height_);
  }

  // Clamp ROI to image bounds
  valid_roi &= cv::Rect(0, 0, image_width_, image_height_);

  // Extract ROI from depth image
  cv::Mat depth_roi = depth_image(valid_roi);

  // Process each zone (from innermost to outermost)
  for (size_t i = 0; i < zones.size(); ++i) {
    if (!zones[i].enabled) {
      continue;
    }

    // Get exclusive zone mask for this zone
    cv::Mat zone_mask_roi = exclusive_zone_masks_[i](valid_roi);

    // Apply mask to depth image
    cv::Mat masked_depth;
    cv::bitwise_and(depth_roi, zone_mask_roi, masked_depth);

    // Binarize
    cv::Mat binary_mask;
    cv::threshold(masked_depth, binary_mask, 0, 255, cv::THRESH_BINARY);

    // Connected component analysis
    cv::Mat labels, stats, centroids;
    int num_labels = cv::connectedComponentsWithStats(
      binary_mask, labels, stats, centroids, 4, CV_16U);

    // Check for objects above sensitivity threshold
    for (int label = 1; label < num_labels; ++label) {
      int blob_area = stats.at<int>(label, cv::CC_STAT_AREA);
      if (blob_area > sensitivity_) {
        result.zone_detected[i] = true;
        result.any_detected = true;

        // Track innermost detected zone
        if (result.innermost_detected_zone == 0 ||
            static_cast<int>(i + 1) < result.innermost_detected_zone) {
          result.innermost_detected_zone = static_cast<int>(i + 1);
        }
        break;
      }
    }
  }

  return result;
}

visualization_msgs::msg::MarkerArray MultiZoneDetector::generateVisualizationMarkers(
  const std::string & frame_id,
  const MultiZoneDetectionResult & detection_result) const
{
  std::lock_guard<std::mutex> lock(config_mutex_);

  visualization_msgs::msg::MarkerArray marker_array;

  const auto & zones = config_.getZones();

  for (size_t i = 0; i < zones.size(); ++i) {
    const auto & zone = zones[i];
    if (!zone.enabled) continue;

    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = frame_id;
    marker.header.stamp = detection_result.timestamp;
    marker.ns = "safety_zones";
    marker.id = zone.id;
    marker.action = visualization_msgs::msg::Marker::ADD;

    // Position at origin (zones are relative to sensor)
    marker.pose.position.x = 0.0;
    marker.pose.position.y = 0.0;
    marker.pose.position.z = 0.0;
    marker.pose.orientation.w = 1.0;

    // Get color (with increased alpha if detected)
    bool detected = (i < detection_result.zone_detected.size()) &&
                    detection_result.zone_detected[i];
    auto color = getZoneColor(zone.id, config_.getNumZones(), detected);

    marker.color.r = color[0];
    marker.color.g = color[1];
    marker.color.b = color[2];
    marker.color.a = color[3];

    if (zone.shape == ZoneShape::CIRCLE) {
      marker.type = visualization_msgs::msg::Marker::CYLINDER;
      float radius_m = zone.getRadiusMeters();
      marker.scale.x = radius_m * 2.0;  // Diameter
      marker.scale.y = radius_m * 2.0;
      marker.scale.z = 0.01;  // Thin cylinder
    } else {
      marker.type = visualization_msgs::msg::Marker::CUBE;
      marker.scale.x = zone.getXDimMeters();
      marker.scale.y = zone.getZDimMeters();
      marker.scale.z = 0.01;  // Thin box
    }

    marker.lifetime = rclcpp::Duration(0, 500000000);  // 0.5 seconds

    marker_array.markers.push_back(marker);

    // Add text label marker
    visualization_msgs::msg::Marker text_marker;
    text_marker.header = marker.header;
    text_marker.ns = "zone_labels";
    text_marker.id = zone.id + 100;  // Offset ID to avoid collision
    text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    text_marker.action = visualization_msgs::msg::Marker::ADD;

    // Position above the zone
    text_marker.pose.position.x = 0.0;
    text_marker.pose.position.y = 0.0;
    text_marker.pose.position.z = 0.1 + (i * 0.05);
    text_marker.pose.orientation.w = 1.0;

    text_marker.scale.z = 0.1;  // Text height

    text_marker.color.r = 1.0;
    text_marker.color.g = 1.0;
    text_marker.color.b = 1.0;
    text_marker.color.a = 1.0;

    std::stringstream ss;
    ss << "Zone " << zone.id;
    if (detected) {
      ss << " [DETECTED]";
    }
    text_marker.text = ss.str();

    text_marker.lifetime = rclcpp::Duration(0, 500000000);

    marker_array.markers.push_back(text_marker);
  }

  return marker_array;
}

std::string MultiZoneDetector::generateJsonStatus(
  const MultiZoneDetectionResult & detection_result) const
{
  std::lock_guard<std::mutex> lock(config_mutex_);

  const auto & zones = config_.getZones();

  std::stringstream ss;
  ss << "{\n";

  // Timestamp in ISO 8601 format
  auto time_ns = detection_result.timestamp.nanoseconds();
  auto time_s = time_ns / 1000000000;
  ss << "  \"timestamp\": \"" << time_s << "\",\n";

  ss << "  \"num_zones\": " << config_.getNumZones() << ",\n";
  ss << "  \"any_detected\": " << (detection_result.any_detected ? "true" : "false") << ",\n";
  ss << "  \"innermost_detected_zone\": " << detection_result.innermost_detected_zone << ",\n";

  ss << "  \"zones\": [\n";

  for (size_t i = 0; i < zones.size(); ++i) {
    const auto & zone = zones[i];
    bool detected = (i < detection_result.zone_detected.size()) &&
                    detection_result.zone_detected[i];

    ss << "    {\n";
    ss << "      \"id\": " << zone.id << ",\n";
    ss << "      \"enabled\": " << (zone.enabled ? "true" : "false") << ",\n";
    ss << "      \"shape\": \"" << zoneShapeToString(zone.shape) << "\",\n";

    if (zone.shape == ZoneShape::CIRCLE) {
      ss << "      \"radius_mtr\": " << std::fixed << std::setprecision(3) << zone.radius_mtr << ",\n";
    } else {
      ss << "      \"x_dim_mtr\": " << std::fixed << std::setprecision(3) << zone.x_dim_mtr << ",\n";
      ss << "      \"z_dim_mtr\": " << std::fixed << std::setprecision(3) << zone.z_dim_mtr << ",\n";
    }

    ss << "      \"detected\": " << (detected ? "true" : "false") << "\n";
    ss << "    }";

    if (i < zones.size() - 1) {
      ss << ",";
    }
    ss << "\n";
  }

  ss << "  ]\n";
  ss << "}\n";

  return ss.str();
}

void MultiZoneDetector::overlayZoneBoundaries(
  cv::Mat & output_image,
  const MultiZoneDetectionResult & detection_result,
  bool show_labels) const
{
  std::lock_guard<std::mutex> lock(config_mutex_);

  if (output_image.empty()) return;

  const auto & zones = config_.getZones();
  int center_x = output_image.cols / 2;
  int center_y = output_image.rows / 2;

  // Draw from outermost to innermost
  for (int i = static_cast<int>(zones.size()) - 1; i >= 0; --i) {
    const auto & zone = zones[i];
    if (!zone.enabled) continue;

    bool detected = (static_cast<size_t>(i) < detection_result.zone_detected.size()) &&
                    detection_result.zone_detected[i];

    // Get color with detection state
    auto color = getZoneColor(zone.id, config_.getNumZones(), detected);
    cv::Scalar cv_color = zoneColorToScalar(color);

    // Increase line thickness for detected zones
    int line_thickness = detected ? 3 : 2;

    if (zone.shape == ZoneShape::CIRCLE) {
      int radius_pixels = radiusMtrToPixels(zone.radius_mtr);
      cv::circle(output_image, cv::Point(center_x, center_y), radius_pixels,
                 cv_color, line_thickness);
    } else {
      // Rectangle: x_dim_mtr and z_dim_mtr are already half-dimensions (distance from center)
      int half_width, half_height;
      dimsMtrToPixels(zone.x_dim_mtr, zone.z_dim_mtr, half_width, half_height);

      cv::Point top_left(center_x - half_width, center_y - half_height);
      cv::Point bottom_right(center_x + half_width, center_y + half_height);
      cv::rectangle(output_image, top_left, bottom_right, cv_color, line_thickness);
    }

    // Draw labels
    if (show_labels) {
      std::stringstream label;
      label << "Z" << zone.id;
      if (detected) {
        label << "*";
      }

      int label_x, label_y;
      if (zone.shape == ZoneShape::CIRCLE) {
        int radius_pixels = radiusMtrToPixels(zone.radius_mtr);
        label_x = center_x + radius_pixels - 30;
        label_y = center_y - 5;
      } else {
        int half_width, half_height;
        dimsMtrToPixels(zone.x_dim_mtr, zone.z_dim_mtr, half_width, half_height);
        label_x = center_x + half_width / 2 - 20;
        label_y = center_y - half_height / 2 + 15;
      }

      // Ensure label is within image bounds
      label_x = std::max(5, std::min(label_x, output_image.cols - 40));
      label_y = std::max(15, std::min(label_y, output_image.rows - 5));

      cv::putText(output_image, label.str(), cv::Point(label_x, label_y),
                  cv::FONT_HERSHEY_SIMPLEX, 0.5, cv_color, 2);
    }
  }

  // Draw detection status indicator
  cv::Rect status_box(8, 10, 25, 25);
  cv::Scalar status_color;
  if (detection_result.any_detected) {
    if (detection_result.innermost_detected_zone == 1) {
      status_color = cv::Scalar(0, 0, 255);  // Red for Zone 1
    } else if (detection_result.innermost_detected_zone == 2) {
      status_color = cv::Scalar(0, 255, 255);  // Yellow for Zone 2
    } else {
      status_color = cv::Scalar(0, 255, 0);  // Green for outer zones
    }
  } else {
    status_color = cv::Scalar(0, 255, 0);  // Green when clear
  }
  cv::rectangle(output_image, status_box, status_color, -1);
}

}  // namespace adi_3dtof_safety_bubble_detector
