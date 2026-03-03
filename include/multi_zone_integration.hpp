/******************************************************************************
Copyright (c), 2023 - Analog Devices Inc. All Rights Reserved.
This software is PROPRIETARY & CONFIDENTIAL to Analog Devices, Inc.
and its licensors.
******************************************************************************/
#ifndef ADI_3DTOF_SAFETY_BUBBLE_DETECTOR_MULTI_ZONE_INTEGRATION_HPP
#define ADI_3DTOF_SAFETY_BUBBLE_DETECTOR_MULTI_ZONE_INTEGRATION_HPP

/**
 * @file multi_zone_integration.hpp
 * @brief Integration helpers for multi-zone detection in the safety bubble detector
 *
 * This header provides helper classes and functions to integrate multi-zone
 * detection capabilities into the existing ADI3DToFSafetyBubbleDetector node.
 *
 * Usage:
 *   1. Include this header in adi_3dtof_safety_bubble_detector_node.h
 *   2. Add MultiZoneIntegration as a member of ADI3DToFSafetyBubbleDetector
 *   3. Initialize in constructor with image dimensions and camera parameters
 *   4. Call processMultiZoneDetection() in runSafetyBubbleDetection()
 *   5. Call publishMultiZoneResults() in processOutput()
 */

#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "multi_zone_detector.hpp"
#include "zone_config.hpp"

namespace adi_3dtof_safety_bubble_detector
{

/**
 * @brief Helper class for integrating multi-zone detection into the main node
 *
 * This class manages:
 * - MultiZoneDetector instance
 * - ROS 2 publishers for zone status topics
 * - ROS 2 services for zone configuration
 * - Parameter handling for zone configuration
 */
class MultiZoneIntegration
{
public:
  /**
   * @brief Construct multi-zone integration helper
   * @param node Parent ROS node
   * @param image_width Image width in pixels
   * @param image_height Image height in pixels
   * @param virtual_camera_height Virtual camera height in meters
   * @param focal_length_x Focal length X
   * @param focal_length_y Focal length Y
   */
  MultiZoneIntegration(
    rclcpp::Node * node,
    int image_width, int image_height,
    float virtual_camera_height,
    float focal_length_x, float focal_length_y)
  : node_(node),
    image_width_(image_width),
    image_height_(image_height)
  {
    // Create multi-zone detector
    detector_ = std::make_unique<MultiZoneDetector>(
      image_width, image_height,
      virtual_camera_height,
      focal_length_x, focal_length_y);

    // Declare parameters
    declareParameters();

    // Initialize detector with parameters
    initializeFromParameters();

    // Create publishers
    createPublishers();

    // Set up parameter callback
    setupParameterCallback();

    RCLCPP_INFO(node_->get_logger(), "Multi-zone integration initialized with %d zones",
                detector_->getNumZones());
  }

  /**
   * @brief Get the multi-zone detector
   */
  MultiZoneDetector * getDetector() { return detector_.get(); }

  /**
   * @brief Process multi-zone detection on a frame
   * @param vcam_depth_8bpp 8-bit depth image from virtual camera
   * @param roi Valid region of interest
   * @return Detection result for all zones
   */
  MultiZoneDetectionResult processDetection(
    const unsigned char * vcam_depth_8bpp,
    const cv::Rect & roi)
  {
    last_result_ = detector_->detectZones(vcam_depth_8bpp, roi);
    return last_result_;
  }

  /**
   * @brief Publish multi-zone detection results
   * @param frame_id Frame ID for messages
   */
  void publishResults(const std::string & frame_id)
  {
    // Publish individual zone detection topics
    const auto & zones = detector_->getConfig().getZones();
    for (size_t i = 0; i < zones.size() && i < zone_detected_pubs_.size(); ++i) {
      std_msgs::msg::Bool msg;
      msg.data = (i < last_result_.zone_detected.size()) && last_result_.zone_detected[i];
      zone_detected_pubs_[i]->publish(msg);
    }

    // Publish RViz markers
    auto markers = detector_->generateVisualizationMarkers(frame_id, last_result_);
    marker_array_pub_->publish(markers);

    // Publish JSON status
    std_msgs::msg::String json_msg;
    json_msg.data = detector_->generateJsonStatus(last_result_);
    json_status_pub_->publish(json_msg);
  }

  /**
   * @brief Overlay zone boundaries on visualization image
   * @param output_image Image to draw on
   * @param show_labels Whether to show zone labels
   */
  void overlayZones(cv::Mat & output_image, bool show_labels = true)
  {
    detector_->overlayZoneBoundaries(output_image, last_result_, show_labels);
  }

  /**
   * @brief Get the innermost zone with detection
   * @return Zone ID (1-based) or 0 if no detection
   */
  int getInnermostDetectedZone() const
  {
    return last_result_.innermost_detected_zone;
  }

  /**
   * @brief Check if any zone has detection
   */
  bool anyZoneDetected() const
  {
    return last_result_.any_detected;
  }

  /**
   * @brief Get detection status for a specific zone
   * @param zone_id Zone ID (1-based)
   * @return true if object detected in zone
   */
  bool isZoneDetected(int zone_id) const
  {
    if (zone_id < 1 || zone_id > static_cast<int>(last_result_.zone_detected.size())) {
      return false;
    }
    return last_result_.zone_detected[zone_id - 1];
  }

  /**
   * @brief Update zone publishers after configuration change
   */
  void updatePublishers()
  {
    createZoneDetectedPublishers();
  }

private:
  void declareParameters()
  {
    // Number of zones
    rcl_interfaces::msg::ParameterDescriptor num_zones_desc;
    num_zones_desc.description = "Number of safety zones (1-10)";
    node_->declare_parameter("param_num_zones", 3, num_zones_desc);

    // Zone configuration file
    rcl_interfaces::msg::ParameterDescriptor zone_config_desc;
    zone_config_desc.description = "Path to zone configuration YAML file";
    node_->declare_parameter("param_zone_config_file", "", zone_config_desc);

    // Individual zone parameters (for zones 1-3 as commonly used)
    for (int i = 1; i <= 3; ++i) {
      std::string prefix = "param_zone_" + std::to_string(i) + "_";

      node_->declare_parameter(prefix + "enabled", true);
      node_->declare_parameter(prefix + "shape", "circle");
      node_->declare_parameter(prefix + "radius_cm", 50.0 * i);
      node_->declare_parameter(prefix + "x_dim_cm", 50.0 * i);
      node_->declare_parameter(prefix + "z_dim_cm", 50.0 * i);
    }
  }

  void initializeFromParameters()
  {
    // Check for config file first
    std::string config_file = node_->get_parameter("param_zone_config_file").as_string();
    if (!config_file.empty()) {
      if (detector_->loadConfigFromYaml(config_file)) {
        RCLCPP_INFO(node_->get_logger(), "Loaded zone config from: %s", config_file.c_str());
        return;
      }
      RCLCPP_WARN(node_->get_logger(), "Failed to load config file, using parameters");
    }

    // Initialize from parameters
    int num_zones = node_->get_parameter("param_num_zones").as_int();
    detector_->setNumZones(num_zones);

    // Configure individual zones from parameters
    for (int i = 1; i <= std::min(num_zones, 3); ++i) {
      std::string prefix = "param_zone_" + std::to_string(i) + "_";

      ZoneConfig config;
      config.id = i;
      config.enabled = node_->get_parameter(prefix + "enabled").as_bool();

      std::string shape = node_->get_parameter(prefix + "shape").as_string();
      config.shape = stringToZoneShape(shape);

      config.radius_cm = node_->get_parameter(prefix + "radius_cm").as_double();
      config.x_dim_cm = node_->get_parameter(prefix + "x_dim_cm").as_double();
      config.z_dim_cm = node_->get_parameter(prefix + "z_dim_cm").as_double();

      detector_->updateZone(i, config);
    }

    detector_->regenerateZoneMasks();
  }

  void createPublishers()
  {
    // Create marker array publisher
    marker_array_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(
      "safety_bubble/visualization", 10);

    // Create JSON status publisher
    json_status_pub_ = node_->create_publisher<std_msgs::msg::String>(
      "safety_bubble/status_json", 10);

    // Create individual zone publishers
    createZoneDetectedPublishers();
  }

  void createZoneDetectedPublishers()
  {
    zone_detected_pubs_.clear();
    int num_zones = detector_->getNumZones();

    for (int i = 1; i <= num_zones; ++i) {
      std::string topic = "safety_bubble/zone_" + std::to_string(i) + "/detected";
      auto pub = node_->create_publisher<std_msgs::msg::Bool>(topic, 10);
      zone_detected_pubs_.push_back(pub);
    }
  }

  void setupParameterCallback()
  {
    param_callback_handle_ = node_->add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter> & parameters) {
        return this->onParameterChange(parameters);
      });
  }

  rcl_interfaces::msg::SetParametersResult onParameterChange(
    const std::vector<rclcpp::Parameter> & parameters)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

    bool need_regenerate = false;

    for (const auto & param : parameters) {
      if (param.get_name() == "param_num_zones") {
        int new_num = param.as_int();
        if (new_num != detector_->getNumZones()) {
          detector_->setNumZones(new_num);
          createZoneDetectedPublishers();
          need_regenerate = true;
        }
      } else if (param.get_name().find("param_zone_") == 0) {
        // Parse zone parameter
        need_regenerate = true;
      }
    }

    if (need_regenerate) {
      initializeFromParameters();
    }

    return result;
  }

  rclcpp::Node * node_;
  int image_width_;
  int image_height_;

  std::unique_ptr<MultiZoneDetector> detector_;
  MultiZoneDetectionResult last_result_;

  // Publishers
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_array_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr json_status_pub_;
  std::vector<rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr> zone_detected_pubs_;

  // Parameter callback
  OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
};

}  // namespace adi_3dtof_safety_bubble_detector

#endif  // ADI_3DTOF_SAFETY_BUBBLE_DETECTOR_MULTI_ZONE_INTEGRATION_HPP
