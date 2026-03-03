/******************************************************************************
Copyright (c), 2023 - Analog Devices Inc. All Rights Reserved.
This software is PROPRIETARY & CONFIDENTIAL to Analog Devices, Inc.
and its licensors.
******************************************************************************/
#ifndef ADI_3DTOF_SAFETY_BUBBLE_DETECTOR_MULTI_ZONE_DETECTOR_HPP
#define ADI_3DTOF_SAFETY_BUBBLE_DETECTOR_MULTI_ZONE_DETECTOR_HPP

#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "zone_config.hpp"

namespace adi_3dtof_safety_bubble_detector
{

/**
 * @brief Structure to hold detection results for all zones
 */
struct MultiZoneDetectionResult
{
  std::vector<bool> zone_detected;            ///< Detection status for each zone
  bool any_detected = false;                  ///< True if any zone has detection
  int innermost_detected_zone = 0;            ///< ID of innermost zone with detection (0 if none)
  rclcpp::Time timestamp;                     ///< Timestamp of detection
};

/**
 * @brief Class for multi-zone safety bubble detection
 *
 * This class manages multiple concentric safety zones around a sensor,
 * each with independent shape, size, and detection parameters.
 */
class MultiZoneDetector
{
public:
  /**
   * @brief Construct a new MultiZoneDetector
   * @param image_width Width of the input image
   * @param image_height Height of the input image
   * @param virtual_camera_height_mtr Height of virtual camera in meters
   * @param focal_length_x Focal length in X direction
   * @param focal_length_y Focal length in Y direction
   */
  MultiZoneDetector(
    int image_width, int image_height,
    float virtual_camera_height_mtr,
    float focal_length_x, float focal_length_y);

  /**
   * @brief Destructor
   */
  ~MultiZoneDetector();

  /**
   * @brief Initialize the detector with configuration
   * @param config Multi-zone configuration
   * @return true if initialization successful
   */
  bool initialize(const MultiZoneConfig & config);

  /**
   * @brief Load configuration from YAML file
   * @param filepath Path to YAML configuration file
   * @return true if loaded successfully
   */
  bool loadConfigFromYaml(const std::string & filepath);

  /**
   * @brief Save current configuration to YAML file
   * @param filepath Path to YAML file
   * @return true if saved successfully
   */
  bool saveConfigToYaml(const std::string & filepath) const;

  /**
   * @brief Get current zone configuration
   */
  const MultiZoneConfig & getConfig() const { return config_; }

  /**
   * @brief Get mutable reference to zone configuration
   */
  MultiZoneConfig & getConfig() { return config_; }

  /**
   * @brief Set the number of zones
   * @param num_zones Number of zones to configure
   */
  void setNumZones(int num_zones);

  /**
   * @brief Get the number of zones
   */
  int getNumZones() const { return config_.getNumZones(); }

  /**
   * @brief Update a specific zone's configuration
   * @param zone_id Zone identifier (1-based)
   * @param zone_config New configuration for the zone
   * @return true if update successful
   */
  bool updateZone(int zone_id, const ZoneConfig & zone_config);

  /**
   * @brief Set detection sensitivity (minimum blob area to trigger detection)
   * @param sensitivity Minimum number of pixels for detection
   */
  void setSensitivity(int sensitivity) { sensitivity_ = sensitivity; }

  /**
   * @brief Get current sensitivity setting
   */
  int getSensitivity() const { return sensitivity_; }

  /**
   * @brief Regenerate zone masks after configuration change
   */
  void regenerateZoneMasks();

  /**
   * @brief Perform multi-zone detection on virtual camera depth image
   * @param vcam_depth_8bpp 8-bit depth image from virtual camera view
   * @param roi Region of interest for detection
   * @return Detection results for all zones
   */
  MultiZoneDetectionResult detectZones(
    const unsigned char * vcam_depth_8bpp,
    const cv::Rect & roi);

  /**
   * @brief Get the combined zone mask image (for visualization)
   * @return Reference to the combined mask
   */
  const cv::Mat & getCombinedZoneMask() const { return combined_zone_mask_; }

  /**
   * @brief Get the zone mask for a specific zone
   * @param zone_id Zone identifier (1-based)
   * @return Zone mask or empty Mat if zone not found
   */
  cv::Mat getZoneMask(int zone_id) const;

  /**
   * @brief Get the colored zone visualization image
   * @return Reference to the colored zone image
   */
  const cv::Mat & getColoredZoneImage() const { return colored_zone_image_; }

  /**
   * @brief Generate RViz visualization markers for all zones
   * @param frame_id Frame ID for the markers
   * @param detection_result Current detection results
   * @return MarkerArray for RViz visualization
   */
  visualization_msgs::msg::MarkerArray generateVisualizationMarkers(
    const std::string & frame_id,
    const MultiZoneDetectionResult & detection_result) const;

  /**
   * @brief Generate JSON status string for external integrations
   * @param detection_result Current detection results
   * @return JSON string with zone status
   */
  std::string generateJsonStatus(const MultiZoneDetectionResult & detection_result) const;

  /**
   * @brief Overlay zone boundaries on an output image
   * @param output_image Image to draw on (modified in place)
   * @param detection_result Current detection results
   * @param show_labels Whether to draw zone labels
   */
  void overlayZoneBoundaries(
    cv::Mat & output_image,
    const MultiZoneDetectionResult & detection_result,
    bool show_labels = true) const;

  /**
   * @brief Convert pixel coordinates to zone radius in pixels
   * @param radius_mtr Radius in meters
   * @return Radius in pixels
   */
  int radiusMtrToPixels(float radius_mtr) const;

  /**
   * @brief Convert dimensions to pixel coordinates
   * @param x_dim_mtr X dimension in meters
   * @param z_dim_mtr Z dimension in meters
   * @param width_pixels Output width in pixels
   * @param height_pixels Output height in pixels
   */
  void dimsMtrToPixels(float x_dim_mtr, float z_dim_mtr, int & width_pixels, int & height_pixels) const;

private:
  /**
   * @brief Create mask for a single zone
   * @param zone Zone configuration
   * @return Binary mask image
   */
  cv::Mat createZoneMask(const ZoneConfig & zone) const;

  /**
   * @brief Create exclusive zone mask (zone area minus inner zones)
   * @param zone_idx Index of the zone (0-based)
   * @return Exclusive binary mask
   */
  cv::Mat createExclusiveZoneMask(size_t zone_idx) const;

  /**
   * @brief Update the combined zone mask and colored image
   */
  void updateCombinedMasks();

  /**
   * @brief Convert OpenCV Scalar color to zone color array
   */
  cv::Scalar zoneColorToScalar(const std::array<float, 4> & color) const;

  // Configuration
  MultiZoneConfig config_;

  // Image parameters
  int image_width_;
  int image_height_;
  float virtual_camera_height_mtr_;
  float focal_length_x_;
  float focal_length_y_;
  float pixels_per_meter_;

  // Detection parameters
  int sensitivity_ = 10;

  // Zone masks (one per zone, cumulative from center)
  std::vector<cv::Mat> zone_masks_;

  // Exclusive zone masks (zone area excluding inner zones)
  std::vector<cv::Mat> exclusive_zone_masks_;

  // Combined mask for visualization
  cv::Mat combined_zone_mask_;

  // Colored zone image for visualization
  cv::Mat colored_zone_image_;

  // Thread safety
  mutable std::mutex config_mutex_;

  // Initialization flag
  bool initialized_ = false;
};

}  // namespace adi_3dtof_safety_bubble_detector

#endif  // ADI_3DTOF_SAFETY_BUBBLE_DETECTOR_MULTI_ZONE_DETECTOR_HPP
