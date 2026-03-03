/******************************************************************************
Copyright (c), 2023 - Analog Devices Inc. All Rights Reserved.
This software is PROPRIETARY & CONFIDENTIAL to Analog Devices, Inc.
and its licensors.
******************************************************************************/
#include <cv_bridge/cv_bridge.h>
#include <image_geometry/pinhole_camera_model.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.h>
#include <tf2/LinearMath/Quaternion.h>

#include <chrono>
#include <functional>
#include <image_transport/image_transport.hpp>
#include <memory>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <pcl_ros/transforms.hpp>
#include <queue>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/distortion_models.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <utility>
#include <sstream>
#include <iomanip>

// Include custom message types for zone status
#include "adi_3dtof_safety_bubble_detector/msg/zone_status.hpp"
#include "adi_3dtof_safety_bubble_detector/msg/zone_status_array.hpp"

#include "image_proc_utils.h"

using namespace std::chrono_literals;
namespace enc = sensor_msgs::image_encodings;

// Maximum queue size for time synchronization
#define MAX_QUEUE_SIZE_FOR_TIME_SYNC 5

/**
 * @brief Safety Bubble Detector Stitch Host Node
 *
 * This class demonstrates how to integrate multiple SBD nodes running on remote
 * robots into a centralized host monitoring system. Key features:
 *
 * - Supports 1-4 cameras/SBD nodes
 * - Time-synchronized multi-camera processing using message_filters
 * - Combines zone detection status (logical OR across cameras)
 * - Creates stitched visualization with detection indicator
 * - Handles both compressed and uncompressed image transport
 *
 * ARCHITECTURE:
 * -------------
 * For single camera: Direct subscription callbacks
 * For multiple cameras: message_filters::Synchronizer with ApproximateTime policy
 *
 * MESSAGE FLOW:
 * -------------
 * 1. SBD nodes publish zone_status and out_image topics
 * 2. This node subscribes and synchronizes messages (multi-camera only)
 * 3. Callbacks store received data in internal buffers
 * 4. Timer callback (timerCallback) stitches data when all sensors ready
 * 5. Combined results published to /combo_safety_bubble/ topics
 *
 */
class ADI3DToFSafetyBubbleStitch : public rclcpp::Node
{
public:
  static const int MAX_NUM_DEVICES = 10;   ///< Maximum supported cameras
  static const int MAX_NUM_ZONES = 10;     ///< Maximum safety zones per camera

  /**
   * @brief Construct a new ADI3DToFSafetyBubbleStitch object
   *
   * Initialization sequence:
   * 1. Declare and get ROS parameters
   * 2. Create subscribers for each camera (zone_status and out_image)
   * 3. Setup time synchronizers for multi-camera configurations
   * 4. Create publishers for combined outputs
   * 5. Start timer for periodic processing
   */
  ADI3DToFSafetyBubbleStitch()
  : Node("adi_3dtof_safety_bubble_detector_stitch"),
    synchronizer_for_2_compressed_images(
      sync_policy_for_2_compressed_images(MAX_QUEUE_SIZE_FOR_TIME_SYNC),
      compressed_out_image_subscriber_[0], compressed_out_image_subscriber_[1]),
    synchronizer_for_3_compressed_images(
      sync_policy_for_3_compressed_images(MAX_QUEUE_SIZE_FOR_TIME_SYNC),
      compressed_out_image_subscriber_[0], compressed_out_image_subscriber_[1],
      compressed_out_image_subscriber_[2]),
    synchronizer_for_4_compressed_images(
      sync_policy_for_4_compressed_images(MAX_QUEUE_SIZE_FOR_TIME_SYNC),
      compressed_out_image_subscriber_[0], compressed_out_image_subscriber_[1],
      compressed_out_image_subscriber_[2], compressed_out_image_subscriber_[3]),
    synchronizer_for_2_images(
      sync_policy_for_2_images(MAX_QUEUE_SIZE_FOR_TIME_SYNC), out_image_subscriber_[0],
      out_image_subscriber_[1]),
    synchronizer_for_3_images(
      sync_policy_for_3_images(MAX_QUEUE_SIZE_FOR_TIME_SYNC), out_image_subscriber_[0],
      out_image_subscriber_[1], out_image_subscriber_[2]),
    synchronizer_for_4_images(
      sync_policy_for_4_images(MAX_QUEUE_SIZE_FOR_TIME_SYNC), out_image_subscriber_[0],
      out_image_subscriber_[1], out_image_subscriber_[2], out_image_subscriber_[3])
  {
    RCLCPP_INFO(this->get_logger(), "=== Safety Bubble Detector Stitch Host Node ===");
    RCLCPP_INFO(this->get_logger(), "Initializing multi-camera integration node...");

    // ========================================================================
    // STEP 1: Declare and retrieve ROS parameters
    // ========================================================================
    // param_camera_prefixes: List of camera namespace prefixes (required)
    // Example: ["cam1", "cam2", "cam3", "cam4"] for 4 cameras
    this->declare_parameter("param_camera_prefixes", rclcpp::PARAMETER_STRING_ARRAY);
    
    // param_use_compressed: Whether to use compressed image transport (optional)
    // Set to true to reduce network bandwidth (especially for remote hosts)
    this->declare_parameter("param_use_compressed", false);
    
    // param_update_rate_hz: Processing rate in Hz (optional)
    // Default: 30 Hz (matches typical camera frame rates)
    this->declare_parameter("param_update_rate_hz", 30.0);

    // Get camera prefixes from parameters
    rclcpp::Parameter string_array_param = this->get_parameter("param_camera_prefixes");
    camera_prefixes_ = string_array_param.as_string_array();
    
    use_compressed_ = this->get_parameter("param_use_compressed").as_bool();
    double update_rate_hz = this->get_parameter("param_update_rate_hz").as_double();
    
    // Validate configuration
    if (camera_prefixes_.empty()) {
      RCLCPP_ERROR(this->get_logger(), "ERROR: param_camera_prefixes is empty!");
      RCLCPP_ERROR(this->get_logger(), "Please specify camera prefixes, e.g.:");
      RCLCPP_ERROR(this->get_logger(), "  --ros-args -p param_camera_prefixes:=\"['cam1','cam2']\"");
      throw std::runtime_error("No camera prefixes specified");
    }

    RCLCPP_INFO(this->get_logger(), "Configuration:");
    RCLCPP_INFO(this->get_logger(), "  Number of cameras: %zu", camera_prefixes_.size());
    for (size_t i = 0; i < camera_prefixes_.size(); i++) {
      RCLCPP_INFO(this->get_logger(), "    Camera %zu: %s", i + 1, camera_prefixes_[i].c_str());
    }
    RCLCPP_INFO(this->get_logger(), "  Image transport: %s", 
                use_compressed_ ? "COMPRESSED (JPEG)" : "UNCOMPRESSED (RAW)");
    RCLCPP_INFO(this->get_logger(), "  Update rate: %.1f Hz", update_rate_hz);

    // Limit to maximum supported devices
    int max_devices_allowed =
      static_cast<int>(sizeof(out_image_subscriber_) / sizeof(out_image_subscriber_[0]));
    num_sensors_ = std::min(max_devices_allowed, static_cast<int>(camera_prefixes_.size()));

    if (num_sensors_ > 4) {
      RCLCPP_WARN(this->get_logger(), "WARNING: Maximum 4 cameras supported. Using first 4.");
      num_sensors_ = 4;
    }

    // ========================================================================
    // STEP 2: Create subscribers for each camera
    // ========================================================================
    RCLCPP_INFO(this->get_logger(), "Creating subscribers for %d cameras...", num_sensors_);
    
    for (int i = 0; i < num_sensors_; i++) {
      // --- Zone Status Subscriber ---
      // Subscribes to multi-zone detection status from each SBD node
      // Topic: /<camera_prefix>/zone_status
      // Type: adi_3dtof_safety_bubble_detector::msg::ZoneStatusArray
      std::function<void(const adi_3dtof_safety_bubble_detector::msg::ZoneStatusArray::SharedPtr)>
        zone_status_callback_func = std::bind(
          &ADI3DToFSafetyBubbleStitch::zoneStatusCallback, this, std::placeholders::_1, i);

      zone_status_subscriber_[i] = 
        this->create_subscription<adi_3dtof_safety_bubble_detector::msg::ZoneStatusArray>(
          "/" + camera_prefixes_[i] + "/zone_status", 10, zone_status_callback_func);

      RCLCPP_INFO(this->get_logger(), "  [%d] Subscribed to: /%s/zone_status", 
                  i + 1, camera_prefixes_[i].c_str());

      // --- Image Subscribers (synchronized for multi-camera) ---
      // For time synchronization, we use message_filters::Subscriber
      // Topic: /<camera_prefix>/out_image or /out_image/compressed
      out_image_subscriber_[i].subscribe(this, "/" + camera_prefixes_[i] + "/out_image");
      compressed_out_image_subscriber_[i].subscribe(
        this, "/" + camera_prefixes_[i] + "/out_image/compressed");

      RCLCPP_INFO(this->get_logger(), "  [%d] Subscribed to: /%s/out_image%s", 
                  i + 1, camera_prefixes_[i].c_str(),
                  use_compressed_ ? "/compressed" : "");

      // Initialize state flags
      out_image_recvd_[i] = false;
      zone_status_recvd_[i] = false;
      out_image_[i] = nullptr;
      
      // Initialize zone detection arrays
      for (int j = 0; j < MAX_NUM_ZONES; j++) {
        zone_detected_per_camera_[i][j] = false;
      }
    }

    // ========================================================================
    // STEP 3: Setup time synchronizers based on number of cameras
    // ========================================================================
    // For multi-camera setups, we use message_filters to synchronize messages
    // based on timestamps. This ensures we process frames from the same time instant.
    //
    // ApproximateTime policy allows small timestamp differences (useful when
    // cameras don't have perfectly synchronized clocks).
    //
    // For single camera, we use direct subscription (no synchronization needed).
    
    RCLCPP_INFO(this->get_logger(), "Setting up message synchronization...");
    
    if (num_sensors_ == 1) {
      // Single camera: Direct subscription, no time synchronization needed
      RCLCPP_INFO(this->get_logger(), "  Mode: SINGLE CAMERA (direct subscription)");
      
      if (use_compressed_) {
        std::function<void(const sensor_msgs::msg::CompressedImage::SharedPtr)>
          compressed_callback = std::bind(
            &ADI3DToFSafetyBubbleStitch::compressedOutImageCallback, 
            this, std::placeholders::_1, 0);
        single_compressed_out_image_subscriber_ =
          this->create_subscription<sensor_msgs::msg::CompressedImage>(
            "/" + camera_prefixes_[0] + "/out_image/compressed", 10, compressed_callback);
      } else {
        std::function<void(const sensor_msgs::msg::Image::SharedPtr)>
          image_callback = std::bind(
            &ADI3DToFSafetyBubbleStitch::outImageCallback, 
            this, std::placeholders::_1, 0);
        single_out_image_subscriber_ = this->create_subscription<sensor_msgs::msg::Image>(
          "/" + camera_prefixes_[0] + "/out_image", 10, image_callback);
      }
    } else if (num_sensors_ == 2) {
      // Two cameras: Time synchronization with ApproximateTime policy
      RCLCPP_INFO(this->get_logger(), "  Mode: TWO CAMERAS (time-synchronized)");
      
      if (use_compressed_) {
        synchronizer_for_2_compressed_images.registerCallback(
          &ADI3DToFSafetyBubbleStitch::sync2CamerasCompressedOutImageCallback, this);
      } else {
        synchronizer_for_2_images.registerCallback(
          &ADI3DToFSafetyBubbleStitch::sync2CamerasOutImageCallback, this);
      }
    } else if (num_sensors_ == 3) {
      // Three cameras: Time synchronization
      RCLCPP_INFO(this->get_logger(), "  Mode: THREE CAMERAS (time-synchronized)");
      
      if (use_compressed_) {
        synchronizer_for_3_compressed_images.registerCallback(
          &ADI3DToFSafetyBubbleStitch::sync3CamerasCompressedOutImageCallback, this);
      } else {
        synchronizer_for_3_images.registerCallback(
          &ADI3DToFSafetyBubbleStitch::sync3CamerasOutImageCallback, this);
      }
    } else if (num_sensors_ == 4) {
      // Four cameras: Time synchronization
      RCLCPP_INFO(this->get_logger(), "  Mode: FOUR CAMERAS (time-synchronized)");
      
      if (use_compressed_) {
        synchronizer_for_4_compressed_images.registerCallback(
          &ADI3DToFSafetyBubbleStitch::sync4CamerasCompressedOutImageCallback, this);
      } else {
        synchronizer_for_4_images.registerCallback(
          &ADI3DToFSafetyBubbleStitch::sync4CamerasOutImageCallback, this);
      }
    }

    // ========================================================================
    // STEP 4: Create publishers for combined outputs
    // ========================================================================
    RCLCPP_INFO(this->get_logger(), "Creating publishers...");
    
    // Combined visualization image
    // Shows stitched view from all cameras with detection indicator
    combo_out_image_publisher_ =
      this->create_publisher<sensor_msgs::msg::Image>("combo_safety_bubble/out_image", 10);
    RCLCPP_INFO(this->get_logger(), "  Publishing: /combo_safety_bubble/out_image");
    
    // Combined zone status (logical OR across all cameras)
    // Each zone shows detected=true if ANY camera detects in that zone
    combo_zone_status_publisher_ =
      this->create_publisher<adi_3dtof_safety_bubble_detector::msg::ZoneStatusArray>(
        "combo_safety_bubble/zone_status", 10);
    RCLCPP_INFO(this->get_logger(), "  Publishing: /combo_safety_bubble/zone_status");
    
    // JSON string version of zone status (no custom message dependency needed!)
    combo_zone_status_json_publisher_ =
      this->create_publisher<std_msgs::msg::String>(
        "combo_safety_bubble/zone_status_json", 10);
    RCLCPP_INFO(this->get_logger(), "  Publishing: /combo_safety_bubble/zone_status_json (JSON format)");

    // ========================================================================
    // STEP 5: Start processing timer
    // ========================================================================
    // Timer periodically checks if all camera data is ready and performs stitching
    auto period_ms = std::chrono::milliseconds(static_cast<int>(1000.0 / update_rate_hz));
    timer_ = this->create_wall_timer(
      period_ms, std::bind(&ADI3DToFSafetyBubbleStitch::timerCallback, this));

    RCLCPP_INFO(this->get_logger(), "Timer started (period: %d ms)", 
                static_cast<int>(1000.0 / update_rate_hz));

    // Initialize frame counter
    frame_counter_ = 0;
    
    RCLCPP_INFO(this->get_logger(), "=== Initialization Complete ===");
    RCLCPP_INFO(this->get_logger(), "Waiting for data from SBD nodes...");
    RCLCPP_INFO(this->get_logger(), " ");
    RCLCPP_INFO(this->get_logger(), "USAGE TIP: View combined output with:");
    RCLCPP_INFO(this->get_logger(), "  ros2 run rqt_image_view rqt_image_view /combo_safety_bubble/out_image");
    RCLCPP_INFO(this->get_logger(), "Monitor zone status with:");
    RCLCPP_INFO(this->get_logger(), "  ros2 topic echo /combo_safety_bubble/zone_status");
    RCLCPP_INFO(this->get_logger(), "Or without custom message dependency:");
    RCLCPP_INFO(this->get_logger(), "  ros2 topic echo /combo_safety_bubble/zone_status_json");
  }

  void timerCallback() { stitchFrames(); }

  /**
   * @brief Destroy the ADI3DToFSafetyBubbleStitch object
   *
   */
  ~ADI3DToFSafetyBubbleStitch() override
  {
    for (int i = 0; i < num_sensors_; i++) {
      if (out_image_[i] != nullptr) {
        delete[] out_image_[i];
        out_image_[i] = nullptr;
      }
    }
  }

  /**
   * @brief
   *
   * @param compressed_out_image_cam1
   * @param compressed_out_image_cam2
   */
  void sync2CamerasCompressedOutImageCallback(
    const sensor_msgs::msg::CompressedImage::ConstSharedPtr & compressed_out_image_cam1,
    const sensor_msgs::msg::CompressedImage::ConstSharedPtr & compressed_out_image_cam2)
  {
    // Call respective callbacks with the id.
    compressedOutImageCallback(compressed_out_image_cam1, 0);
    compressedOutImageCallback(compressed_out_image_cam2, 1);
  }

  /**
   * @brief
   *
   * @param out_image_cam1
   * @param out_image_cam2
   */
  void sync2CamerasOutImageCallback(
    const sensor_msgs::msg::Image::ConstSharedPtr & out_image_cam1,
    const sensor_msgs::msg::Image::ConstSharedPtr & out_image_cam2)
  {
    // Call respective callbacks with the id.
    outImageCallback(out_image_cam1, 0);
    outImageCallback(out_image_cam2, 1);
  }

  /**
   * @brief
   *
   * @param compressed_out_image_cam1
   * @param compressed_out_image_cam2
   * @param compressed_out_image_cam3
   */
  void sync3CamerasCompressedOutImageCallback(
    const sensor_msgs::msg::CompressedImage::ConstSharedPtr & compressed_out_image_cam1,
    const sensor_msgs::msg::CompressedImage::ConstSharedPtr & compressed_out_image_cam2,
    const sensor_msgs::msg::CompressedImage::ConstSharedPtr & compressed_out_image_cam3)
  {
    // Call respective callbacks with the id.
    compressedOutImageCallback(compressed_out_image_cam1, 0);
    compressedOutImageCallback(compressed_out_image_cam2, 1);
    compressedOutImageCallback(compressed_out_image_cam3, 2);
  }

  /**
   * @brief
   *
   * @param compressed_out_image_cam1
   * @param compressed_out_image_cam2
   * @param compressed_out_image_cam3
   */
  void sync3CamerasOutImageCallback(
    const sensor_msgs::msg::Image::ConstSharedPtr & out_image_cam1,
    const sensor_msgs::msg::Image::ConstSharedPtr & out_image_cam2,
    const sensor_msgs::msg::Image::ConstSharedPtr & out_image_cam3)
  {
    // Call respective callbacks with the id.
    outImageCallback(out_image_cam1, 0);
    outImageCallback(out_image_cam2, 1);
    outImageCallback(out_image_cam3, 2);
  }

  /**
   * @brief
   *
   * @param compressed_out_image_cam1
   * @param compressed_out_image_cam2
   * @param compressed_out_image_cam3
   * @param compressed_out_image_cam4
   */
  void sync4CamerasCompressedOutImageCallback(
    const sensor_msgs::msg::CompressedImage::ConstSharedPtr & compressed_out_image_cam1,
    const sensor_msgs::msg::CompressedImage::ConstSharedPtr & compressed_out_image_cam2,
    const sensor_msgs::msg::CompressedImage::ConstSharedPtr & compressed_out_image_cam3,
    const sensor_msgs::msg::CompressedImage::ConstSharedPtr & compressed_out_image_cam4)
  {
    // Call respective callbacks with the id.
    compressedOutImageCallback(compressed_out_image_cam1, 0);
    compressedOutImageCallback(compressed_out_image_cam2, 1);
    compressedOutImageCallback(compressed_out_image_cam3, 2);
    compressedOutImageCallback(compressed_out_image_cam4, 3);
  }

  /**
   * @brief
   *
   * @param out_image_cam1
   * @param out_image_cam2
   * @param out_image_cam3
   * @param out_image_cam4
   */
  void sync4CamerasOutImageCallback(
    const sensor_msgs::msg::Image::ConstSharedPtr & out_image_cam1,
    const sensor_msgs::msg::Image::ConstSharedPtr & out_image_cam2,
    const sensor_msgs::msg::Image::ConstSharedPtr & out_image_cam3,
    const sensor_msgs::msg::Image::ConstSharedPtr & out_image_cam4)
  {
    // Call respective callbacks with the id.
    outImageCallback(out_image_cam1, 0);
    outImageCallback(out_image_cam2, 1);
    outImageCallback(out_image_cam3, 2);
    outImageCallback(out_image_cam4, 3);
  }

  /**
   * @brief Low-level callback for ouput image
   *
   * @param message image pointer
   * @param cam_id camera ID
   */
  void outImageCallback(const sensor_msgs::msg::Image::ConstSharedPtr & message, int cam_id)
  {
    sensor_msgs::msg::Image::ConstSharedPtr out_image;

    out_image = message;

    image_width_ = out_image->width;
    image_height_ = out_image->height;
    if ((image_width_ != 512) && (image_height_ != 512)) {
      return;
    }

    if (out_image_[cam_id] == nullptr) {
      out_image_[cam_id] =
        new unsigned char[image_width_ * image_height_ * 3];  //*3 assumed RGB format
    }

    // copy
    memcpy(out_image_[cam_id], &out_image->data[0], image_width_ * image_height_ * 3);

    // Set flag
    out_image_recvd_[cam_id] = true;
  }

  /**
   * @brief Low-level callback for ouput image
   *
   * @param message compressed image pointer
   * @param cam_id camera ID
   */
  void compressedOutImageCallback(
    const sensor_msgs::msg::CompressedImage::ConstSharedPtr & message, int cam_id)
  {
    int imdecode_flag = cv::IMREAD_COLOR;
    sensor_msgs::msg::Image::ConstSharedPtr out_image;

    cv_bridge::CvImagePtr cv_ptr(new cv_bridge::CvImage);

    // Copy message header
    cv_ptr->header = message->header;
    cv_ptr->encoding = enc::BGR8;

    // Decode the color image
    cv_ptr->image = cv::imdecode(cv::Mat(message->data), imdecode_flag);

    size_t rows = cv_ptr->image.rows;
    size_t cols = cv_ptr->image.cols;

    if ((rows <= 0) || (cols <= 0)) {
      return;
    }
    out_image = (cv_ptr->toImageMsg());

    image_width_ = out_image->width;
    image_height_ = out_image->height;
    if ((image_width_ != 512) && (image_height_ != 512)) {
      return;
    }
    if (out_image_[cam_id] == nullptr) {
      out_image_[cam_id] =
        new unsigned char[image_width_ * image_height_ * 3];  //*3 assumed RGB format
    }

    // copy
    memcpy(out_image_[cam_id], &out_image->data[0], image_width_ * image_height_ * 3);

    // Set flag
    out_image_recvd_[cam_id] = true;
  }

  /**
   * @brief Callback for zone status from individual SBD nodes
   *
   * This callback receives multi-zone detection status from each camera.
   * It stores the complete zone status array and extracts detection flags
   * for each zone for later combination across all cameras.   
   * This is where you receive detailed zone information including:
   * - Zone ID and enabled status
   * - Detection status per zone
   * - Zone shape (circle/rectangle) and dimensions
   * - Visualization colors
   *
   * @param zone_status Zone status array message from SBD node
   * @param cam_id Camera identifier (0-based index)
   */
  void zoneStatusCallback(
    const adi_3dtof_safety_bubble_detector::msg::ZoneStatusArray::SharedPtr & zone_status,
    int cam_id)
  {
    if (zone_status == nullptr) {
      return;
    }

    // Store detection status for each zone
    // This allows us to track which specific zones are triggered
    for (size_t i = 0; i < zone_status->zones.size() && i < MAX_NUM_ZONES; ++i) {
      zone_detected_per_camera_[cam_id][i] = zone_status->zones[i].detected;
    }

    // Store complete zone status for later combination
    // The complete status includes zone dimensions, colors, etc.
    zone_status_per_camera_[cam_id] = *zone_status;
    
    // Set flag indicating we received zone status from this camera
    zone_status_recvd_[cam_id] = true;
  }

  /**
   * @brief Combine zone status from multiple cameras
   *
   * Creates a combined zone status array where each zone is marked as detected
   * if ANY camera detects an object in that zone (logical OR).   
   *
   * @return Combined zone status array with merged detection results
   */
  adi_3dtof_safety_bubble_detector::msg::ZoneStatusArray combineZoneStatus()
  {
    adi_3dtof_safety_bubble_detector::msg::ZoneStatusArray combined;
    
    if (num_sensors_ == 0) {
      return combined;
    }

    // Use first camera's zone configuration as template
    // This assumes all cameras have the same zone configuration
    combined = zone_status_per_camera_[0];
    combined.header.stamp = curr_frame_timestamp_;
    combined.header.frame_id = "combo_safety_bubble";

    // Combine detection status across all cameras using logical OR
    for (size_t zone_idx = 0; zone_idx < combined.zones.size(); ++zone_idx) {
      bool zone_detected = false;
      
      // Check if ANY camera detected an object in this zone
      for (int cam_id = 0; cam_id < num_sensors_; ++cam_id) {
        if (zone_idx < zone_status_per_camera_[cam_id].zones.size()) {
          if (zone_status_per_camera_[cam_id].zones[zone_idx].detected) {
            zone_detected = true;
            break;  // One detection is enough for OR logic
          }
        }
      }
      
      // Update combined zone status
      combined.zones[zone_idx].detected = zone_detected;
    }

    return combined;
  }

  /**
   * @brief Stitch function: combines images and zone status from all cameras
   *
   * This is the main processing function that:
   * 1. Waits for data from all cameras (zone_status + out_image)
   * 2. Combines zone detection status using combineZoneStatus()
   * 3. Stitches visualization images from all cameras
   * 4. Adds detection indicator (red/green box)
   * 5. Publishes combined results
   *
   * The function only processes when data from ALL cameras is available,
   * ensuring synchronized multi-camera operation.
   *
   * @return true if stitching was performed, false if waiting for data
   */
  bool stitchFrames()
  {
    // ========================================================================
    // STEP 1: Check if all camera data has been received
    // ========================================================================
    // We need both zone_status and out_image from each camera before processing
    bool all_callbacks_recvd = true;
    for (int i = 0; i < num_sensors_; i++) {
      if ((!out_image_recvd_[i]) || (!zone_status_recvd_[i])) {
        all_callbacks_recvd = false;
        break;
      }
    }

    // If we don't have data from all cameras yet, wait for next timer tick
    if (!all_callbacks_recvd) {
      return false;
    }

    // ========================================================================
    // STEP 2: Process synchronized multi-camera data
    // ========================================================================
    RCLCPP_INFO_STREAM(
      this->get_logger(),
      "Processing frame " << frame_counter_ << " from " << num_sensors_ << " camera(s)");

    // Update timestamp for this synchronized frame set
    curr_frame_timestamp_ = this->now();

    // ========================================================================
    // STEP 3: Combine zone status from all cameras
    // ========================================================================
    auto combined_zone_status = combineZoneStatus();
    
    // Check if any zone has a detection
    bool any_zone_detected = false;
    for (const auto& zone : combined_zone_status.zones) {
      if (zone.detected) {
        any_zone_detected = true;
        break;
      }
    }
    
    // Log detection status (useful for debugging)
    if (any_zone_detected) {
      RCLCPP_WARN(this->get_logger(), "⚠️  OBJECT DETECTED in safety zones!");
      for (size_t i = 0; i < combined_zone_status.zones.size(); ++i) {
        if (combined_zone_status.zones[i].detected) {
          RCLCPP_WARN(this->get_logger(), "    Zone %d: DETECTED", 
                      combined_zone_status.zones[i].id);
        }
      }
    }
    
    // Publish combined zone status (custom message)
    combo_zone_status_publisher_->publish(combined_zone_status);
    
    // Publish combined zone status as JSON (standard message - no dependency needed!)
    std_msgs::msg::String json_msg;
    json_msg.data = convertZoneStatusToJson(combined_zone_status);
    combo_zone_status_json_publisher_->publish(json_msg);

    // ========================================================================
    // STEP 4: Stitch visualization images
    // ========================================================================
    // The individual sensor output images already have zones drawn on them
    // We just need to OR them together to get the combined view
    
    // Start with first camera's image (make a copy to avoid modifying original buffer)
    cv::Mat stitched_image = 
      cv::Mat(cv::Size(image_width_, image_height_), CV_8UC3, out_image_[0]).clone();

    // Blend images from remaining cameras with reduced alpha to avoid over-brightness
    // Using addWeighted with alpha < 1.0 prevents zone colors from becoming too saturated
    // when multiple cameras overlay the same area
    const double blend_alpha = 0.5;  // Adjust this value: lower = more transparent overlays
    
    for (int i = 1; i < num_sensors_; i++) {
      cv::Mat cam_image = 
        cv::Mat(cv::Size(image_width_, image_height_), CV_8UC3, out_image_[i]);
      
      // Blend: result = alpha * stitched_image + (1-alpha) * cam_image
      // This prevents color saturation when zones overlap
      cv::addWeighted(stitched_image, 1.0, cam_image, blend_alpha, 0.0, stitched_image);
    }

    // ========================================================================
    // STEP 5: Draw zone occupancy indicators
    // ========================================================================
    // Mask off top-left area and draw indicator boxes showing combined zone status
    // Format matches individual SBD nodes: 20x20 boxes with zone colors
    
    int box_size = 20;
    int box_spacing = 5;
    int start_x = 8;
    int start_y = 10;
    int label_height = 20;
    
    // Calculate the region containing all indicator boxes and labels
    int num_boxes = static_cast<int>(combined_zone_status.zones.size());
    int mask_width = std::max(num_boxes, 1) * (box_size + box_spacing) + start_x;
    int mask_height = start_y + box_size + label_height;
    cv::Rect indicator_region(0, 0, mask_width, mask_height);
    
    // Mask the indicator region to black (clear any overlaid zones from individual images)
    cv::rectangle(stitched_image, indicator_region, cv::Scalar(0, 0, 0), -1);
    
    // Draw indicator box for each zone based on combined_zone_status
    for (int zone_idx = 0; zone_idx < num_boxes; ++zone_idx) {
      const auto& zone = combined_zone_status.zones[zone_idx];
      
      // Calculate position for this zone's indicator
      int x_pos = start_x + zone_idx * (box_size + box_spacing);
      int y_pos = start_y;
      cv::Rect box(x_pos, y_pos, box_size, box_size);
      
      // Convert zone color from RGBA (0.0-1.0) to BGR (0-255)
      cv::Scalar bgr_color(
        static_cast<int>(zone.color[2] * 255),  // B
        static_cast<int>(zone.color[1] * 255),  // G
        static_cast<int>(zone.color[0] * 255)   // R
      );
      
      // Dim color for non-detected zones (approximately 1/3 brightness)
      cv::Scalar dim_color(
        bgr_color[0] / 3,
        bgr_color[1] / 3,
        bgr_color[2] / 3
      );
      
      if (!zone.enabled) {
        // Disabled zone: gray box with X
        cv::rectangle(stitched_image, box, cv::Scalar(100, 100, 100), -1);
        cv::line(stitched_image, 
                 cv::Point(x_pos, y_pos), 
                 cv::Point(x_pos + box_size, y_pos + box_size), 
                 cv::Scalar(200, 200, 200), 2);
        cv::line(stitched_image, 
                 cv::Point(x_pos + box_size, y_pos), 
                 cv::Point(x_pos, y_pos + box_size), 
                 cv::Scalar(200, 200, 200), 2);
      } else if (zone.detected) {
        // Detected: bright fill only
        cv::rectangle(stitched_image, box, bgr_color, -1);
      } else {
        // Not detected: dim fill + bright outline
        cv::rectangle(stitched_image, box, dim_color, -1);
        cv::rectangle(stitched_image, box, bgr_color, 2);
      }
      
      // Add zone ID label below box (white text)
      std::string zone_label = std::to_string(zone.id);
      cv::Point text_pos(x_pos + 6, y_pos + box_size + 15);
      cv::putText(
        stitched_image, zone_label, text_pos,
        cv::FONT_HERSHEY_SIMPLEX, 0.4,
        cv::Scalar(255, 255, 255), 1);
    }

    // ========================================================================
    // STEP 6: Publish stitched image
    // ========================================================================
    publishImageAsRosMsg(
      stitched_image, "bgr8", "combo_safety_bubble", combo_out_image_publisher_);

    // ========================================================================
    // STEP 7: Reset flags and prepare for next frame
    // ========================================================================
    for (int i = 0; i < num_sensors_; i++) {
      out_image_recvd_[i] = false;
      zone_status_recvd_[i] = false;
    }

    // Update frame counter
    frame_counter_++;

    return true;
  }

private:
  // ========================================================================
  // Image dimensions and frame tracking
  // ========================================================================
  int image_width_ = 512;   ///< Expected image width from SBD nodes
  int image_height_ = 512;  ///< Expected image height from SBD nodes
  int frame_counter_;       ///< Counter for processed frames

  // ========================================================================
  // Configuration
  // ========================================================================
  std::vector<std::string> camera_prefixes_;  ///< Camera namespace prefixes from parameters
  bool use_compressed_;                        ///< Whether to use compressed image transport
  int num_sensors_;                            ///< Number of cameras/SBD nodes

  // ========================================================================
  // ROS Communication
  // ========================================================================
  rclcpp::TimerBase::SharedPtr timer_;  ///< Timer for periodic processing

  // --- Zone Status Subscribers (NEW - Multi-zone support) ---
  rclcpp::Subscription<adi_3dtof_safety_bubble_detector::msg::ZoneStatusArray>::SharedPtr 
    zone_status_subscriber_[MAX_NUM_DEVICES];

  // --- Image Subscribers ---
  // For single camera: direct subscription
  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr
    single_compressed_out_image_subscriber_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr 
    single_out_image_subscriber_;
  
  // For multiple cameras: message_filters for time synchronization
  message_filters::Subscriber<sensor_msgs::msg::CompressedImage>
    compressed_out_image_subscriber_[MAX_NUM_DEVICES];
  message_filters::Subscriber<sensor_msgs::msg::Image> 
    out_image_subscriber_[MAX_NUM_DEVICES];

  // --- Publishers ---
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr 
    combo_out_image_publisher_;  ///< Combined visualization image
  rclcpp::Publisher<adi_3dtof_safety_bubble_detector::msg::ZoneStatusArray>::SharedPtr 
    combo_zone_status_publisher_;  ///< Combined zone status (NEW)
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr 
    combo_zone_status_json_publisher_;  ///< Combined zone status as JSON string

  // ========================================================================
  // Data Buffers and Flags
  // ========================================================================
  
  // --- Zone Status Data (NEW - Multi-zone support) ---
  bool zone_status_recvd_[MAX_NUM_DEVICES];  ///< Flags: zone status received
  bool zone_detected_per_camera_[MAX_NUM_DEVICES][MAX_NUM_ZONES];  ///< Detection per zone per camera
  adi_3dtof_safety_bubble_detector::msg::ZoneStatusArray 
    zone_status_per_camera_[MAX_NUM_DEVICES];  ///< Complete zone status from each camera

  // --- Image Data ---
  bool out_image_recvd_[MAX_NUM_DEVICES];      ///< Flags: image received
  unsigned char * out_image_[MAX_NUM_DEVICES]; ///< Raw image buffers

  // ========================================================================
  // Time Synchronizers for Multi-Camera Setups
  // ========================================================================
  // These use message_filters::Synchronizer with ApproximateTime policy
  // to synchronize messages from multiple cameras based on timestamps
  typedef message_filters::sync_policies::ApproximateTime<
    sensor_msgs::msg::CompressedImage, sensor_msgs::msg::CompressedImage>
    sync_policy_for_2_compressed_images;
  message_filters::Synchronizer<sync_policy_for_2_compressed_images>
    synchronizer_for_2_compressed_images;
  typedef message_filters::sync_policies::ApproximateTime<
    sensor_msgs::msg::CompressedImage, sensor_msgs::msg::CompressedImage,
    sensor_msgs::msg::CompressedImage>
    sync_policy_for_3_compressed_images;
  message_filters::Synchronizer<sync_policy_for_3_compressed_images>
    synchronizer_for_3_compressed_images;
  typedef message_filters::sync_policies::ApproximateTime<
    sensor_msgs::msg::CompressedImage, sensor_msgs::msg::CompressedImage,
    sensor_msgs::msg::CompressedImage, sensor_msgs::msg::CompressedImage>
    sync_policy_for_4_compressed_images;
  message_filters::Synchronizer<sync_policy_for_4_compressed_images>
    synchronizer_for_4_compressed_images;
  typedef message_filters::sync_policies::ApproximateTime<
    sensor_msgs::msg::Image, sensor_msgs::msg::Image>
    sync_policy_for_2_images;
  message_filters::Synchronizer<sync_policy_for_2_images> synchronizer_for_2_images;
  typedef message_filters::sync_policies::ApproximateTime<
    sensor_msgs::msg::Image, sensor_msgs::msg::Image, sensor_msgs::msg::Image>
    sync_policy_for_3_images;
  message_filters::Synchronizer<sync_policy_for_3_images> synchronizer_for_3_images;
  typedef message_filters::sync_policies::ApproximateTime<
    sensor_msgs::msg::Image, sensor_msgs::msg::Image, sensor_msgs::msg::Image,
    sensor_msgs::msg::Image>
    sync_policy_for_4_images;
  message_filters::Synchronizer<sync_policy_for_4_images> synchronizer_for_4_images;

  rclcpp::Time curr_frame_timestamp_ = rclcpp::Clock{}.now();

  /**
   * @brief Convert ZoneStatusArray to JSON string for standard message publishing
   *
   * This allows subscribing to zone status without building the custom message package.
   * JSON format allows easy parsing in any language (Python, C++, JavaScript, etc.)
   *
   * @param zone_status The zone status array to convert
   * @return JSON string representation
   */
  std::string convertZoneStatusToJson(
    const adi_3dtof_safety_bubble_detector::msg::ZoneStatusArray & zone_status)
  {
    std::ostringstream json;
    json << std::fixed << std::setprecision(3);
    
    json << "{";
    json << "\"timestamp\":" << zone_status.header.stamp.sec << "." 
         << std::setfill('0') << std::setw(9) << zone_status.header.stamp.nanosec << ",";
    json << "\"frame_id\":\"" << zone_status.header.frame_id << "\",";
    json << "\"zones\":[";
    
    for (size_t i = 0; i < zone_status.zones.size(); ++i) {
      const auto& zone = zone_status.zones[i];
      if (i > 0) json << ",";
      
      json << "{";
      json << "\"id\":" << zone.id << ",";
      json << "\"enabled\":" << (zone.enabled ? "true" : "false") << ",";
      json << "\"detected\":" << (zone.detected ? "true" : "false") << ",";
      json << "\"shape\":\"" << zone.shape << "\",";
      
      // Zone dimensions (shape-specific)
      if (zone.shape == "circle") {
        json << "\"radius_mtr\":" << zone.radius_mtr << ",";
      } else if (zone.shape == "rectangle") {
        json << "\"x_dim_mtr\":" << zone.x_dim_mtr << ",";
        json << "\"z_dim_mtr\":" << zone.z_dim_mtr << ",";
      }
      
      // Zone color (RGBA)
      json << "\"color\":[";
      for (size_t j = 0; j < zone.color.size() && j < 4; ++j) {
        if (j > 0) json << ",";
        json << zone.color[j];
      }
      json << "]";
      
      json << "}";
    }
    
    json << "]";
    json << "}";
    
    return json.str();
  }

  /**
   * @brief This function publishes images as Ros messages.
   *
   * @param img This is input image
   * @param encoding_type number of bits used to represent one pixel of image.
   * @param frame_id frame id of image
   * @param publisher This is ros publisher
   */
  void publishImageAsRosMsg(
    const cv::Mat & img, const std::string & encoding_type, const std::string & frame_id,
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher)
  {
    cv_bridge::CvImagePtr cv_ptr(new cv_bridge::CvImage);

    cv_ptr->encoding = encoding_type;
    cv_ptr->header.stamp = curr_frame_timestamp_;
    cv_ptr->header.frame_id = std::move(frame_id);
    cv_ptr->image = std::move(img);

    publisher->publish(*cv_ptr->toImageMsg());
  }
};

/**
 * @brief This is main function
 *
 * @param argc number of input arguments to the function
 * @param argv array of pointer to the input arguments
 * @return int
 */
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto adi_3dtof_safety_bubble_stitch = std::make_shared<ADI3DToFSafetyBubbleStitch>();

  rclcpp::spin(adi_3dtof_safety_bubble_stitch);
  rclcpp::shutdown();
  return 0;
}
