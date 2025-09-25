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
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <utility>

#include "image_proc_utils.h"

using namespace std::chrono_literals;
namespace enc = sensor_msgs::image_encodings;
#define MAX_QUEUE_SIZE_FOR_TIME_SYNC 5

/**
 * @brief This is main class for the Stitch node
 *
 *
 */
class ADI3DToFSafetyBubbleStitch : public rclcpp::Node
{
public:
  static const int MAX_NUM_DEVICES = 10;

  /**
   * @brief Construct a new ADI3DToFSafetyBubbleStitch object
   *
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
    RCLCPP_INFO(
      this->get_logger(),
      "adi_3dtof_safety_bubble_detector_stitch::Inside ADI3DToFSafetyBubbleStitch()");

    this->declare_parameter("param_camera_prefixes", rclcpp::PARAMETER_STRING_ARRAY);

    // Getting Camera prefixes
    rclcpp::Parameter string_array_param = this->get_parameter("param_camera_prefixes");
    camera_prefixes_ = string_array_param.as_string_array();
    for (int i = 0; i < (int)camera_prefixes_.size(); i++) {
      std::cerr << "camera_prefixes: " << camera_prefixes_[i] << std::endl;
    }

    // Do not allow more than the max cameras supported.
    int max_devices_allowed =
      static_cast<int>(sizeof(out_image_subscriber_) / sizeof(out_image_subscriber_[0]));
    num_sensors_ = std::min(max_devices_allowed, static_cast<int>(camera_prefixes_.size()));

    for (int i = 0; i < num_sensors_; i++) {
      // Create subscribers.
      std::function<void(const std_msgs::msg::Bool::SharedPtr object_detected_flag)>
        object_detected_callback_func = std::bind(
          &ADI3DToFSafetyBubbleStitch::objectDetectedCallback, this, std::placeholders::_1, i);

      object_detected_subscriber_[i] = this->create_subscription<std_msgs::msg::Bool>(
        "/" + camera_prefixes_[i] + "/object_detected", 1, object_detected_callback_func);

      // Synchronized subscribers
      out_image_subscriber_[i].subscribe(this, "/" + camera_prefixes_[i] + "/out_image");
      compressed_out_image_subscriber_[i].subscribe(
        this, "/" + camera_prefixes_[i] + "/out_image/compressed");

      // Flags related to output images
      out_image_recvd_[i] = false;
      out_image_[i] = nullptr;
    }

    // Add connections and register callback based on number of devices.
    if (num_sensors_ == 1) {
      std::function<void(
        const sensor_msgs::msg::CompressedImage::SharedPtr compressed_image_callback)>
        compressed_out_image_callback_func = std::bind(
          &ADI3DToFSafetyBubbleStitch::compressedOutImageCallback, this, std::placeholders::_1, 0);
      single_compressed_out_image_subscriber_ =
        this->create_subscription<sensor_msgs::msg::CompressedImage>(
          "/" + camera_prefixes_[0] + "/out_image/compressed", 1,
          compressed_out_image_callback_func);
      std::function<void(const sensor_msgs::msg::Image::SharedPtr image_callback)>
        image_callback_func =
          std::bind(&ADI3DToFSafetyBubbleStitch::outImageCallback, this, std::placeholders::_1, 0);
      single_out_image_subscriber_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/" + camera_prefixes_[0] + "/out_image", 1, image_callback_func);
    } else if (num_sensors_ == 2) {
      synchronizer_for_2_images.registerCallback(
        &ADI3DToFSafetyBubbleStitch::sync2CamerasOutImageCallback, this);
      synchronizer_for_2_compressed_images.registerCallback(
        &ADI3DToFSafetyBubbleStitch::sync2CamerasCompressedOutImageCallback, this);
    } else if (num_sensors_ == 3) {
      synchronizer_for_3_images.registerCallback(
        &ADI3DToFSafetyBubbleStitch::sync3CamerasOutImageCallback, this);
      synchronizer_for_3_compressed_images.registerCallback(
        &ADI3DToFSafetyBubbleStitch::sync3CamerasCompressedOutImageCallback, this);
    } else if (num_sensors_ == 4) {
      synchronizer_for_4_images.registerCallback(
        &ADI3DToFSafetyBubbleStitch::sync4CamerasOutImageCallback, this);
      synchronizer_for_4_compressed_images.registerCallback(
        &ADI3DToFSafetyBubbleStitch::sync4CamerasCompressedOutImageCallback, this);
    }

    // Create publishers.
    combo_out_image_publisher_ =
      this->create_publisher<sensor_msgs::msg::Image>("combo_safety_bubble_out_image", 10);
    combo_object_detected_publisher_ =
      this->create_publisher<std_msgs::msg::Bool>("combo_safety_bubble_object_detected", 10);

    timer_ =
      this->create_wall_timer(33ms, std::bind(&ADI3DToFSafetyBubbleStitch::timerCallback, this));

    // init
    frame_counter_ = 0;
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
   * @brief Low level callback for Object detection flag
   *
   * @param object_detected - Object detected message pointer
   * @param cam_id - Camera Id
   */
  void objectDetectedCallback(
    const std_msgs::msg::Bool::ConstSharedPtr & object_detected, int cam_id)
  {
    if (object_detected == nullptr) {
      return;
    }
    object_detected_[cam_id] = object_detected->data;

    // Set flag
    object_detected_recvd_[cam_id] = true;
  }

  /**
   * @brief Stitch function: stitches all the output images, and the point cloud.
   *
   * @return true if the image is stitched.
   * @return false if the image is not stitched as all the callbacks are not received.
   */
  bool stitchFrames()
  {
    // Make sure we have received frames from all the sensors
    bool all_callbacks_recvd = true;
    for (int i = 0; i < num_sensors_; i++) {
      if ((!out_image_recvd_[i]) || (!object_detected_recvd_[i])) {
        all_callbacks_recvd = false;
      }
    }

    if (all_callbacks_recvd) {
      RCLCPP_INFO_STREAM(
        this->get_logger(),
        "adi_3dtof_safety_bubble_detector_stitch_host::Running loop : " << frame_counter_);

      // Perform stitching : A simple ORing of the images and Flag
      cv::Mat m_stitched_out_image =
        cv::Mat(cv::Size(image_width_, image_height_), CV_8UC3, out_image_[0]);
      std_msgs::msg::Bool combo_object_detected_msg;
      combo_object_detected_msg.data = object_detected_[0];

      // Combine
      for (int i = 1; i < num_sensors_; i++) {
        // Output image
        m_stitched_out_image +=
          cv::Mat(cv::Size(image_width_, image_height_), CV_8UC3, out_image_[i]);

        // Object detection flag
        combo_object_detected_msg.data |= object_detected_[i];
      }

      // Draw Red/Green box to indicate presence of object in the Safety zone
      if (static_cast<bool>(combo_object_detected_msg.data)) {
        cv::Rect box = cv::Rect(8, 10, 20, 20);
        cv::Scalar color = cv::Scalar(0, 0, 255);
        cv::rectangle(m_stitched_out_image, box, color, -1, 8, 0);
      }

      // Publish
      cv::Mat m_stitched_out_image_mirror;
      cv::flip(m_stitched_out_image, m_stitched_out_image_mirror, 1);
      combo_object_detected_publisher_->publish(combo_object_detected_msg);
      publishImageAsRosMsg(
        m_stitched_out_image_mirror, "bgr8", "stitched_image", combo_out_image_publisher_);

      // Reset flags
      for (int i = 0; i < num_sensors_; i++) {
        out_image_recvd_[i] = false;
        object_detected_recvd_[i] = false;
      }

      // Update frame count
      frame_counter_++;
    }

    return true;
  }

private:
  int image_width_ = 512;
  int image_height_ = 512;
  int frame_counter_;
  std::vector<std::string> camera_prefixes_;

  rclcpp::TimerBase::SharedPtr timer_;
  // rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr camera_info_subscriber_;

  // ros subscribers
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr object_detected_subscriber_[MAX_NUM_DEVICES];
  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr
    single_compressed_out_image_subscriber_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr single_out_image_subscriber_;
  message_filters::Subscriber<sensor_msgs::msg::CompressedImage>
    compressed_out_image_subscriber_[MAX_NUM_DEVICES];
  message_filters::Subscriber<sensor_msgs::msg::Image> out_image_subscriber_[MAX_NUM_DEVICES];

  bool object_detected_[MAX_NUM_DEVICES];
  bool out_image_recvd_[MAX_NUM_DEVICES];
  bool object_detected_recvd_[MAX_NUM_DEVICES];
  unsigned char * out_image_[MAX_NUM_DEVICES];

  // ros publishers
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr combo_out_image_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr
    combo_compressed_out_image_publisher_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr combo_object_detected_publisher_;

  // Output image synchronizers
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

  int num_sensors_;

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
