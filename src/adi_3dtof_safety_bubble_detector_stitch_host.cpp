/******************************************************************************
Copyright (c), 2023 - Analog Devices Inc. All Rights Reserved.
This software is PROPRIETARY & CONFIDENTIAL to Analog Devices, Inc.
and its licensors.
******************************************************************************/
#include "image_proc_utils.h"
#include <ros/ros.h>
#include <utility>
#include <std_msgs/Bool.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <eigen_conversions/eigen_msg.h>
#include <image_geometry/pinhole_camera_model.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <sensor_msgs/distortion_models.h>
#include <tf2/LinearMath/Quaternion.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/transforms.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

namespace enc = sensor_msgs::image_encodings;
#define MAX_QUEUE_SIZE_FOR_TIME_SYNC 2

// output image synchronizers.

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::CompressedImage, sensor_msgs::CompressedImage>
    sync_out_image_2;

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::CompressedImage, sensor_msgs::CompressedImage,
                                                        sensor_msgs::CompressedImage>
    sync_out_image_3;

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::CompressedImage, sensor_msgs::CompressedImage,
                                                        sensor_msgs::CompressedImage, sensor_msgs::CompressedImage>
    sync_out_image_4;

/**
 * @brief This is main class for the Stitch node
 *
 *
 */
class ADI3DToFSafetyBubbleStitch : public ros::NodeHandle
{
public:
  static const int MAX_NUM_DEVICES = 10;

  /**
   * @brief Construct a new ADI3DToFSafetyBubbleStitch object
   *
   * @param nh - Node handle
   * @param cam_prefix - vector of camera prefix for all the cameras
   * @param sync_out_image_2cam - Synchronizer handle for 2 camera output image
   * @param sync_out_image_3cam - Synchronizer handle for 3 camera output image
   * @param sync_out_image_4cam - Synchronizer handle for 4 camera output image
   */
  ADI3DToFSafetyBubbleStitch(ros::NodeHandle& nh, std::vector<std::string> cam_prefix,
                             message_filters::Synchronizer<sync_out_image_2>& sync_out_image_2cam,
                             message_filters::Synchronizer<sync_out_image_3>& sync_out_image_3cam,
                             message_filters::Synchronizer<sync_out_image_4>& sync_out_image_4cam)
  {
    ROS_INFO("adi_3dtof_safety_bubble_detector_stitch::Inside ADI3DToFSafetyBubbleStitch()");

    // Do not allow more than the max cameras supported.
    int max_devices_allowed = static_cast<int>(sizeof(out_image_subscriber_) / sizeof(out_image_subscriber_[0]));
    num_sensors_ = std::min(max_devices_allowed, static_cast<int>(cam_prefix.size()));
    for (int i = 0; i < num_sensors_; i++)
    {
      // Create subscribers.
      object_detected_subscriber_[i] = this->subscribe<std_msgs::Bool>(
          "/" + cam_prefix[i] + "/object_detected", 1,
          boost::bind(&ADI3DToFSafetyBubbleStitch::objectDetectedCallback, this, _1, i));

      // Synchronized subscribers
      out_image_subscriber_[i].subscribe(nh, "/" + cam_prefix[i] + "/out_image/compressed", 5);

      // Different flags
      out_image_recvd_[i] = false;
      out_image_[i] = nullptr;
    }

    // Add connections and register callback based on number of devices.
    if (num_sensors_ == 1)
    {
      single_sensor_out_image_subscriber_ = this->subscribe<sensor_msgs::CompressedImage>(
          "/" + cam_prefix[0] + "/out_image/compressed", 1,
          boost::bind(&ADI3DToFSafetyBubbleStitch::outImageCallback, this, _1, 0));
    }
    else if (num_sensors_ == 2)
    {
      sync_out_image_2cam.connectInput(out_image_subscriber_[0], out_image_subscriber_[1]);

      sync_out_image_2cam.registerCallback(
          boost::bind(&ADI3DToFSafetyBubbleStitch::sync2CamerasOutImageCallback, this, _1, _2));
    }
    else if (num_sensors_ == 3)
    {
      // 3 sensor configuration test once whether it works fine.
      sync_out_image_3cam.connectInput(out_image_subscriber_[0], out_image_subscriber_[1], out_image_subscriber_[2]);

      sync_out_image_3cam.registerCallback(
          boost::bind(&ADI3DToFSafetyBubbleStitch::sync3CamerasOutImageCallback, this, _1, _2, _3));
    }
    else if (num_sensors_ == 4)
    {
      sync_out_image_4cam.connectInput(out_image_subscriber_[0], out_image_subscriber_[1], out_image_subscriber_[2],
                                       out_image_subscriber_[3]);

      sync_out_image_4cam.registerCallback(
          boost::bind(&ADI3DToFSafetyBubbleStitch::sync4CamerasOutImageCallback, this, _1, _2, _3, _4));
    }

    // Create publishers.
    combo_out_image_publisher_ = this->advertise<sensor_msgs::Image>("combo_safety_bubble_out_image", 10);
    combo_object_detected_publisher_ = this->advertise<std_msgs::Bool>("combo_safety_bubble_object_detected", 10);

    // init
    frame_counter_ = 0;
  }

  /**
   * @brief This function shuts down all the nodes running in a roscore
   *
   */
  void shutDownAllNodes()
  {
    int status = system("rosnode kill -a");
    if (status < 0)
    {
      ROS_INFO_STREAM("Error in \"rosnode kill -a\": " << status);
    }
    ros::shutdown();
  }

  /**
   * @brief Destroy the ADI3DToFSafetyBubbleStitch object
   *
   */
  ~ADI3DToFSafetyBubbleStitch()
  {
    for (int i = 0; i < num_sensors_; i++)
    {
      if (out_image_[i] != nullptr)
      {
        delete[] out_image_[i];
        out_image_[i] = nullptr;
      }
    }
  }

  /**
   * @brief Call back for synchronised topics(Output image) for 3 camera configuration
   *
   * @param out_image_cam1 - Cam1 output image pointer
   * @param out_image_cam2 - Cam2 output image pointer
   */
  void sync2CamerasOutImageCallback(const sensor_msgs::CompressedImageConstPtr& out_image_cam1,
                                    const sensor_msgs::CompressedImageConstPtr& out_image_cam2)
  {
    // Call respective callbacks with the id.
    outImageCallback(out_image_cam1, 0);
    outImageCallback(out_image_cam2, 1);
  }

  /**
   * @brief Call back for synchronised topics(Output image) for 3 camera configuration
   *
   * @param out_image_cam1 - Cam1 output image pointer
   * @param out_image_cam2 - Cam2 output image pointer
   * @param out_image_cam3 - Cam3 output image pointer
   */
  void sync3CamerasOutImageCallback(const sensor_msgs::CompressedImageConstPtr& out_image_cam1,
                                    const sensor_msgs::CompressedImageConstPtr& out_image_cam2,
                                    const sensor_msgs::CompressedImageConstPtr& out_image_cam3)
  {
    // Call respective callbacks with the id.
    outImageCallback(out_image_cam1, 0);
    outImageCallback(out_image_cam2, 1);
    outImageCallback(out_image_cam3, 2);
  }

  /**
   * @brief Call back for synchronised topics(Output image) for 4 camera configuration
   *
   * @param compressed_out_image_cam1 - Cam1 output image pointer
   * @param compressed_out_image_cam2 - Cam2 output image pointer
   * @param compressed_out_image_cam3 - Cam3 output image pointer
   * @param compressed_out_image_cam4 - Cam4 output image pointer
   */
  void sync4CamerasOutImageCallback(const sensor_msgs::CompressedImageConstPtr& compressed_out_image_cam1,
                                    const sensor_msgs::CompressedImageConstPtr& compressed_out_image_cam2,
                                    const sensor_msgs::CompressedImageConstPtr& compressed_out_image_cam3,
                                    const sensor_msgs::CompressedImageConstPtr& compressed_out_image_cam4)
  {
    // Call respective callbacks with the id.
    outImageCallback(compressed_out_image_cam1, 0);
    outImageCallback(compressed_out_image_cam2, 1);
    outImageCallback(compressed_out_image_cam3, 2);
    outImageCallback(compressed_out_image_cam4, 3);
  }

  /**
   * @brief Low-level callback for ouput image
   *
   * @param message compressed image pointer
   * @param cam_id camera ID
   */
  void outImageCallback(const sensor_msgs::CompressedImageConstPtr& message, int cam_id)
  {
    int imdecode_flag = cv::IMREAD_COLOR;
    sensor_msgs::ImagePtr out_image;

    cv_bridge::CvImagePtr cv_ptr(new cv_bridge::CvImage);

    // Copy message header
    cv_ptr->header = message->header;
    cv_ptr->encoding = enc::BGR8;

    // Decode the color image
    cv_ptr->image = cv::imdecode(cv::Mat(message->data), imdecode_flag);

    size_t rows = cv_ptr->image.rows;
    size_t cols = cv_ptr->image.cols;

    if ((rows <= 0) || (cols <= 0))
    {
      return;
    }
    out_image = (cv_ptr->toImageMsg());

    image_width_ = out_image->width;
    image_height_ = out_image->height;
    if ((image_width_ != 512) && (image_height_ != 512))
    {
      return;
    }
    if (out_image_[cam_id] == nullptr)
    {
      out_image_[cam_id] = new unsigned char[image_width_ * image_height_ * 3];  //*3 assumed RGB format
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
  void objectDetectedCallback(const std_msgs::BoolConstPtr& object_detected, int cam_id)
  {
    if (object_detected == nullptr)
    {
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
    for (int i = 0; i < num_sensors_; i++)
    {
      if ((!out_image_recvd_[i]) || (!object_detected_recvd_[i]))
      {
        all_callbacks_recvd = false;
      }
    }

    if (all_callbacks_recvd)
    {
      ROS_INFO_STREAM("adi_3dtof_safety_bubble_detector_stitch_host::Running loop : " << frame_counter_);

      // Perform stitching : A simple ORing of the images and Flag
      cv::Mat m_stitched_out_image = cv::Mat(cv::Size(image_width_, image_height_), CV_8UC3, out_image_[0]);
      std_msgs::Bool combo_object_detected_msg;
      combo_object_detected_msg.data = object_detected_[0];

      // Combine
      for (int i = 1; i < num_sensors_; i++)
      {
        // Output image
        m_stitched_out_image += cv::Mat(cv::Size(image_width_, image_height_), CV_8UC3, out_image_[i]);

        // Object detection flag
        combo_object_detected_msg.data |= object_detected_[i];
      }

      // Draw Red/Green box to indicate presence of object in the Safety zone
      if (static_cast<bool>(combo_object_detected_msg.data))
      {
        cv::Rect box = cv::Rect(8, 10, 20, 20);
        cv::Scalar color = cv::Scalar(0, 0, 255);
        cv::rectangle(m_stitched_out_image, box, color, -1, 8, 0);
      }

      // Publish
      cv::Mat m_stitched_out_image_mirror;
      cv::flip(m_stitched_out_image, m_stitched_out_image_mirror, 1);
      combo_object_detected_publisher_.publish(combo_object_detected_msg);
      publishImageAsRosMsg(m_stitched_out_image_mirror, "bgr8", "stitched_image", combo_out_image_publisher_);

      // Reset flags
      for (int i = 0; i < num_sensors_; i++)
      {
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

  ros::Subscriber object_detected_subscriber_[MAX_NUM_DEVICES];
  ros::Subscriber single_sensor_out_image_subscriber_;
  message_filters::Subscriber<sensor_msgs::CompressedImage> out_image_subscriber_[MAX_NUM_DEVICES];
  bool object_detected_[MAX_NUM_DEVICES];
  bool out_image_recvd_[MAX_NUM_DEVICES];
  bool object_detected_recvd_[MAX_NUM_DEVICES];
  unsigned char* out_image_[MAX_NUM_DEVICES];
  ros::Publisher combo_out_image_publisher_;
  ros::Publisher combo_object_detected_publisher_;

  ros::Time curr_frame_timestamp_ = ros::Time::now();

  int num_sensors_;

  /**
   * @brief This function publishes images as Ros messages.
   *
   * @param img This is input image
   * @param encoding_type number of bits used to represent one pixel of image.
   * @param frame_id frame id of image
   * @param publisher This is ros publisher
   */
  void publishImageAsRosMsg(cv::Mat img, const std::string& encoding_type, std::string frame_id,
                            const ros::Publisher& publisher)
  {
    cv_bridge::CvImagePtr cv_ptr(new cv_bridge::CvImage);

    cv_ptr->encoding = encoding_type;
    cv_ptr->header.seq = frame_counter_;
    cv_ptr->header.stamp = curr_frame_timestamp_;
    cv_ptr->header.frame_id = std::move(frame_id);
    cv_ptr->image = std::move(img);

    publisher.publish(cv_ptr->toImageMsg());
  }
};

/**
 * @brief This is main function
 *
 * @param argc number of input arguments to the function
 * @param argv array of pointer to the input arguments
 * @return int
 */
int main(int argc, char** argv)
{
  ros::init(argc, argv, "adi_3dtof_safety_bubble_detector_stitch");
  ros::NodeHandle nh("~");

  // Get Parameters
  std::vector<std::string> cam_prefix;
  XmlRpc::XmlRpcValue cam_prefix_arr;
  nh.param("camera_prefixes", cam_prefix_arr, cam_prefix_arr);
  std::vector<std::string> cam_prefix_1;
  for (int i = 0; i < cam_prefix_arr.size(); i++)
  {
    cam_prefix.push_back(cam_prefix_arr[i]);
    std::cerr << "camera_prefixes: " << cam_prefix[i] << std::endl;
  }
  message_filters::Synchronizer<sync_out_image_2> sync_out_image_2cam(sync_out_image_2(MAX_QUEUE_SIZE_FOR_TIME_SYNC));
  message_filters::Synchronizer<sync_out_image_3> sync_out_image_3cam(sync_out_image_3(MAX_QUEUE_SIZE_FOR_TIME_SYNC));
  message_filters::Synchronizer<sync_out_image_4> sync_out_image_4cam(sync_out_image_4(MAX_QUEUE_SIZE_FOR_TIME_SYNC));

  try
  {
    // Create an instance of ADI3DToFSafetyBubbleStitch
    ADI3DToFSafetyBubbleStitch adi_3dtof_safety_bubble_detector_stitch(nh, cam_prefix, sync_out_image_2cam,
                                                                       sync_out_image_3cam, sync_out_image_4cam);

    ros::Rate loop_rate(30);

    while (ros::ok())
    {
      /* Perform stitching. */
      if (!adi_3dtof_safety_bubble_detector_stitch.stitchFrames())
      {
        adi_3dtof_safety_bubble_detector_stitch.shutDownAllNodes();
      }

      loop_rate.sleep();

      ros::spinOnce();
    }

    ros::shutdown();
  }
  catch (const std::exception& e)
  {
    std::cerr << " Exception in constructor of adi_3dtof_safety_bubble_detector_stitch : " << e.what() << '\n';
  }

  return 0;
}
