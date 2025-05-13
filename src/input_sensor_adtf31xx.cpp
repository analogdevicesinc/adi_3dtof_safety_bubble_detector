/******************************************************************************
Copyright (c), 2023 - Analog Devices Inc. All Rights Reserved.
This software is PROPRIETARY & CONFIDENTIAL to Analog Devices, Inc.
and its licensors.
******************************************************************************/

#include "aditof/camera.h"
#include "aditof/frame.h"
#include "aditof/system.h"
#include "aditof/version.h"
#include "aditof/sensor_definitions.h"
#include "aditof/depth_sensor_interface.h"
//#include <aditof/ccb.h>
#include "input_sensor_adtf31xx.h"
#include <ros/ros.h>
#include <fstream>

using aditof::Status;

/**
 * @brief Opens adtf31xx sensor
 *
 * @param sensor_name This parameter is not used in this derived member function.
 * @param input_image_width width of the image
 * @param input_image_height height of the image
 * @param processing_scale scale factor for image and camera intrinsics
 * @param config_file_name path of configuration json file for ToF SDK.
 */
void InputSensorADTF31xx::openSensor(std::string /*sensor_name*/, int input_image_width, int input_image_height,
                                     std::string config_file_name, std::string input_sensor_ip)
{
  sensor_open_flag_ = false;

  // realtime mode
  Status status = Status::OK;
  aditof::System system;

  std::vector<std::shared_ptr<aditof::Camera>> cameras;

  if (!input_sensor_ip.empty())
  {
    std::string ip = "ip:" + input_sensor_ip;
    system.getCameraList(cameras, ip);
  }
  else
  {
    system.getCameraList(cameras);
  }

  if (cameras.empty())
  {
    ROS_ERROR("No cameras found");
    return;
  }

  camera_ = cameras.front();

  status = camera_->initialize(config_file_name);
  if (status != Status::OK)
  {
    ROS_ERROR("Could not initialize camera!");
    return;
  }

  sensor_open_flag_ = true;

  frame_width_ = input_image_width;
  frame_height_ = input_image_height;

  // Clear camera parameters.
  memset(&camera_intrinsics_, 0, sizeof(camera_intrinsics_));

  return;
}

/**
 * @brief Configures the camera
 *
 * @param camera_mode camera_mode
 */
void InputSensorADTF31xx::configureSensor(int camera_mode)
{
  Status status = Status::OK;
  aditof::System system;

  std::vector<uint8_t> available_modes;
  camera_->getAvailableModes(available_modes);
  if (available_modes.empty())
  {
    ROS_ERROR("no frame type avaialble!");
    return;
  }

  aditof::CameraDetails camera_details;
  camera_->getDetails(camera_details);

  std::cout << std::endl << "Camera Parameters:" << std::endl;
  std::cout << "Cx, Cy : " << camera_details.intrinsics.cx << ", " << camera_details.intrinsics.cy << std::endl;
  std::cout << "Fx, Fy : " << camera_details.intrinsics.fx << ", " << camera_details.intrinsics.fy << std::endl;
  std::cout << "K1, K2 : " << camera_details.intrinsics.k1 << ", " << camera_details.intrinsics.k2 << std::endl;
  std::cout << "K3, K4 : " << camera_details.intrinsics.k3 << ", " << camera_details.intrinsics.k4 << std::endl;
  std::cout << "K5, K6 : " << camera_details.intrinsics.k5 << ", " << camera_details.intrinsics.k6 << std::endl;
  std::cout << "P1, P2 : " << camera_details.intrinsics.p1 << ", " << camera_details.intrinsics.p2 << std::endl;

  // Camera matrix = [fx, 0, cx, 0, fy, cy, 0, 0, 1]
  camera_intrinsics_.camera_matrix[0] = camera_details.intrinsics.fx;
  camera_intrinsics_.camera_matrix[1] = 0;
  camera_intrinsics_.camera_matrix[2] = camera_details.intrinsics.cx;
  camera_intrinsics_.camera_matrix[3] = 0;
  camera_intrinsics_.camera_matrix[4] = camera_details.intrinsics.fy;
  camera_intrinsics_.camera_matrix[5] = camera_details.intrinsics.cy;
  camera_intrinsics_.camera_matrix[6] = 0;
  camera_intrinsics_.camera_matrix[7] = 0;
  camera_intrinsics_.camera_matrix[8] = 1;

  // Distortion vector = [k1, k2, p1, p2, k3, k4, k5, k6]
  camera_intrinsics_.distortion_coeffs[0] = camera_details.intrinsics.k1;
  camera_intrinsics_.distortion_coeffs[1] = camera_details.intrinsics.k2;

  camera_intrinsics_.distortion_coeffs[2] = camera_details.intrinsics.p1;
  camera_intrinsics_.distortion_coeffs[3] = camera_details.intrinsics.p2;

  camera_intrinsics_.distortion_coeffs[4] = camera_details.intrinsics.k3;
  camera_intrinsics_.distortion_coeffs[5] = camera_details.intrinsics.k4;
  camera_intrinsics_.distortion_coeffs[6] = camera_details.intrinsics.k5;
  camera_intrinsics_.distortion_coeffs[7] = camera_details.intrinsics.k6;

  status = camera_->setMode(camera_mode);
  if (status != Status::OK)
  {
    ROS_ERROR("Could not set camera mode!");
    return;
  }

  aditof::DepthSensorModeDetails depth_sensor_details;
  std::shared_ptr<aditof::DepthSensorInterface> sensor = camera_->getSensor();

  aditof::Status st = sensor->getModeDetails(camera_mode, depth_sensor_details);
  if (st != aditof::Status::OK)
  {
    ROS_ERROR("Failed to get frame type details!");
    return;
  }

  setFrameWidth(depth_sensor_details.baseResolutionWidth);
  setFrameHeight(depth_sensor_details.baseResolutionHeight);

  // Set the scale factor
  if((depth_sensor_details.baseResolutionWidth == 1024 && depth_sensor_details.baseResolutionHeight == 1024) ||
      (depth_sensor_details.baseResolutionWidth == 512 && depth_sensor_details.baseResolutionHeight == 640))
  {
    setProcessingScale(1);
  }
  else
  {
    setProcessingScale(2);
  }

  status = camera_->start();
  if (status != Status::OK)
  {
    ROS_ERROR("Could not start the camera!");
    return;
  }

  return;
}

/**
 * @brief gets the camera intrinsics from the ToF SDK.
 *
 * @param camera_intrinsics camera intrinsics of ToF module.
 */
void InputSensorADTF31xx::getIntrinsics(CameraIntrinsics* camera_intrinsics)
{
  *camera_intrinsics = camera_intrinsics_;

  // Scale
  camera_intrinsics->camera_matrix[0] /= input_scale_factor_;
  camera_intrinsics->camera_matrix[2] /= input_scale_factor_;
  camera_intrinsics->camera_matrix[4] /= input_scale_factor_;
  camera_intrinsics->camera_matrix[5] /= input_scale_factor_;

  return;
}

/**
 * @brief reads frame from ToF SDK
 *
 * @param depth_frame pointer to get depth frame
 * @param ab_frame pointer to get ir frame
 * @return true if read is successful
 * @return false if read is not successful
 */

bool InputSensorADTF31xx::readNextFrame(unsigned short* depth_frame, unsigned short* ab_frame)
{
  aditof::Frame frame;

  Status status = camera_->requestFrame(&frame);
  if (status != Status::OK)
  {
    ROS_ERROR("Could not request frame!");
    return false;
  }

  // Depth
  uint16_t* depth_frame_src;
  status = frame.getData("depth", &depth_frame_src);
  if (status != Status::OK)
  {
    ROS_ERROR("Could not get depth data!");
    return false;
  }
  int frame_width = frame_width_;
  int frame_height = frame_height_;

  // Copy Depth
  memcpy(depth_frame, depth_frame_src, frame_width * frame_height * bytes_per_pixel_);

  // IR
  unsigned short* ab_frame_src;
  status = frame.getData("ab", &ab_frame_src);
  if (status != Status::OK)
  {
    ROS_ERROR("Could not get ir data!");
    return false;
  }
  memcpy(ab_frame, ab_frame_src, frame_width * frame_height * bytes_per_pixel_);

  // Confidence image
  // unsigned char* confidence_img = (unsigned char*)&ab_frame_src[frame_width * frame_height];
  // memcpy(conf_frame, confidence_img, frame_width * frame_height);

  return true;
}

/**
 * @brief gets the frame timestamp from ToF SDK
 *
 */

bool InputSensorADTF31xx::getFrameTimestamp(ros::Time* timestamp)
{
  *timestamp = ros::Time::now();
  return true;
}

/**
 * @brief Gets camera extrinsics
 *
 * @param camera_extrinsics Camera extrinsics
 */

void InputSensorADTF31xx::getExtrinsics(CameraExtrinsics* camera_extrinsics)
{
  camera_extrinsics->rotation_matrix[0] = 1.0f;
  camera_extrinsics->rotation_matrix[1] = 0.0f;
  camera_extrinsics->rotation_matrix[2] = 0.0f;
  camera_extrinsics->rotation_matrix[3] = 0.0f;
  camera_extrinsics->rotation_matrix[4] = 1.0f;
  camera_extrinsics->rotation_matrix[5] = 0.0f;
  camera_extrinsics->rotation_matrix[6] = 0.0f;
  camera_extrinsics->rotation_matrix[7] = 0.0f;
  camera_extrinsics->rotation_matrix[8] = 1.0f;

  camera_extrinsics->translation_matrix[0] = 0.0f;
  camera_extrinsics->translation_matrix[1] = 0.0f;
  camera_extrinsics->translation_matrix[2] = 0.0f;

  return;
}

/**
 * @brief closes the sensor
 *
 */
void InputSensorADTF31xx::closeSensor()
{
  ROS_INFO("Stopping the Camera");
  camera_->stop();
  return;
}

/**
 * @brief calls set ab invalidation threshold function from ToF SDK
 *
 * @param threshold abinvalidation threshold
 */
void InputSensorADTF31xx::setABinvalidationThreshold(int threshold)
{
  camera_->adsd3500SetABinvalidationThreshold(threshold);
  return;
}

/**
 * @brief calls set confidence threshold function from ToF SDK
 *
 * @param threshold confidence threshold
 */
void InputSensorADTF31xx::setConfidenceThreshold(int threshold)
{
  camera_->adsd3500SetConfidenceThreshold(threshold);
  return;
}
