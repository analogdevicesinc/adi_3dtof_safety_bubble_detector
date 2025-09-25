/******************************************************************************
Copyright (c), 2023 - Analog Devices Inc. All Rights Reserved.
This software is PROPRIETARY & CONFIDENTIAL to Analog Devices, Inc.
and its licensors.
******************************************************************************/
#ifndef ADI_3DTOF_SAFETY_BUBBLE_DETECTOR_OUTPUT_INFO_H
#define ADI_3DTOF_SAFETY_BUBBLE_DETECTOR_OUTPUT_INFO_H

#include <cstring>
/**
 * @brief This is the class for Safety bubble detector output info
 *
 */
class ADI3DToFSafetyBubbleDetectorOutputInfo
{
public:
  /**
   * @brief Constructor
   *
   * @param image_width - Image Width
   * @param image_height - Image Height
   */
  ADI3DToFSafetyBubbleDetectorOutputInfo(int image_width, int image_height)
  {
    // Create the node.
    frame_number_ = -1;
    object_detected_ = false;
    ransac_floor_detection_status_ = false;
    ransac_iterations_ = -1;
    ransac_time_ms_ = -1;
    noise_count_ = -1;
    depth_frame_ = nullptr;
    ab_frame_ = nullptr;
    xyz_frame_ = nullptr;
    compressed_ab_frame_ = nullptr;
    compressed_depth_frame_ = nullptr;
    depth_frame_with_floor_ = nullptr;
    vcam_depth_frame_ = nullptr;
    vcam_depth_frame_with_floor_ = nullptr;
    vcam_depth_image_floor_pixels_removed_8bpp_ = nullptr;
    image_width_ = image_width;
    image_height_ = image_height;
    compressed_depth_frame_size_ = 0;
    compressed_ab_frame_size_ = 0;

    depth_frame_ = new unsigned short[image_width * image_height];
    ab_frame_ = new unsigned short[image_width * image_height];
    xyz_frame_ = new short[3 * image_width * image_height];
    depth_frame_with_floor_ = new unsigned short[image_width * image_height];
    vcam_depth_frame_ = new unsigned short[image_width * image_height];
    vcam_depth_frame_with_floor_ = new unsigned short[image_width * image_height];
    vcam_depth_image_floor_pixels_removed_8bpp_ = new unsigned char[image_width * image_height];
    compressed_depth_frame_ = new unsigned char[2 * image_width * image_height];
    compressed_ab_frame_ = new unsigned char[2 * image_width * image_height];
  }

  /**
   * @brief Destructor
   */
  ~ADI3DToFSafetyBubbleDetectorOutputInfo()
  {
    if (depth_frame_ != nullptr) {
      delete[] depth_frame_;
    }
    if (ab_frame_ != nullptr) {
      delete[] ab_frame_;
    }
    if (xyz_frame_ != nullptr) {
      delete[] xyz_frame_;
    }
    if (vcam_depth_frame_ != nullptr) {
      delete[] vcam_depth_frame_;
    }
    if (vcam_depth_image_floor_pixels_removed_8bpp_ != nullptr) {
      delete[] vcam_depth_image_floor_pixels_removed_8bpp_;
    }
    if (vcam_depth_frame_with_floor_ != nullptr) {
      delete[] vcam_depth_frame_with_floor_;
    }
    if (depth_frame_with_floor_ != nullptr) {
      delete[] depth_frame_with_floor_;
    }
    if (compressed_depth_frame_ != nullptr) {
      delete[] compressed_depth_frame_;
    }
    if (compressed_ab_frame_ != nullptr) {
      delete[] compressed_ab_frame_;
    }
  }

  // Assignment operator
  ADI3DToFSafetyBubbleDetectorOutputInfo & operator=(
    const ADI3DToFSafetyBubbleDetectorOutputInfo & rhs)
  {
    frame_number_ = rhs.frame_number_;
    object_detected_ = rhs.object_detected_;
    ransac_floor_detection_status_ = rhs.ransac_floor_detection_status_;
    ransac_iterations_ = rhs.ransac_iterations_;
    noise_count_ = rhs.noise_count_;
    ransac_time_ms_ = rhs.ransac_time_ms_;
    memcpy(depth_frame_, rhs.depth_frame_, sizeof(depth_frame_) * image_width_ * image_height_);
    memcpy(ab_frame_, rhs.ab_frame_, sizeof(ab_frame_) * image_width_ * image_height_);
    compressed_depth_frame_size_ = rhs.compressed_depth_frame_size_;
    compressed_ab_frame_size_ = rhs.compressed_ab_frame_size_;
    memcpy(
      compressed_depth_frame_, rhs.compressed_depth_frame_,
      sizeof(compressed_depth_frame_) * compressed_depth_frame_size_);
    memcpy(
      compressed_ab_frame_, rhs.compressed_ab_frame_,
      sizeof(compressed_ab_frame_) * compressed_ab_frame_size_);
    memcpy(xyz_frame_, rhs.xyz_frame_, sizeof(xyz_frame_) * image_width_ * image_height_ * 3);
    memcpy(
      depth_frame_with_floor_, rhs.depth_frame_with_floor_,
      sizeof(depth_frame_with_floor_) * image_width_ * image_height_);
    memcpy(
      vcam_depth_frame_, rhs.vcam_depth_frame_,
      sizeof(vcam_depth_frame_) * image_width_ * image_height_);
    memcpy(
      vcam_depth_frame_with_floor_, rhs.vcam_depth_frame_with_floor_,
      sizeof(vcam_depth_frame_with_floor_) * image_width_ * image_height_);
    memcpy(
      vcam_depth_image_floor_pixels_removed_8bpp_, rhs.vcam_depth_image_floor_pixels_removed_8bpp_,
      sizeof(vcam_depth_image_floor_pixels_removed_8bpp_) * image_width_ * image_height_);
    image_width_ = rhs.image_width_;
    image_height_ = rhs.image_height_;

    return *this;
  }

  int frame_number_;
  bool object_detected_;
  bool ransac_floor_detection_status_;
  int ransac_iterations_;
  int noise_count_;
  float ransac_time_ms_;
  unsigned short * depth_frame_;
  unsigned short * ab_frame_;
  int compressed_depth_frame_size_;
  int compressed_ab_frame_size_;
  unsigned char * compressed_depth_frame_;
  unsigned char * compressed_ab_frame_;
  short * xyz_frame_;
  unsigned short * depth_frame_with_floor_;
  unsigned short * vcam_depth_frame_;
  unsigned short * vcam_depth_frame_with_floor_;
  unsigned char * vcam_depth_image_floor_pixels_removed_8bpp_;
  int image_width_;
  int image_height_;
};

#endif
