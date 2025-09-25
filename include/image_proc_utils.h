/******************************************************************************
Copyright (c), 2023 - Analog Devices Inc. All Rights Reserved.
This software is PROPRIETARY & CONFIDENTIAL to Analog Devices, Inc.
and its licensors.
******************************************************************************/

#ifndef IMAGE_PROC_UTILS_H
#define IMAGE_PROC_UTILS_H

#include <cv_bridge/cv_bridge.h>

#include <cmath>

#include "adi_camera.h"

#define QFORMAT_FOR_POINTCLOUD_LUT 14
#define QFORMAT_FOR_POINTCLOUD_LUT_SHIFTED_VALUE (1 << QFORMAT_FOR_POINTCLOUD_LUT)

/**
 * @brief Structure to hold image ROI info.
 *
 */
typedef struct __tADIImageROI
{
  int x;
  int y;
  int width;
  int height;
} ADIImageROI;

/**
 * @brief Structure to hold image info.
 *
 */
typedef struct __tADIImage
{
  void * data = NULL;
  int width = 0;
  int height = 0;
  int bpp = 0;               // bits per pixel
  ADIImageROI * roi = NULL;  // Set the pointer to null, to process full image
} ADIImage;

/**
 * @brief Structure to hold point.
 *
 */
typedef struct __tADI2DPoint
{
  short x;
  short y;
} ADI2DPoint;

/**
 * @brief This class has image processing utilities
 *
 */
class ImageProcUtils
{
public:
  // default constructor with no argument
  ImageProcUtils() = default;

  /**
   * @brief Construct a new Image Proc Utils object
   *
   * @param camera_intrinsics pointer to camera intrinsics
   * @param image_width width of the image
   * @param image_height height of the image
   */
  ImageProcUtils(CameraIntrinsics * camera_intrinsics, int image_width, int image_height)
  {
    image_width_ = image_width;
    image_height_ = image_height;

    // Generate LUT for range to depth correction
    range_to_xyz_lut_fixed_point_ = new short[image_width_ * image_height_ * 3];
    memset(range_to_xyz_lut_fixed_point_, 0, sizeof(short) * image_width_ * image_height_ * 3);

    generateRangeTo3DLUT(camera_intrinsics);
  }

  /**
   * @brief Destroy the Image Proc Utils object
   *
   */
  ~ImageProcUtils()
  {
    // Generate LUT for range to depth correction
    delete[] range_to_xyz_lut_fixed_point_;
  }
  static void convertTo8BppImage(ADIImage * in_img, ADIImage * out_img, int scale_factor);

  /**
   * @brief This function transforms image from one frame to another.
   * Source rotation and translation regarding mounting lens physically are not considered here.
   *
   * @param in_img Pointer to input image
   * @param out_img Poiunter to output image
   * @param src_intrinsics address of input camera intrinsics
   * @param src_extrinsics address of input camera extrinsics
   * @param dst_intrinsics address of output camera intrinsics
   * @param dst_extrinsics address of output camera extrinsics
   * @param out_roi Pointer to valid ROI strucure poopulated by the function
   * @param compute_point_cloud_enable_ enables point cloud computation
   * @param xyz_frame Pointer to point cloud
   *
   * @note
   * The function supports the images with 16bpp only.
   * The function doesn't support ROI processing.
   *
   */
  void transformFrame(
    ADIImage * in_img, ADIImage * out_img, CameraIntrinsics * src_intrinsics,
    CameraExtrinsics * src_extrinsics, CameraIntrinsics * dst_intrinsics,
    CameraExtrinsics * dst_extrinsics, ADIImageROI * out_roi, bool compute_point_cloud_enable_,
    short * xyz_frame);

  /**
   * @brief This function transforms image from one frame to another.
   * Source rotation and translation regarding mounting lens physically are not considered here.
   *
   * @param in_img Pointer to input image
   * @param out_img Poiunter to output image
   * @param src_intrinsics address of input camera intrinsics
   * @param src_extrinsics address of input camera extrinsics
   * @param dst_intrinsics address of output camera intrinsics
   * @param dst_extrinsics address of output camera extrinsics
   * @param out_roi Pointer to valid ROI strucure poopulated by the function
   * @param floor_height_mtr floor height
   * @param virtual_camera_height_mtr virtual camera height
   * @param xyz_frame Pointer to point cloud
   * @param compute_point_cloud_enable_ enables point cloud computation
   */
  void transformFrameWithFloorRemoval(
    ADIImage * in_img, ADIImage * out_img, CameraIntrinsics * src_intrinsics,
    CameraExtrinsics * src_extrinsics, CameraIntrinsics * dst_intrinsics,
    CameraExtrinsics * dst_extrinsics, ADIImageROI * out_roi, float floor_height_mtr,
    float virtual_camera_height_mtr, short * xyz_frame, bool compute_point_cloud_enable_);

  static void matrixMultiplication(
    float * input_matrix1, int rows1, int columns1, float * input_matrix2, int rows2, int columns2,
    float * output_matrix);

  static void matrixMultiplication3x3And3x1(
    float * input_matrix1, short * input_matrix2, short * output_matrix);

  static void rotatePointCloud(
    short * input_point_cloud, short * rotated_point_cloud, CameraExtrinsics * camera_extrinsics,
    int image_width, int image_height);

  /**
   * @brief This function returns minimum element of array
   *
   * @tparam T template to support any type to input array
   * @param input_array  This is input array
   * @param size size of the input array
   * @return T template to return any type.
   */
  template <typename T>
  static T minElement(T * input_array, int size)
  {
    int i;
    // Initialize minimum element
    T min = input_array[0];
    for (i = 1; i < size; i++) {
      if (input_array[i] < min) {
        min = input_array[i];
      }
    }
    return min;
  }

  void generateRangeTo3DLUT(CameraIntrinsics * camera_intrinsics);

  void computePointCloud(unsigned short * range_image, short * xyz_frame);

private:
  int image_width_;
  int image_height_;
  short * range_to_xyz_lut_fixed_point_;
};

#endif
