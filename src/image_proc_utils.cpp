/******************************************************************************
Copyright (c), 2023 - Analog Devices Inc. All Rights Reserved.
This software is PROPRIETARY & CONFIDENTIAL to Analog Devices, Inc.
and its licensors.
******************************************************************************/

#include "image_proc_utils.h"
#include <string.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/calib3d.hpp>

/**
 * @brief This function input image to 8 bits per pixel image
 *
 * @param in_img Ponter to input image
 * @param out_img Pointer to output image
 * @param scale_factor This is scale factor
 *
 * @note
 * 1. The dimensions of inoput and output images are assumed to be same.
 * 2. In case of ROI processing, the ROI part in the output image is populated.
 *
 */
void ImageProcUtils::convertTo8BppImage(ADIImage* in_img, ADIImage* out_img, int scale_factor)
{
  if ((in_img != nullptr) && (out_img != nullptr))
  {
    if ((in_img->data != nullptr) && (out_img->data != nullptr) && (in_img->bpp == 16) && (out_img->bpp == 8))
    {
      unsigned short* src_buf = (unsigned short*)in_img->data;
      unsigned char* dst_buf = (unsigned char*)out_img->data;

      int roi_min_x = 0;
      int roi_min_y = 0;
      int roi_width = in_img->width;
      int roi_height = in_img->height;

      if (in_img->roi != nullptr)
      {
        roi_min_x = in_img->roi->x;
        roi_min_y = in_img->roi->y;
        roi_width = in_img->roi->width;
        roi_height = in_img->roi->height;
      }

      // Move to ROI start
      int start_idx = (roi_min_y * in_img->width) + roi_min_x;
      src_buf += start_idx;
      dst_buf += start_idx;

      float inv_scale_factor = 1.0f / scale_factor;
      for (int i = 0; i < roi_height; i++)
      {
        for (int j = 0; j < roi_width; j++)
        {
          *dst_buf++ = static_cast<unsigned char>((*src_buf++ * inv_scale_factor) * 255);
        }
        // Goto next line
        src_buf += (in_img->width - roi_width);
        dst_buf += (in_img->width - roi_width);
      }
    }
  }
}

void ImageProcUtils::transformFrame(ADIImage* in_img, ADIImage* out_img, CameraIntrinsics* /*src_intrinsics*/,
                                    CameraExtrinsics* /*src_extrinsics*/, CameraIntrinsics* dst_intrinsics,
                                    CameraExtrinsics* dst_extrinsics, ADIImageROI* out_roi,
                                    bool /*compute_point_cloud_enable*/, short* xyz_frame)
{
  if ((in_img != nullptr) && (out_img != nullptr) && (out_roi != nullptr))
  {
    if ((in_img->data != nullptr) && (out_img->data != nullptr))
    {
      unsigned short* src_img = (unsigned short*)in_img->data;
      unsigned short* dst_img = (unsigned short*)out_img->data;
      unsigned short* src_img_runner = src_img;
      int img_width = in_img->width;
      int img_height = in_img->height;
      int min_x = img_width;
      int min_y = img_height;
      int max_x = 0;
      int max_y = 0;
      short* xyz_frame_org = xyz_frame;
      float* rot_matrix = dst_extrinsics->rotation_matrix;
      float* trans_matrix = dst_extrinsics->translation_matrix;
      float* cam_matrix = dst_intrinsics->camera_matrix;

      for (int i = 0; i < img_height; i++)
      {
        for (int j = 0; j < img_width; j++)
        {
          unsigned short src_val = *src_img_runner++;

          if (src_val == 0)
          {
            xyz_frame += 3;
            continue;
          }

          /*Get 3D src camera coordinates*/
          float src_xc_mm = *xyz_frame++;
          float src_yc_mm = *xyz_frame++;
          float src_zc_mm = *xyz_frame++;

          float dstc_coord[3];
          dstc_coord[0] =
              (rot_matrix[0] * src_xc_mm + rot_matrix[1] * src_yc_mm + rot_matrix[2] * src_zc_mm) + trans_matrix[0];
          dstc_coord[1] =
              (rot_matrix[3] * src_xc_mm + rot_matrix[4] * src_yc_mm + rot_matrix[5] * src_zc_mm) + trans_matrix[1];
          dstc_coord[2] =
              (rot_matrix[6] * src_xc_mm + rot_matrix[7] * src_yc_mm + rot_matrix[8] * src_zc_mm) + trans_matrix[2];

          /*3d dst camera coordinates to 2d dst image coordinates conversion*/
          int dst_x1 =
              (int)(((dstc_coord[0] * cam_matrix[0] + trans_matrix[0]) / dstc_coord[2]) + cam_matrix[2] + 0.5f);
          int dst_y1 =
              (int)(((dstc_coord[1] * cam_matrix[4] + trans_matrix[1]) / dstc_coord[2]) + cam_matrix[5] + 0.5f);

          if (dst_x1 < 0 || dst_x1 >= img_width || dst_y1 < 0 || dst_y1 >= img_height)
          {
            continue;
          }

          unsigned short dst_depth = dst_img[dst_y1 * img_width + dst_x1];
          unsigned short new_depth = (unsigned short)(dstc_coord[2]);
          if ((dst_depth == 0) || (dst_depth > new_depth))
          {
            dst_img[dst_y1 * img_width + dst_x1] = new_depth;

            // Update valid ROI limits
            min_x = std::min(min_x, dst_x1);
            min_y = std::min(min_y, dst_y1);
            max_x = std::max(max_x, dst_x1);
            max_y = std::max(max_y, dst_y1);
          }
        }
      }

      // Update ROI
      out_roi->x = std::max(0, min_x);
      out_roi->y = std::max(0, min_y);
      out_roi->width = std::min(std::max(0, (max_x - out_roi->x) + 1), img_width);
      out_roi->height = std::min(std::max(0, (max_y - out_roi->y) + 1), img_width);
    }
  }
}

void ImageProcUtils::transformFrameWithFloorRemoval(
    ADIImage* in_img, ADIImage* out_img, CameraIntrinsics* /*src_intrinsics*/, CameraExtrinsics* /*src_extrinsics*/,
    CameraIntrinsics* dst_intrinsics, CameraExtrinsics* dst_extrinsics, ADIImageROI* out_roi, float floor_height_mtr,
    float virtual_camera_height_mtr, short* xyz_frame, bool /*compute_point_cloud_enable*/)
{
  float effective_floor_height_mm = (virtual_camera_height_mtr - floor_height_mtr) * 1000.0f;

  if ((in_img != nullptr) && (out_img != nullptr) && (out_roi != nullptr))
  {
    if ((in_img->data != nullptr) && (out_img->data != nullptr))
    {
      unsigned short* src_img = (unsigned short*)in_img->data;
      unsigned short* dst_img = (unsigned short*)out_img->data;
      unsigned short* src_img_runner = src_img;
      int img_width = in_img->width;
      int img_height = in_img->height;
      int min_x = img_width;
      int min_y = img_height;
      int max_x = 0;
      int max_y = 0;
      short* xyz_frame_org = xyz_frame;
      float* rot_matrix = dst_extrinsics->rotation_matrix;
      float* trans_matrix = dst_extrinsics->translation_matrix;
      float* cam_matrix = dst_intrinsics->camera_matrix;

      for (int i = 0; i < img_height; i++)
      {
        for (int j = 0; j < img_width; j++)
        {
          unsigned short src_val = *src_img_runner++;

          if (src_val == 0)
          {
            xyz_frame += 3;
            continue;
          }

          /*Get 3D src camera coordinates*/
          float src_xc_mm = *xyz_frame++;
          float src_yc_mm = *xyz_frame++;
          float src_zc_mm = *xyz_frame++;

          float dstc_coord[3];
          dstc_coord[0] =
              (rot_matrix[0] * src_xc_mm + rot_matrix[1] * src_yc_mm + rot_matrix[2] * src_zc_mm) + trans_matrix[0];
          dstc_coord[1] =
              (rot_matrix[3] * src_xc_mm + rot_matrix[4] * src_yc_mm + rot_matrix[5] * src_zc_mm) + trans_matrix[1];
          dstc_coord[2] =
              (rot_matrix[6] * src_xc_mm + rot_matrix[7] * src_yc_mm + rot_matrix[8] * src_zc_mm) + trans_matrix[2];

          /*3d dst camera coordinates to 2d dst image coordinates conversion*/
          int dst_x1 =
              (int)(((dstc_coord[0] * cam_matrix[0] + trans_matrix[0]) / dstc_coord[2]) + cam_matrix[2] + 0.5f);
          int dst_y1 =
              (int)(((dstc_coord[1] * cam_matrix[4] + trans_matrix[1]) / dstc_coord[2]) + cam_matrix[5] + 0.5f);

          if (dst_x1 < 0 || dst_x1 >= img_width || dst_y1 < 0 || dst_y1 >= img_height)
          {
            continue;
          }

          unsigned short dst_depth = dst_img[dst_y1 * img_width + dst_x1];
          unsigned short new_depth = (unsigned short)(dstc_coord[2]);
          if ((dst_depth == 0) || (dst_depth > new_depth))
          {
            if (new_depth > effective_floor_height_mm)
            {
              // Floor Removal
              // i->row, j->column and x->column, y->row
              int pos = (i * image_width_) + j;
              src_img[pos] = 0;
              int ptcld_pos = 3 * pos;
              xyz_frame_org[ptcld_pos] = 0;
              xyz_frame_org[ptcld_pos + 1] = 0;
              xyz_frame_org[ptcld_pos + 2] = 0;
              dst_img[dst_y1 * img_width + dst_x1] = 0;
            }
            else
            {
              dst_img[dst_y1 * img_width + dst_x1] = new_depth;
              // Update valid ROI limits
              min_x = std::min(min_x, dst_x1);
              min_y = std::min(min_y, dst_y1);
              max_x = std::max(max_x, dst_x1);
              max_y = std::max(max_y, dst_y1);
            }
          }
          else if (new_depth > effective_floor_height_mm)
          {
            // Floor Removal
            // i->row, j->column and x->column, y->row
            int pos = (i * image_width_) + j;
            src_img[pos] = 0;
            int ptcld_pos = 3 * pos;
            xyz_frame_org[ptcld_pos] = 0;
            xyz_frame_org[ptcld_pos + 1] = 0;
            xyz_frame_org[ptcld_pos + 2] = 0;
          }
        }
      }

      // Update ROI
      out_roi->x = std::max(0, min_x);
      out_roi->y = std::max(0, min_y);
      out_roi->width = std::min(std::max(0, (max_x - out_roi->x) + 1), img_width);
      out_roi->height = std::min(std::max(0, (max_y - out_roi->y) + 1), img_width);
    }
  }
}

/**
 * @brief    Generates a Range to 3D projection Look up table, which can be
 *           used to compute point-cloud from the depth image.
 *
 * @param camera_intrinsics camera intrinsics
 */
void ImageProcUtils::generateRangeTo3DLUT(CameraIntrinsics* camera_intrinsics)
{
  /* Generate Camera Intrinsics calibration arrays. */
  double k_raw_array[9] = { camera_intrinsics->camera_matrix[0], camera_intrinsics->camera_matrix[1],
                            camera_intrinsics->camera_matrix[2], camera_intrinsics->camera_matrix[3],
                            camera_intrinsics->camera_matrix[4], camera_intrinsics->camera_matrix[5],
                            camera_intrinsics->camera_matrix[6], camera_intrinsics->camera_matrix[7],
                            camera_intrinsics->camera_matrix[8] };
  double d_raw_array[8] = { camera_intrinsics->distortion_coeffs[0], camera_intrinsics->distortion_coeffs[1],
                            camera_intrinsics->distortion_coeffs[2], camera_intrinsics->distortion_coeffs[3],
                            camera_intrinsics->distortion_coeffs[4], camera_intrinsics->distortion_coeffs[5],
                            camera_intrinsics->distortion_coeffs[6], camera_intrinsics->distortion_coeffs[7] };

  cv::Mat k_raw = cv::Mat(3, 3, CV_64F, k_raw_array);
  cv::Mat d_raw = cv::Mat(1, 8, CV_64F, d_raw_array);
  cv::Size img_size(image_width_, image_height_);
  cv::Mat k_rect = cv::getOptimalNewCameraMatrix(k_raw, d_raw, img_size, 0, img_size, nullptr);

  // Prepare the rectification maps
  cv::Mat r = cv::Mat::eye(3, 3, CV_32F);

  short* range_to_3d_lut = range_to_xyz_lut_fixed_point_;

  for (int y = 0; y < image_height_; y++)
  {
    for (int x = 0; x < image_width_; x++)
    {
      cv::Mat distorted_pt(1, 1, CV_32FC2, cv::Scalar(x, y));
      cv::Mat undistorted_pt(1, 1, CV_32FC2);

      cv::undistortPoints(distorted_pt, undistorted_pt, k_raw, d_raw);

      float ux = undistorted_pt.at<float>(0);
      float uy = undistorted_pt.at<float>(1);

      float scale_factor = 1.0f / sqrtf(1.0f + ux * ux + uy * uy);

      *range_to_3d_lut++ = short((ux * scale_factor) * QFORMAT_FOR_POINTCLOUD_LUT_SHIFTED_VALUE);
      *range_to_3d_lut++ = short((uy * scale_factor) * QFORMAT_FOR_POINTCLOUD_LUT_SHIFTED_VALUE);
      *range_to_3d_lut++ = short(scale_factor * QFORMAT_FOR_POINTCLOUD_LUT_SHIFTED_VALUE);
    }
  }
}

/**
 * @brief Computes point cloud using range_to_xyz_lut look up table and range image
 *
 * @param range_image Range image
 * @param xyz_frame Output frame for point cloud
 */

void ImageProcUtils::computePointCloud(unsigned short* range_image, short* xyz_frame)
{
  short* range_to_xyz_lut_fixed_point = range_to_xyz_lut_fixed_point_;
  for (int i = 0; i < image_height_; i++)
  {
    for (int j = 0; j < image_width_; j++)
    {
      *xyz_frame++ = ((*range_to_xyz_lut_fixed_point++) * (*range_image)) >> QFORMAT_FOR_POINTCLOUD_LUT;
      *xyz_frame++ = ((*range_to_xyz_lut_fixed_point++) * (*range_image)) >> QFORMAT_FOR_POINTCLOUD_LUT;
      *xyz_frame++ = ((*range_to_xyz_lut_fixed_point++) * (*range_image)) >> QFORMAT_FOR_POINTCLOUD_LUT;
      range_image++;
    }
  }
}

/**
 * @brief This function does the matrix multiplication
 *
 * @param input_matrix1 address of first matrix array
 * @param rows1 Number of rows in first matrix
 * @param columns1 Number of columns in first matrix
 * @param input_matrix2 address of second matrix array
 * @param rows2 Number of rows in second matrix
 * @param columns2 Number of columns in second matrix
 * @param output_matrix address of output matrix array
 */
void ImageProcUtils::matrixMultiplication(float* input_matrix1, int rows1, int columns1, float* input_matrix2,
                                          int rows2, int columns2, float* output_matrix)
{
  /*Assumption output_matrix has minimum size = rows1 x columns2*/
  /*Rule: No. of columns of matrix1 should be equal to no.of rows of matrix2*/
  if (input_matrix1 == nullptr || input_matrix2 == nullptr || output_matrix == nullptr || columns1 != rows2)
  {
    return;
  }

  for (int r1 = 0; r1 < rows1; r1++)
  {
    for (int c2 = 0; c2 < columns2; c2++)
    {
      *((output_matrix + r1 * columns2) + c2) = 0;
      for (int i = 0; i < rows2; i++)
      {
        *((output_matrix + r1 * columns2) + c2) +=
            *((input_matrix1 + r1 * columns1) + i) * *((input_matrix2 + i * columns2) + c2);
      }
    }
  }
}

/**
 * @brief This function is the optimized version of multiplying 3x3 and 3x1 matrices.
 *
 * @param input_matrix1 address of first matrix array
 * @param input_matrix2 address of second matrix array
 * @param output_matrix address of output matrix array
 */
void ImageProcUtils::matrixMultiplication3x3And3x1(float* input_matrix1, short* input_matrix2, short* output_matrix)
{
  if (input_matrix1 == nullptr || input_matrix2 == nullptr || output_matrix == nullptr)
  {
    return;
  }

  output_matrix[0] =
      input_matrix1[0] * input_matrix2[0] + input_matrix1[1] * input_matrix2[1] + input_matrix1[2] * input_matrix2[2];
  input_matrix1 += 3;
  output_matrix[1] =
      input_matrix1[0] * input_matrix2[0] + input_matrix1[1] * input_matrix2[1] + input_matrix1[2] * input_matrix2[2];
  input_matrix1 += 3;
  output_matrix[2] =
      input_matrix1[0] * input_matrix2[0] + input_matrix1[1] * input_matrix2[1] + input_matrix1[2] * input_matrix2[2];
  input_matrix1 += 3;
}

/**
 * @brief Rotates the given input pointcloud using rotation matrix provided
 *
 * @param input_point_cloud Input point cloud
 * @param rotated_point_cloud Output rotated point cloud
 * @param camera_extrinsics Camera extrinsics which provides rotation matrix
 * @param image_width Image width
 * @param image_height Image height
 */
void ImageProcUtils::rotatePointCloud(short* input_point_cloud, short* rotated_point_cloud,
                                      CameraExtrinsics* camera_extrinsics, int image_width, int image_height)
{
  float* rotation_matrix = camera_extrinsics->rotation_matrix;
  for (int i = 0; i < (image_width * image_height); i++)
  {
    if (input_point_cloud[2] == 0)
    {
      *rotated_point_cloud++ = 0;
      *rotated_point_cloud++ = 0;
      *rotated_point_cloud++ = 0;
      input_point_cloud += 3;
    }
    else
    {
      matrixMultiplication3x3And3x1(&rotation_matrix[0], input_point_cloud, rotated_point_cloud);
      input_point_cloud += 3;
      rotated_point_cloud += 3;
    }
  }
}
