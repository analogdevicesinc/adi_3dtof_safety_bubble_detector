/******************************************************************************
Copyright (c), 2023 - Analog Devices Inc. All Rights Reserved.
This software is PROPRIETARY & CONFIDENTIAL to Analog Devices, Inc.
and its licensors.
******************************************************************************/

#ifndef OUTPUT_SENSOR_H
#define OUTPUT_SENSOR_H

#include <fstream>
#include <iostream>

/*Video Write*/
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>

/**
 * @brief enum to represent output flag.
 *
 */

enum OutputFlag {
  /**
   * @brief 0: No outputs are saved into files
   *
   */
  DisableAllOutputs,
  /**
   * @brief 1: Enable only csv output
   *
   */
  EnableCSVOutputOnly,
  /**
   * @brief 2: Enable only video(.avi) output
   *
   */
  EnableVideoOutputOnly,
  /**
   * @brief 3: Enable both CSV and Video outputs
   *
   */
  EnableAllOutputs
};

/**
 * @brief This is base class for output
 *
 */
class OOutputSensor
{
public:
  virtual void open(
    std::string input_file_name, int image_width, int image_height, OutputFlag output_flag) = 0;
  virtual void write(
    int frame_number, bool object_detected, unsigned short * depth_frame_16bpp,
    const cv::Mat & out_visualization_image, int image_width, int image_height,
    bool floor_detection_status, int ransac_iterations, int noise_count,
    float ransac_time_ms_out) = 0;
  virtual void close() = 0;
};

#endif
