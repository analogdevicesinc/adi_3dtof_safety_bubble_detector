/******************************************************************************
Copyright (c), 2023 - Analog Devices Inc. All Rights Reserved.
This software is PROPRIETARY & CONFIDENTIAL to Analog Devices, Inc.
and its licensors.
******************************************************************************/

#ifndef OUTPUT_SENSOR_FILE_H
#define OUTPUT_SENSOR_FILE_H

#include "output_sensor.h"

/**
 * @brief This class is for output sensor file
 *
 */
class OutputSensorFile : public OOutputSensor
{
public:
  void open(std::string input_file_name, int image_width, int image_height, OutputFlag output_flag);
  void write(int frame_number, bool object_detected, unsigned short* depth_frame_16bpp,
             const cv::Mat& out_visualization_image, int image_width, int image_height, bool floor_detection_status,
             int ransac_iterations, int noise_count, float ransac_time_ms_out);
  void close();

private:
  bool csv_enabled_ = false;
  bool video_enabled_ = false;
  std::ofstream output_csv_file_;
  std::string output_csv_file_name_;
  cv::VideoWriter* output_video_writer_;
  std::string output_video_file_name_;

  void openOutputCsvFile(const std::string& input_file_name);
  void openOutputVideoFile(const std::string& input_file_name, int image_width, int image_height);
  void writeOutputCsvFile(int frame_number, bool object_detected, bool floor_detection_status, int ransac_iterations,
                          int noise_count, float ransac_time_ms_out);
  void writeOutputVideoFile(const cv::Mat& image);
  void closeOutputCsvFile();
  void closeOutputVideoFile();
};

#endif