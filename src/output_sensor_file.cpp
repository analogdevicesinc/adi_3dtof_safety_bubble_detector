/******************************************************************************
Copyright (c), 2023 - Analog Devices Inc. All Rights Reserved.
This software is PROPRIETARY & CONFIDENTIAL to Analog Devices, Inc.
and its licensors.
******************************************************************************/

#include "output_sensor_file.h"
#include "image_proc_utils.h"

/**
 * @brief Opens output files (video and/or csv)
 *
 * @param input_file_name Input file name
 * @param image_width Image Width
 * @param image_height Image Height
 * @param output_flag enum to enable csv or video output
 */
void OutputSensorFile::open(std::string input_file_name, int image_width, int image_height, OutputFlag output_flag)
{
  if (output_flag == EnableCSVOutputOnly)
  {
    csv_enabled_ = true;
  }
  else if (output_flag == EnableVideoOutputOnly)
  {
    video_enabled_ = true;
  }
  else if (output_flag == EnableAllOutputs)
  {
    csv_enabled_ = true;
    video_enabled_ = true;
  }

  if (csv_enabled_)
  {
    // Open the output csv file
    openOutputCsvFile(input_file_name);
  }

  if (video_enabled_)
  {
    // Open output video file
    openOutputVideoFile(input_file_name, image_width * 2, image_height);
  }
}

/**
 * @brief writes output files (video and/or csv)
 *
 * @param frame_number Current frame number
 * @param object_detected This flag indicates object detection
 * @param depth_frame_16bpp Pointer to depth image
 * @param m_out_visualization_image Pointer to output image
 * @param image_width Image width
 * @param image_height Image height
 * @param floor_detection_status Floor detection status
 * @param ransac_iterations Number of iterations ransac took to converge
 * @param noise_count Noisy pixels below RANSAC plane
 * @param ransac_time_ms_out The time taken by ransac per frame
 */
void OutputSensorFile::write(int frame_number, bool object_detected, unsigned short* depth_frame_16bpp,
                             const cv::Mat& m_out_visualization_image, int image_width, int image_height,
                             bool floor_detection_status, int ransac_iterations, int noise_count,
                             float ransac_time_ms_out)
{
  // Get 8bit image
  cv::Mat depth_8bit_image = cv::Mat::zeros(image_height, image_width, CV_8UC1);
  unsigned short max_element = 8192;
  float scale_factor = 255.0f / max_element;
  cv::Mat depth_16bit_image = cv::Mat(image_height, image_width, CV_16UC1, depth_frame_16bpp);
  depth_16bit_image.convertTo(depth_8bit_image, CV_8UC1, scale_factor, 0);

  // Get rgb 8 bit image
  cv::Mat depth_8bit_rgb_image = cv::Mat::zeros(image_height, image_width, CV_8UC3);
  applyColorMap(depth_8bit_image, depth_8bit_rgb_image, cv::COLORMAP_JET);
  // cv::cvtColor(depth_8bit_image, depth_8bit_rgb_image, cv::COLOR_GRAY2BGR);

  // Concatenation of input depth and algo output image
  cv::Mat final_out_image = cv::Mat::zeros(cv::Size(image_width * 2, image_height), CV_8UC3);
  cv::hconcat(depth_8bit_rgb_image, m_out_visualization_image, final_out_image);

  if (csv_enabled_)
  {
    writeOutputCsvFile(frame_number, object_detected, floor_detection_status, ransac_iterations, noise_count,
                       ransac_time_ms_out);
  }

  if (video_enabled_)
  {
    writeOutputVideoFile(final_out_image);
  }
}

/**
 * @brief Closes all opened output files
 *
 */
void OutputSensorFile::close()
{
  if (csv_enabled_)
  {
    closeOutputCsvFile();
  }

  if (video_enabled_)
  {
    closeOutputVideoFile();
  }
}

/**
 * @brief Opens output csv file
 *
 * @param input_file_name Input file name
 */
void OutputSensorFile::openOutputCsvFile(const std::string& input_file_name)
{
  if (input_file_name.find('.') != std::string::npos)
  {
    output_csv_file_name_ = input_file_name.substr(0, input_file_name.find_last_of('.')) + ".csv";
  }
  else
  {
    output_csv_file_name_ = input_file_name + ".csv";
  }

  // Open file for streaming.
  output_csv_file_.open(output_csv_file_name_, std::ios::out);
  if (output_csv_file_.is_open())
  {
    // Update flag.
    output_csv_file_ << "frame_number"
                     << ","
                     << "zone_status"
                     << ","
                     << "ransac_floor_detection_status"
                     << ","
                     << "ransac_iterations"
                     << ","
                     << "noise_count"
                     << ","
                     << "ransac_time_in_ms" << std::endl;
  }
  else
  {
    std::cout << "Could not open output csv file for the input " << input_file_name << std::endl;
  }
}

/**
 * @brief Opens output video file
 *
 * @param input_file_name Input file name
 * @param image_width Image Width
 * @param image_height Image Height
 */
void OutputSensorFile::openOutputVideoFile(const std::string& input_file_name, int image_width, int image_height)
{
  if (input_file_name.find('.') != std::string::npos)
  {
    output_video_file_name_ = input_file_name.substr(0, input_file_name.find_last_of('.')) + ".avi";
  }
  else
  {
    output_video_file_name_ = input_file_name + ".avi";
  }
  output_video_writer_ = new cv::VideoWriter(output_video_file_name_, cv::VideoWriter::fourcc('m', 'j', 'p', 'g'), 10,
                                             cv::Size(image_width, image_height), true);
  if (!output_video_writer_->isOpened())
  {
    std::cout << "Could not open output video file for the input " << input_file_name << std::endl;
  }
}

/**
 * @brief writes output csv file
 *
 * @param frame_number Current frame number
 * @param object_detected This flag indicates object detection
 * @param floor_detection_status Floor detection status
 * @param ransac_iterations Number of iterations ransac took to converge
 * @param noise_count Noisy pixels below RANSAC plane
 * @param ransac_time_ms_out The time taken by ransac per frame
 */
void OutputSensorFile::writeOutputCsvFile(int frame_number, bool object_detected, bool floor_detection_status,
                                          int ransac_iterations, int noise_count, float ransac_time_ms_out)
{
  if (output_csv_file_.is_open())
  {
    output_csv_file_ << frame_number << "," << object_detected << "," << floor_detection_status << ","
                     << ransac_iterations << "," << noise_count << "," << ransac_time_ms_out << std::endl;
  }
}

/**
 * @brief Writes output video file
 *
 * @param image Output image
 */
void OutputSensorFile::writeOutputVideoFile(const cv::Mat& image)
{
  if (output_video_writer_->isOpened())
  {
    output_video_writer_->write(image);
  }
}

/**
 * @brief closes the file
 *
 */
void OutputSensorFile::closeOutputCsvFile()
{
  if (output_csv_file_.is_open())
  {
    output_csv_file_.close();
  }
}

/**
 * @brief Closes output video file
 *
 */
void OutputSensorFile::closeOutputVideoFile()
{
  if (output_video_writer_->isOpened())
  {
    output_video_writer_->release();
    output_video_writer_ = nullptr;
  }
}