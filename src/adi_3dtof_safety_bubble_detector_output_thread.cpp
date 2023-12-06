/******************************************************************************
Copyright (c), 2023 - Analog Devices Inc. All Rights Reserved.
This software is PROPRIETARY & CONFIDENTIAL to Analog Devices, Inc.
and its licensors.
******************************************************************************/
#include <chrono>

#include "adi_3dtof_safety_bubble_detector_node.h"
#include "module_profile.h"

namespace enc = sensor_msgs::image_encodings;
using namespace std::chrono_literals;

/**
 * @brief This function sets the abort flag for the output thread,
 * the function is normally called by the main function to abort the output thread.
 *
 */
void ADI3DToFSafetyBubbleDetector::processOutputAbort() { process_output_thread_abort_ = true; }

/**
 * @brief This function pushes the output node to the output queue.
 * If the queue is full, then the last item in the queue gets replaced
 * with the latest node.
 *
 * @param new_output_node : Pointer to the output node to be published
 */
void ADI3DToFSafetyBubbleDetector::safetyBubbleDetectorIOThreadPushOutputNode(
  ADI3DToFSafetyBubbleDetectorOutputInfo * new_output_node)
{
  output_thread_mtx_.lock();
  int queue_size = output_node_queue_.size();
  output_thread_mtx_.unlock();

  if (queue_size <= (max_output_queue_length_ - 1)) {
    // Push this one
    output_thread_mtx_.lock();
    output_node_queue_.push(new_output_node);
    output_thread_mtx_.unlock();
  } else {
    __attribute__((unused)) ADI3DToFSafetyBubbleDetectorOutputInfo * last_node = nullptr;
    // Replace the last item with the current one.
    output_thread_mtx_.lock();
    last_node = (ADI3DToFSafetyBubbleDetectorOutputInfo *)output_node_queue_.back();
    output_thread_mtx_.unlock();

    // Copy the contents of new node into the old one and then delete the new node.
    last_node = new_output_node;

    // Delete this one
    delete new_output_node;
  }
}

/**
 * @brief The output process function, this is running a loop
 * which reads the frame from the output queue, generates the visualization output
 * and publishes the output as ROS messages.
 *
 */
void ADI3DToFSafetyBubbleDetector::processOutput()
{
  int output_queue_size = output_node_queue_.size();

  while ((!process_output_thread_abort_) || (output_queue_size > 0)) {
    // std::cout << "Inside processOutput" << std::endl;
    output_thread_mtx_.lock();
    output_queue_size = output_node_queue_.size();
    output_thread_mtx_.unlock();
    if (output_queue_size > 0) {
      PROFILE_FUNCTION_START(processOutput_Thread)
      output_thread_mtx_.lock();
      ADI3DToFSafetyBubbleDetectorOutputInfo * new_frame =
        (ADI3DToFSafetyBubbleDetectorOutputInfo *)output_node_queue_.front();
      output_node_queue_.pop();
      output_thread_mtx_.unlock();

      // Generate Visualization image(_out_image).
      cv::Mat m_out_visualization_image = generateVisualizationImage(
        new_frame->vcam_depth_image_floor_pixels_removed_8bpp_,
        new_frame->vcam_depth_frame_with_floor_, new_frame->object_detected_);

      // Publish visualization output image.
      PROFILE_FUNCTION_START(Publish_CompressImg)
      if (enable_output_image_compression_) {
        publishCompressedImageAsRosMsg(
          m_out_visualization_image, "bgr8", "None", compressed_out_image_publisher_);
      } else {
        publishImageAsRosMsg(m_out_visualization_image, "bgr8", "None", out_image_publisher_);
      }
      PROFILE_FUNCTION_START(Publish_CompressImg)

      // Publish other debug images
      if (enable_depth_ir_compression_) {
        publishImageAndCameraInfo(
          new_frame->compressed_depth_frame_, new_frame->compressed_depth_frame_size_,
          new_frame->compressed_ir_frame_, new_frame->compressed_ir_frame_size_);
      } else {
        publishImageAndCameraInfo(
          new_frame->depth_frame_with_floor_, new_frame->ir_frame_, new_frame->vcam_depth_frame_,
          new_frame->xyz_frame_);
      }

      // Write outputs
      if (output_sensor_ != nullptr) {
        output_sensor_->write(
          new_frame->frame_number_, new_frame->object_detected_, new_frame->depth_frame_with_floor_,
          m_out_visualization_image, image_width_, image_height_,
          new_frame->ransac_floor_detection_status_, new_frame->ransac_iterations_,
          new_frame->noise_count_, new_frame->ransac_time_ms_);
      }

      delete new_frame;
      PROFILE_FUNCTION_END(processOutput_Thread)
    }

    // Sleep
    std::this_thread::sleep_for(2ms);
  }
}
