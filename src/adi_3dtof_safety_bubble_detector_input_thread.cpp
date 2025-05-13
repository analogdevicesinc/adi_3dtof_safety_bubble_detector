/******************************************************************************
Copyright (c), 2023 - Analog Devices Inc. All Rights Reserved.
This software is PROPRIETARY & CONFIDENTIAL to Analog Devices, Inc.
and its licensors.
******************************************************************************/
#include "module_profile.h"
#include "adi_3dtof_safety_bubble_detector_node.h"
#include "compressed_depth_image_transport/rvl_codec.h"
#include <boost/chrono.hpp>
#include <boost/thread/thread.hpp>

/**
 * @brief Function to read Abort flag, this function will be called by the main function to exit the thread.
 *
 */
void ADI3DToFSafetyBubbleDetector::readInputAbort()
{
  read_input_thread_abort_ = true;
}

/**
 * @brief Function to read the next frame from the input Queue
 *
 * @return ADTF31xxSensorFrameInfo* - Pointer to the next frame
 */
ADTF31xxSensorFrameInfo* ADI3DToFSafetyBubbleDetector::safetyBubbleDetectorIOThreadGetNextFrame()
{
  ADTF31xxSensorFrameInfo* inframe = nullptr;
  input_thread_mtx_.lock();
  int queue_size = input_frames_queue_.size();
  input_thread_mtx_.unlock();

  // We will try till we fill read the buffer, this is just to allow the read thread to fill the queue
  while ((inframe == nullptr) && ((!error_in_frame_read_) || queue_size > 0))
  {
    input_thread_mtx_.lock();
    queue_size = input_frames_queue_.size();
    input_thread_mtx_.unlock();
    if (queue_size > 0)
    {
      input_thread_mtx_.lock();
      inframe = (ADTF31xxSensorFrameInfo*)input_frames_queue_.front();
      input_frames_queue_.pop();
      input_thread_mtx_.unlock();
    }
    if (inframe == nullptr)
    {
      // Wait for the buffers to be filled
      boost::this_thread::sleep_for(boost::chrono::milliseconds(2));
    }
  }
  return inframe;
}

/**
 * @brief updates parameters of input image based on dynamic reconfigure.
 *
 */
void ADI3DToFSafetyBubbleDetector::updateDynamicReconfigureVariablesInputThread()
{
  if (ab_threshold_ != dynamic_reconfigure_config_.ab_threshold)
  {
    ab_threshold_ = dynamic_reconfigure_config_.ab_threshold;
    ROS_INFO("Changed AB threshold value is %d", ab_threshold_);
    input_sensor_->setABinvalidationThreshold(ab_threshold_);
  }

  if (confidence_threshold_ != dynamic_reconfigure_config_.confidence_threshold)
  {
    confidence_threshold_ = dynamic_reconfigure_config_.confidence_threshold;
    ROS_INFO("Changed Confidence threshold value is %d", confidence_threshold_);
    input_sensor_->setConfidenceThreshold(confidence_threshold_);
  }
}

/**
 * @brief This function reads the frame from the sensor. This runs in a loop,
 * reading the frames and adding them to the input Queue,
 * if the queue is fulll, then the recent buffer is overwritten with the
 * newly read frame.
 *
 */
void ADI3DToFSafetyBubbleDetector::readInput()
{
  while (!read_input_thread_abort_)
  {
    // ROS_INFO("Read loop");

    // Create a new node
    ADTF31xxSensorFrameInfo* new_frame = new ADTF31xxSensorFrameInfo(image_width_, image_height_);

    // Updates variables present in dynamic reconfigure
    updateDynamicReconfigureVariablesInputThread();

    if (new_frame != nullptr)
    {
      PROFILE_FUNCTION_START(readInput_Thread)
      bool result = input_sensor_->readNextFrame(new_frame->getDepthFrame(), new_frame->getIRFrame());
      input_sensor_->getFrameTimestamp(new_frame->getFrameTimestampPtr());
      PROFILE_FUNCTION_END(readInput_Thread)
      if (!result)
      {
        // free memory
        delete new_frame;
        error_in_frame_read_ = true;
        continue;
      }
      else
      {
        PROFILE_FUNCTION_START(SafetyBubble_POINT_CLOUD);
        image_proc_utils_->computePointCloud(new_frame->getDepthFrame(), new_frame->getXYZFrame());
        PROFILE_FUNCTION_END(SafetyBubble_POINT_CLOUD);

        PROFILE_FUNCTION_START(SafetyBubble_RANSAC_PREPROCESS)
        if (enable_ransac_floor_detection_)
        {
          int modified_xyz_frame_size = 0;
          if (camera_tilted_)
          {
            image_proc_utils_->rotatePointCloud(new_frame->getXYZFrame(), new_frame->getRotatedXYZFrame(),
                                                &depth_extrinsics_external_, image_width_, image_height_);
            modified_xyz_frame_size = floor_plane_detection_->preProcessPointCloud(
                new_frame->getRotatedXYZFrame(), new_frame->getModifiedXYZFrame(),
                new_frame->getNearbyObjectsFoundFlagPtr());
          }
          else
          {
            modified_xyz_frame_size = floor_plane_detection_->preProcessPointCloud(
                new_frame->getXYZFrame(), new_frame->getModifiedXYZFrame(), new_frame->getNearbyObjectsFoundFlagPtr());
          }
          new_frame->setModifiedXYZFrameSize(modified_xyz_frame_size);
        }
        PROFILE_FUNCTION_END(SafetyBubble_RANSAC_PREPROCESS)

        // Frame read succesfully, compress the frames.
        PROFILE_FUNCTION_START(RVL_EnCodeDepth_IRImg)
        if (enable_depth_ab_compression_)
        {
          compressed_depth_image_transport::RvlCodec rvl;
          unsigned short* raw_depth_frame = new_frame->getDepthFrame();
          unsigned char* compressed_depth_frame = new_frame->getCompressedDepthFrame();
          int compressed_size_depth_frame =
              rvl.CompressRVL(&raw_depth_frame[0], &compressed_depth_frame[0], image_width_ * image_height_);
          new_frame->setCompressedDepthFrameSize(compressed_size_depth_frame);

          // IR
          unsigned short* raw_ab_frame = new_frame->getIRFrame();
          unsigned char* compressed_ab_frame = new_frame->getCompressedIRFrame();
          int compressed_size_ab_frame =
              rvl.CompressRVL(&raw_ab_frame[0], &compressed_ab_frame[0], image_width_ * image_height_);
          new_frame->setCompressedIRFrameSize(compressed_size_ab_frame);
        }
        PROFILE_FUNCTION_START(RVL_EnCodeDepth_IRImg)
      }
    }
    else
    {
      error_in_frame_read_ = true;
      read_input_thread_abort_ = true;
      break;
    }

    // Add it to the queue
    input_thread_mtx_.lock();
    int queue_size = input_frames_queue_.size();
    input_thread_mtx_.unlock();
    if (queue_size <= (max_input_queue_length_ - 1))
    {
      input_thread_mtx_.lock();
      input_frames_queue_.push(new_frame);
      input_thread_mtx_.unlock();
    }
    else
    {
      std::cout << "Overwrite buffer" << std::endl;
      // If the Queue is full, then overwrite the last buffer with the latest frame
      input_thread_mtx_.lock();
      __attribute__((unused)) ADTF31xxSensorFrameInfo* last_node = (ADTF31xxSensorFrameInfo*)input_frames_queue_.back();
      input_thread_mtx_.unlock();
      last_node = new_frame;
      delete new_frame;
    }

    // sleep
    boost::this_thread::sleep_for(boost::chrono::milliseconds(2));
  }

  // Destroy the queue
  input_thread_mtx_.lock();
  int queue_size = input_frames_queue_.size();
  while (queue_size > 0)
  {
    // pop and delete
    ADTF31xxSensorFrameInfo* new_frame = (ADTF31xxSensorFrameInfo*)input_frames_queue_.front();
    input_frames_queue_.pop();
    delete new_frame;
    queue_size = input_frames_queue_.size();
  }
  input_thread_mtx_.unlock();
}