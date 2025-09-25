/******************************************************************************
Copyright (c), 2023 - Analog Devices Inc. All Rights Reserved.
This software is PROPRIETARY & CONFIDENTIAL to Analog Devices, Inc.
and its licensors.
******************************************************************************/
#include "adi_3dtof_safety_bubble_detector_node.h"
#include "module_profile.h"

namespace enc = sensor_msgs::image_encodings;

/**
 * @mainpage
 *
 * @htmlinclude manifest.html
 */

/**
 * @brief This is main function for the Safety Bubble Detector application
 *
 * @param argc number of input arguments to the function
 * @param argv array of pointer to the input arguments
 * @return 0 - Success
 */
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  INIT_FUNCTION_PROFILE();

  // Create an instance of ADI3DToFADTF31xx
  auto adi_3dtof_safety_bubble_detector = std::make_shared<ADI3DToFSafetyBubbleDetector>();

  bool thread_spawn_status = true;

  // Spawn the read input thread..
  std::thread read_input_thread;
  try {
    read_input_thread =
      std::thread(&ADI3DToFSafetyBubbleDetector::readInput, adi_3dtof_safety_bubble_detector);
  } catch (const std::exception & e) {
    thread_spawn_status = false;
    std::cerr << "Exception when trying to spawn read_input_thread : " << e.what() << '\n';
  }

  // Spawn the process output thread..
  // Note: It does nothing till the processing is triggered
  std::thread process_output_thread;
  try {
    process_output_thread =
      std::thread(&ADI3DToFSafetyBubbleDetector::processOutput, adi_3dtof_safety_bubble_detector);
  } catch (const std::exception & e) {
    thread_spawn_status = false;
    std::cerr << "Exception when trying to spawn process_output_thread : " << e.what() << '\n';
  }

  if (thread_spawn_status) {
    rclcpp::Rate loop_rate(100);
    try {
      rclcpp::spin(adi_3dtof_safety_bubble_detector);
    } catch (const std::exception & e) {
      std::cerr << " Exception, shutting down the node  : " << e.what() << '\n';
    }

    // Signal thread abort
    adi_3dtof_safety_bubble_detector->readInputAbort();
    try {
      // Wait for the thread to complete
      read_input_thread.join();
    } catch (const std::exception & e) {
      std::cerr << " Exception in read_input_thread.join() : " << e.what() << '\n';
    }

    // Signal thread abort
    adi_3dtof_safety_bubble_detector->processOutputAbort();
    try {
      // Wait for the thread to complete
      process_output_thread.join();
    } catch (const std::exception & e) {
      std::cerr << " Exception in process_output_thread.join() : " << e.what() << '\n';
    }
  }

  CLOSE_FUNCTION_PROFILE();

  // adi_3dtof_safety_bubble_detector.shutDownAllNodes();
  rclcpp::shutdown();

  return 0;
}
