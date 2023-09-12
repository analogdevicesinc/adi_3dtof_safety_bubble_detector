/******************************************************************************
Copyright (c), 2023 - Analog Devices Inc. All Rights Reserved.
This software is PROPRIETARY & CONFIDENTIAL to Analog Devices, Inc.
and its licensors.
******************************************************************************/
#include "module_profile.h"
#include "adi_3dtof_safety_bubble_detector_node.h"
#include <boost/thread/thread.hpp>

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
int main(int argc, char** argv)
{
  ros::init(argc, argv, "adi_3dtof_safety_bubble_detector");

  INIT_FUNCTION_PROFILE();

  // Create an instance of ADI3DToFSafetyBubbleDetector
  ADI3DToFSafetyBubbleDetector adi_3dtof_safety_bubble_detector;

  bool thread_spawn_status = true;
  // Spawn the process output thread..
  // Note: It does nothing till the processing is triggered
  boost::thread process_output_thread;
  try
  {
    process_output_thread =
        boost::thread(&ADI3DToFSafetyBubbleDetector::processOutput, &adi_3dtof_safety_bubble_detector);
  }
  catch (const std::exception& e)
  {
    thread_spawn_status = false;
    std::cerr << "Exception when trying to spawn process_output_thread : " << e.what() << '\n';
  }

  // Spawn the read input thread..
  boost::thread read_input_thread;
  try
  {
    read_input_thread = boost::thread(&ADI3DToFSafetyBubbleDetector::readInput, &adi_3dtof_safety_bubble_detector);
  }
  catch (const std::exception& e)
  {
    thread_spawn_status = false;
    std::cerr << "Exception when trying to spawn read_input_thread : " << e.what() << '\n';
  }

  ros::Rate loop_rate(100);

  int frame_num = 0;
  while ((ros::ok()) && (thread_spawn_status))
  {
    ROS_INFO_STREAM("adi_3dtof_safety_bubble_detector::Running loop : " << frame_num++);

    /**
     * The publish() function is how you send messages. The parameter
     * is the message object. The type of this object must agree with the type
     * given as a template parameter to the advertise<>() call, as was done
     * in the constructor above.
     */
    if (!adi_3dtof_safety_bubble_detector.runSafetyBubbleDetection())
    {
      break;
    }

    FLUSH_FUNCTION_PROFILE();

    loop_rate.sleep();

    ros::spinOnce();
  }

  if (thread_spawn_status)
  {
    // Signal thread abort
    adi_3dtof_safety_bubble_detector.readInputAbort();
    try
    {
      // Wait for the thread to complete
      read_input_thread.join();
    }
    catch (const std::exception& e)
    {
      std::cerr << " Exception in read_input_thread.join() : " << e.what() << '\n';
    }

    // Signal thread abort
    adi_3dtof_safety_bubble_detector.processOutputAbort();
    try
    {
      // Wait for the thread to complete
      process_output_thread.join();
    }
    catch (const std::exception& e)
    {
      std::cerr << " Exception in process_output_thread.join() : " << e.what() << '\n';
    }
  }

  CLOSE_FUNCTION_PROFILE();

  // adi_3dtof_safety_bubble_detector.shutDownAllNodes();
  ros::shutdown();

  return 0;
}
