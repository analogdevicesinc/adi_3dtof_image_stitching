/******************************************************************************
Copyright (c), 2023 - Analog Devices Inc. All Rights Reserved.
This software is PROPRIETARY & CONFIDENTIAL to Analog Devices, Inc.
and its licensors.
******************************************************************************/
#include "adi_3dtof_image_stitching.h"

#include <rclcpp/rclcpp.hpp>
#include <thread>

/**
 * @brief This is main function
 *
 * @param argc number of input arguments to the function
 * @param argv array of pointer to the input arguments
 * @return int
 */
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  // initialize function profiling
  INIT_FUNCTION_PROFILE();

  // Create an instance of ADI3DToFImageStitching
  auto adi_3dtof_image_stitching = std::make_shared<ADI3DToFImageStitching>();

  bool thread_spawn_status = true;
  // Spawn the process output thread..
  // Note: It does nothing till the processing is triggered
  std::thread process_output_thread;
  try {
    process_output_thread =
      std::thread(&ADI3DToFImageStitching::processOutput, adi_3dtof_image_stitching);
  } catch (const std::exception & e) {
    thread_spawn_status = false;
    std::cerr << "Exception when trying to spawn process_output_thread : " << e.what() << '\n';
  }
  if (thread_spawn_status) {
    rclcpp::Rate loop_rate(100);

    while (rclcpp::ok()) {
      /**
       * The publish() function is how you send messages. The parameter
       * is the message object. The type of this object must agree with the type
       * given as a template parameter to the advertise<>() call, as was done
       * in the constructor above.
       */
      if (!adi_3dtof_image_stitching->stitchFrames()) {
        break;
      }

      try {
        rclcpp::spin_some(adi_3dtof_image_stitching);
      } catch (const std::exception & e) {
        std::cerr << " Exception, shutting down the node  : " << e.what() << '\n';
      }
    }

    FLUSH_FUNCTION_PROFILE();

    adi_3dtof_image_stitching->inputReadAbort();
    // Signal thread abort
    adi_3dtof_image_stitching->processOutputAbort();
    try {
      // Wait for the thread to complete
      process_output_thread.join();
    } catch (const std::exception & e) {
      std::cerr << " Exception in process_output_thread.join() : " << e.what() << '\n';
    }
  }

  // closing profiling
  CLOSE_FUNCTION_PROFILE();

  rclcpp::shutdown();

  return 0;
}
