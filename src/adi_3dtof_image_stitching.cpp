/******************************************************************************
Copyright (c), 2023 - Analog Devices Inc. All Rights Reserved.
This software is PROPRIETARY & CONFIDENTIAL to Analog Devices, Inc.
and its licensors.
******************************************************************************/
#include "adi_3dtof_image_stitching.h"
#include <ros/ros.h>
#include <boost/thread/thread.hpp>

/**
 * @brief This is main function
 *
 * @param argc number of input arguments to the function
 * @param argv array of pointer to the input arguments
 * @return int
 */
int main(int argc, char** argv)
{
  ros::init(argc, argv, "adi_adtf31xx_sensor_stitch");

  // initialize function profiling
  INIT_FUNCTION_PROFILE();

  // Create an instance of ADI3DToFImageStitching
  ADI3DToFImageStitching adi_3dtof_image_stitching;

  bool thread_spawn_status = true;
  // Spawn the process output thread..
  // Note: It does nothing till the processing is triggered
  boost::thread process_output_thread;
  try
  {
    process_output_thread = boost::thread(&ADI3DToFImageStitching::processOutput, &adi_3dtof_image_stitching);
  }
  catch (const std::exception& e)
  {
    thread_spawn_status = false;
    std::cerr << "Exception when trying to spawn process_output_thread : " << e.what() << '\n';
  }

  ros::Rate loop_rate(100);

  while ((ros::ok()) && (thread_spawn_status))
  {
    /**
     * The publish() function is how you send messages. The parameter
     * is the message object. The type of this object must agree with the type
     * given as a template parameter to the advertise<>() call, as was done
     * in the constructor above.
     */
    if (!adi_3dtof_image_stitching.stitchFrames())
    {
      adi_3dtof_image_stitching.shutDownAllNodes();
    }

    FLUSH_FUNCTION_PROFILE();

    loop_rate.sleep();

    ros::spinOnce();
  }

  // closing profiling
  CLOSE_FUNCTION_PROFILE();

  // Flag error
  adi_3dtof_image_stitching.inputReadAbort();
  if (thread_spawn_status)
  {
    // Signal thread abort
    adi_3dtof_image_stitching.processOutputAbort();
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

  ros::shutdown();

  return 0;
}
