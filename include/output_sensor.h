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
 * @brief This is base class for output
 *
 */
class IOutputSensor
{
public:
  virtual void open(std::string input_file_name, int image_width, int image_height) = 0;
  virtual void write(
    const cv::Mat & stitched_depth_frame_16bb, const cv::Mat & stitched_ir_frame_16bb,
    int image_width, int image_height) = 0;
  virtual void close() = 0;
};

#endif
