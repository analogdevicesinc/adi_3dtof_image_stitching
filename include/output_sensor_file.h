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
class OutputSensorFile : public IOutputSensor
{
public:
  void open(std::string input_file_name, int image_width, int image_height);
  void write(
    const cv::Mat & stitched_depth_frame_16bb, const cv::Mat & stitched_ir_frame_16bb,
    int image_width, int image_height);
  void close();
  OutputSensorFile();
  ~OutputSensorFile() {}

private:
  bool video_enabled_;
  cv::VideoWriter * output_video_writer_;
  std::string output_video_file_name_;

  void openOutputVideoFile(const std::string & input_file_name, int image_width, int image_height);
  void writeOutputVideoFile(const cv::Mat & image);
  void closeOutputVideoFile();
};

#endif