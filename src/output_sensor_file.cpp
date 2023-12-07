/******************************************************************************
Copyright (c), 2023 - Analog Devices Inc. All Rights Reserved.
This software is PROPRIETARY & CONFIDENTIAL to Analog Devices, Inc.
and its licensors.
******************************************************************************/

#include "output_sensor_file.h"

#include "image_proc_utils.h"

/**
 * @brief Constructor for OutputSensorFile class
 */
OutputSensorFile::OutputSensorFile()
{
  video_enabled_ = false;
  output_video_writer_ = nullptr;
  output_video_file_name_ = "";
}

/**
 * @brief Opens output files (video and/or csv)
 *
 * @param input_file_name Input file name
 * @param image_width Image Width
 * @param image_height Image Height
 */
void OutputSensorFile::open(std::string input_file_name, int image_width, int image_height)
{
  // Open output video file
  openOutputVideoFile(
    input_file_name, image_width,
    image_height * 2);  // Multiplied height by 2 as we will be sending
                        // both depth and IR out, vertically concatenated
}

/**
 * @brief Writes one frame to output video file
 * 
 * @param stitched_depth_frame_16bb - 16-bit stitched depth input
 * @param stitched_ir_frame_16bb - 16Bit stitched IR input
 * @param image_width - input image width
 * @param image_height - input image height
 */
void OutputSensorFile::write(
  const cv::Mat & stitched_depth_frame_16bb, const cv::Mat & stitched_ir_frame_16bb,
  int image_width, int image_height)
{
  // Get 8bit image
  cv::Mat depth_8bit_image = cv::Mat::zeros(image_height, image_width, CV_8UC1);
  unsigned short max_element = 8192;
  float scale_factor = 255.0f / max_element;
  stitched_depth_frame_16bb.convertTo(depth_8bit_image, CV_8UC1, scale_factor, 0);

  // Get rgb 8 bit image
  cv::Mat depth_8bit_rgb_image = cv::Mat::zeros(image_height, image_width, CV_8UC3);
  cv::cvtColor(depth_8bit_image, depth_8bit_rgb_image, cv::COLOR_GRAY2BGR);

  // Get 8bit image
  cv::Mat ir_8bit_image = cv::Mat::zeros(image_height, image_width, CV_8UC1);
  ImageProcUtils::gammaCorrect(
    (unsigned short *)stitched_ir_frame_16bb.data, image_height * image_width);
  stitched_ir_frame_16bb.convertTo(ir_8bit_image, CV_8UC1, 1, 0);

  // Get rgb 8 bit image
  cv::Mat ir_8bit_rgb_image = cv::Mat::zeros(image_height, image_width, CV_8UC3);
  cv::cvtColor(ir_8bit_image, ir_8bit_rgb_image, cv::COLOR_GRAY2BGR);

  // Concatenation of input depth and algo output image
  cv::Mat final_out_image = cv::Mat::zeros(cv::Size(image_width, image_height * 2), CV_8UC3);
  cv::vconcat(ir_8bit_rgb_image, depth_8bit_rgb_image, final_out_image);

  // Show output in opencv window
  cv::namedWindow("Stitch_Output", cv::WINDOW_AUTOSIZE);
  cv::imshow("Stitch_Output", final_out_image);
  cv::waitKey(1);

  writeOutputVideoFile(final_out_image);
}

/**
 * @brief Closes all opened output files
 *
 */
void OutputSensorFile::close() { closeOutputVideoFile(); }

/**
 * @brief Opens output video file
 *
 * @param input_file_name Input file name
 * @param image_width Image Width
 * @param image_height Image Height
 */
void OutputSensorFile::openOutputVideoFile(
  const std::string & input_file_name, int image_width, int image_height)
{
  output_video_file_name_ = input_file_name.substr(0, input_file_name.find_last_of('.')) + ".avi";
  output_video_writer_ = new cv::VideoWriter(
    output_video_file_name_, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), 10,
    cv::Size(image_width, image_height), true);
  if (!output_video_writer_->isOpened()) {
    std::cout << "Could not open output video file for the input " << input_file_name << std::endl;
  }
}

/**
 * @brief Writes output video file
 *
 * @param image Output image
 */
void OutputSensorFile::writeOutputVideoFile(const cv::Mat & image)
{
  if (output_video_writer_->isOpened()) {
    output_video_writer_->write(image);
  }
}

/**
 * @brief Closes output video file
 *
 */
void OutputSensorFile::closeOutputVideoFile()
{
  if (output_video_writer_->isOpened()) {
    output_video_writer_->release();
    output_video_writer_ = nullptr;
  }
}