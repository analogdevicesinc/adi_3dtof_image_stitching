/******************************************************************************
Copyright (c), 2023 - Analog Devices Inc. All Rights Reserved.
This software is PROPRIETARY & CONFIDENTIAL to Analog Devices, Inc.
and its licensors.
******************************************************************************/

#ifndef IMAGE_PROC_UTILS_H
#define IMAGE_PROC_UTILS_H

#include <cv_bridge/cv_bridge.h>

#include <cmath>
#include <opencv2/calib3d.hpp>

#include "adi_camera.h"

/**
 * @brief This class has image processing utilities
 *
 */
class ImageProcUtils
{
public:
  // default constructor with no argument
  ImageProcUtils() = default;

  ImageProcUtils(CameraIntrinsics * camera_intrinsics, int image_width, int image_height)
  {
    image_width_ = image_width;
    image_height_ = image_height;

    // Generate LUT for range to depth correction
    range_to_3d_lut_ = new float[image_width_ * image_height_ * 3];
    memset(range_to_3d_lut_, 0, sizeof(float) * image_width_ * image_height_ * 3);

    generateRangeTo3DLUT(camera_intrinsics);
  }

  ~ImageProcUtils() { delete[] range_to_3d_lut_; }

  float * getRangeToXYZLUT() { return range_to_3d_lut_; }
  /**
   * @brief    Generates a Range to 3D projection Look up table, which can be
   *           used to compute point-cloud from the depth image.
   *
   * @param camera_intrinsics camera intrinsics
   */
  void generateRangeTo3DLUT(CameraIntrinsics * camera_intrinsics)
  {
    /* Generate Camera Intrinsics calibration arrays. */
    double k_raw_array[9] = {
      camera_intrinsics->camera_matrix[0], camera_intrinsics->camera_matrix[1],
      camera_intrinsics->camera_matrix[2], camera_intrinsics->camera_matrix[3],
      camera_intrinsics->camera_matrix[4], camera_intrinsics->camera_matrix[5],
      camera_intrinsics->camera_matrix[6], camera_intrinsics->camera_matrix[7],
      camera_intrinsics->camera_matrix[8]};
    double d_raw_array[8] = {
      camera_intrinsics->distortion_coeffs[0], camera_intrinsics->distortion_coeffs[1],
      camera_intrinsics->distortion_coeffs[2], camera_intrinsics->distortion_coeffs[3],
      camera_intrinsics->distortion_coeffs[4], camera_intrinsics->distortion_coeffs[5],
      camera_intrinsics->distortion_coeffs[6], camera_intrinsics->distortion_coeffs[7]};

    cv::Mat K_raw = cv::Mat(3, 3, CV_64F, k_raw_array);
    cv::Mat D_raw = cv::Mat(1, 8, CV_64F, d_raw_array);
    cv::Size img_size(image_width_, image_height_);
    cv::Mat K_rect = cv::getOptimalNewCameraMatrix(K_raw, D_raw, img_size, 0, img_size, nullptr);

    // Prepare the rectification maps
    cv::Mat r = cv::Mat::eye(3, 3, CV_32F);

    float * range_to_3d_lut = range_to_3d_lut_;

    for (int y = 0; y < image_height_; y++) {
      for (int x = 0; x < image_width_; x++) {
        cv::Mat distorted_pt(1, 1, CV_32FC2, cv::Scalar(x, y));
        cv::Mat undistorted_pt(1, 1, CV_32FC2);

        cv::undistortPoints(distorted_pt, undistorted_pt, K_raw, D_raw);

        float ux = undistorted_pt.at<float>(0);
        float uy = undistorted_pt.at<float>(1);

        float scale_factor = 1.0f / sqrtf(1.0f + ux * ux + uy * uy);

        *range_to_3d_lut++ = (ux * scale_factor);
        *range_to_3d_lut++ = (uy * scale_factor);
        *range_to_3d_lut++ = (scale_factor);
      }
    }
  }

  /**
 * @brief Computes point cloud using range_to_xyz_lut look up table and range
 * image
 *
 * @param range_image range image
 * @param xyz_frame output frame for point cloud
 */

  void computePointCloud(unsigned short * range_image, short * xyz_frame)
  {
    float * range_to_3d_lut = range_to_3d_lut_;
    for (int i = 0; i < image_height_; i++) {
      for (int j = 0; j < image_width_; j++) {
        unsigned short depth = (*range_image);
        if (depth == 0) depth = 8000;
        *xyz_frame++ = (*range_to_3d_lut++) * (depth);
        *xyz_frame++ = (*range_to_3d_lut++) * (depth);
        *xyz_frame++ = (*range_to_3d_lut++) * (depth);
        range_image++;
      }
    }
  }

  /**
   * @brief Function for Gamma Correction
   *
   * @param src_dst - Buffer pointer
   * @param num_pixels - Number of pixels
   */
  static void gammaCorrect(unsigned short * src_dst, int num_pixels)
  {
    for (int i = 0; i < num_pixels; i++) {
      float pixel_value = (float)src_dst[i];
      float out_pixel_value = (float)(255.0f * log(pixel_value)) / log(2048.0f);
      src_dst[i] = (unsigned short)out_pixel_value;
    }
  }

private:
  int image_width_;
  int image_height_;
  float * range_to_3d_lut_;
};

#endif
