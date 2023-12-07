/******************************************************************************
Copyright (c), 2023 - Analog Devices Inc. All Rights Reserved.
This software is PROPRIETARY & CONFIDENTIAL to Analog Devices, Inc.
and its licensors.
******************************************************************************/
#ifndef ADI_3DTOF_IMAGE_STITCHING_OUTPUT_INFO_H
#define ADI_3DTOF_IMAGE_STITCHING_OUTPUT_INFO_H

#ifdef ENABLE_GPU_OPTIMIZATION
#include <cuda.h>
#include <cuda_runtime.h>
#endif  //! ENABLE_GPU_OPTIMIZATION
#include <cstring>
#include <rclcpp/rclcpp.hpp>
/**
 * @brief This is the class for Image Stitch output info
 *
 */
class ADI3DToFImageStitchingOutputInfo
{
public:
  /**
   * @brief Constructor
   *
   * @param image_width - Image Width
   * @param image_height - Image Height
   */
  ADI3DToFImageStitchingOutputInfo(int image_width, int image_height)
  {
    // Create the node.
    frame_counter_ = -1;
    depth_frame_ = nullptr;
    depth_frame_timestamp_ = rclcpp::Clock{}.now();
    ir_frame_ = nullptr;
    ir_frame_timestamp_ = rclcpp::Clock{}.now();
    xyz_frame_ = nullptr;
    lut_3d_to_2d_mapping_ = nullptr;
    image_width_ = image_width;
    image_height_ = image_height;

#ifdef ENABLE_GPU_OPTIMIZATION
    // Allocate in Pinned memory
    cudaMallocHost(
      (unsigned short **)&depth_frame_, sizeof(unsigned short) * image_width * image_height);
    cudaMallocHost(
      (unsigned short **)&ir_frame_, sizeof(unsigned short) * image_width * image_height);
    cudaMallocHost((float **)&xyz_frame_, 3 * sizeof(float) * image_width * image_height);
    cudaMallocHost((int **)&lut_3d_to_2d_mapping_, sizeof(int) * image_width * image_height);
#else   //! ENABLE_GPU_OPTIMIZATION
    depth_frame_ = new unsigned short[image_width * image_height];
    ir_frame_ = new unsigned short[image_width * image_height];
    xyz_frame_ = new float[3 * image_width * image_height];
    lut_3d_to_2d_mapping_ = new int[image_width * image_height];
#endif  //! ENABLE_GPU_OPTIMIZATION
  }

  /**
   * @brief Destructor
   */
  ~ADI3DToFImageStitchingOutputInfo()
  {
#ifdef ENABLE_GPU_OPTIMIZATION
    if (depth_frame_ != nullptr) {
      cudaFreeHost(depth_frame_);
    }
    if (ir_frame_ != nullptr) {
      cudaFreeHost(ir_frame_);
    }
    if (xyz_frame_ != nullptr) {
      cudaFreeHost(xyz_frame_);
    }
    if (lut_3d_to_2d_mapping_ != nullptr) {
      cudaFreeHost(lut_3d_to_2d_mapping_);
    }
#else   //! ENABLE_GPU_OPTIMIZATION
    if (depth_frame_ != nullptr) {
      delete[] depth_frame_;
    }
    if (ir_frame_ != nullptr) {
      delete[] ir_frame_;
    }
    if (xyz_frame_ != nullptr) {
      delete[] xyz_frame_;
    }
    if (lut_3d_to_2d_mapping_ != nullptr) {
      delete[] lut_3d_to_2d_mapping_;
    }
#endif  //! ENABLE_GPU_OPTIMIZATION
  }

  /**
   * @brief Assignment operator
   * 
   * @param rhs 
   * @return ADI3DToFImageStitchingOutputInfo& 
   */
  ADI3DToFImageStitchingOutputInfo & operator=(const ADI3DToFImageStitchingOutputInfo & rhs)
  {
    frame_counter_ = rhs.frame_counter_;
    memcpy(depth_frame_, rhs.depth_frame_, sizeof(depth_frame_[0]) * image_width_ * image_height_);
    memcpy(ir_frame_, rhs.ir_frame_, sizeof(ir_frame_[0]) * image_width_ * image_height_);
    memcpy(xyz_frame_, rhs.xyz_frame_, sizeof(xyz_frame_[0]) * image_width_ * image_height_ * 3);
    memcpy(
      lut_3d_to_2d_mapping_, rhs.lut_3d_to_2d_mapping_,
      sizeof(lut_3d_to_2d_mapping_[0]) * image_width_ * image_height_);
    return *this;
  }

  int frame_counter_;
  unsigned short * depth_frame_;
  rclcpp::Time depth_frame_timestamp_;
  unsigned short * ir_frame_;
  rclcpp::Time ir_frame_timestamp_;
  float * xyz_frame_;
  int * lut_3d_to_2d_mapping_;
  int image_width_;
  int image_height_;
};

#endif
