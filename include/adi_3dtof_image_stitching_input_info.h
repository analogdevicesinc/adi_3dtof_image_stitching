/******************************************************************************
Copyright (c), 2023 - Analog Devices Inc. All Rights Reserved.
This software is PROPRIETARY & CONFIDENTIAL to Analog Devices, Inc.
and its licensors.
******************************************************************************/
#ifndef ADI_3DTOF_IMAGE_STITCHING_INPUT_INFO_H
#define ADI_3DTOF_IMAGE_STITCHING_INPUT_INFO_H

#ifdef ENABLE_GPU_OPTIMIZATION
#include <cuda.h>
#include <cuda_runtime.h>
#endif  //! ENABLE_GPU_OPTIMIZATION
#include <stdlib.h>

#include <cstring>
//#include <rclcpp/rclcpp.hpp>

/**
 * @brief This is the class for input node for Stitch code
 *
 */
class ADI3DToFImageStitchingInputInfo
{
public:
  static const int MAX_NUM_SENSORS = 4;
  /**
   * @brief Constructor
   *
   * @param image_width - Image Width
   * @param image_height - Image Height
   * @param num_sensors - Number of Sensors
   */
  ADI3DToFImageStitchingInputInfo(int image_width, int image_height, int num_sensors)
  {
    // Create the node.
    image_width_ = image_width;
    image_height_ = image_height;
    num_sensors_ = num_sensors;

#ifdef ENABLE_GPU_OPTIMIZATION
    // Allocate in Pinned memory
    for (int i = 0; i < num_sensors_; i++) {
      cudaMallocHost(
        (unsigned short **)&depth_frame_[i], sizeof(unsigned short) * image_width * image_height);
      cudaMallocHost(
        (unsigned short **)&ir_frame_[i], sizeof(unsigned short) * image_width * image_height);
      depth_frame_timestamp_[i] = 0;
      ir_frame_timestamp_[i] = 0;
    }
#else   //! ENABLE_GPU_OPTIMIZATION
    for (int i = 0; i < num_sensors_; i++) {
      depth_frame_[i] = new unsigned short[image_width * image_height];
      ir_frame_[i] = new unsigned short[image_width * image_height];
      depth_frame_timestamp_[i] = 0;
      ir_frame_timestamp_[i] = 0;
    }
#endif  //! ENABLE_GPU_OPTIMIZATION
  }

  /**
   * @brief Destructor
   */
  ~ADI3DToFImageStitchingInputInfo()
  {
#ifdef ENABLE_GPU_OPTIMIZATION
    for (int i = 0; i < num_sensors_; i++) {
      if (depth_frame_[i] != nullptr) {
        cudaFreeHost(depth_frame_[i]);
      }
      if (ir_frame_[i] != nullptr) {
        cudaFreeHost(ir_frame_[i]);
      }
    }
#else   //! ENABLE_GPU_OPTIMIZATION
    for (int i = 0; i < num_sensors_; i++) {
      if (depth_frame_[i] != nullptr) {
        delete[] depth_frame_[i];
      }
      if (ir_frame_[i] != nullptr) {
        delete[] ir_frame_[i];
      }
    }
#endif  //! ENABLE_GPU_OPTIMIZATION
  }

  unsigned short * getDepthFrame(int cam_id) const { return depth_frame_[cam_id]; }

  unsigned short * getIRFrame(int cam_id) const { return ir_frame_[cam_id]; }

  int64_t getDepthFrameTimestamp(int cam_id) const { return depth_frame_timestamp_[cam_id]; }

  int64_t getIRFrameTimestamp(int cam_id) const { return ir_frame_timestamp_[cam_id]; }

  void setDepthFrameTimestamp(int64_t timeDepth, int cam_id)
  {
    depth_frame_timestamp_[cam_id] = timeDepth;
  }

  void setIRFrameTimestamp(int64_t timeIR, int cam_id) { ir_frame_timestamp_[cam_id] = timeIR; }

  float * getTransformMatrix(int cam_id) { return transform_matrix_[cam_id]; }

  int getNumSensors() { return num_sensors_; }

  // Assignment operator
  ADI3DToFImageStitchingInputInfo & operator=(const ADI3DToFImageStitchingInputInfo & rhs)
  {
    num_sensors_ = rhs.num_sensors_;
    for (int i = 0; i < num_sensors_; i++) {
      memcpy(
        depth_frame_[i], rhs.depth_frame_[i],
        sizeof(depth_frame_[i][0]) * image_width_ * image_height_);
      memcpy(
        ir_frame_[i], rhs.ir_frame_[i], sizeof(ir_frame_[i][0]) * image_width_ * image_height_);
      memcpy(transform_matrix_[i], rhs.transform_matrix_[i], sizeof(transform_matrix_[i][0]) * 16);
    }
    return *this;
  }

private:
  /**
   * @brief Depth image
   */
  unsigned short * depth_frame_[MAX_NUM_SENSORS];
  /**
   * @brief Depth image
   */
  int64_t depth_frame_timestamp_[MAX_NUM_SENSORS];
  /**
   * @brief IR image
   */
  unsigned short * ir_frame_[MAX_NUM_SENSORS];
  /**
   * @brief IR image
   */
  int64_t ir_frame_timestamp_[MAX_NUM_SENSORS];
  /**
   * @brief Image width
   */
  int image_width_;
  /**
   * @brief Image height
   */
  int image_height_;

  int num_sensors_;

  float transform_matrix_[MAX_NUM_SENSORS][16];  // a 4x4 matrix for all the sensors
};

#endif
