/******************************************************************************
Copyright (c), 2023 - Analog Devices Inc. All Rights Reserved.
This software is PROPRIETARY & CONFIDENTIAL to Analog Devices, Inc.
and its licensors.
******************************************************************************/
#ifndef STITCH_FRAMES_CORE_GPU_CUH
#define STITCH_FRAMES_CORE_GPU_CUH
#include "adi_3dtof_image_stitching_input_info.h"
#include <cuda.h>
#include <cuda_runtime.h>

/**
 * @brief Class for stitch frames core GPU Implementation
 * 
 */
class StitchFramesCoreGPU
{
public:
  StitchFramesCoreGPU(int sensor_image_width, int sensor_image_height, int out_image_width, int out_image_height,
                      int num_sensors, float vertical_fov_in_degrees, float horizontal_fov_in_degrees);

  ~StitchFramesCoreGPU();

  void copyRange2XYZLUT(float* range_2_xyz_lut, int cam_id);

  void update_parameters(int out_image_width, int out_image_height, float horizontal_fov_in_degrees, float yaw_correction_in_radians);

  void stitchFrames(ADI3DToFImageStitchingInputInfo* image_stitch_input_info, int num_sensors, float* out_xyz_frame,
                    unsigned short* out_depth_frame, unsigned short* out_ir_frame, int* out_lut_3d_to_2d_mapping);

private:
  int sensor_image_width_;
  int sensor_image_height_;
  int out_image_width_;
  int out_image_height_;
  int num_sensors_;
  float vertical_fov_in_degrees_;
  float horizontal_fov_in_degrees_;
  float yaw_correction_in_radians_;

  // LUT for the Point cloud(512x512)
  float* range_to_xyz_lut_[4];
  float* range_to_xyz_lut_host_[4];
  unsigned short* depth_image_;
  unsigned short* ir_image_;
  int* hit_count_;

  // Out Point Cloud buffers(3x2048x512)
  float* combined_point_cloud_xyz_;
  float* combined_point_cloud_xyz_out_;
  // Out Depth and IR images(2048x512)
  unsigned short* combined_depth_image_;
  unsigned short* combined_ir_image_;

  // Transform (4x4)
  float* transform_;

  float* lut_3d_to_2d_range_;
  int* lut_3d_to_2d_u_;
  int* lut_3d_to_2d_v_;

  int* valid_projection_boundary_x_min_;
  int* valid_projection_boundary_x_max_;
  int* lut_3d_to_2d_mapping_;
  float* range_out_after_projection_;
  unsigned short* depth_out_before_post_process_;
  unsigned short* ir_out_before_post_process_;

  static const int num_cuda_streams_ = 8;
  cudaStream_t cuda_streams_[num_cuda_streams_];
};
#endif
