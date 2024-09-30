/******************************************************************************
Copyright (c), 2023 - Analog Devices Inc. All Rights Reserved.
This software is PROPRIETARY & CONFIDENTIAL to Analog Devices, Inc.
and its licensors.
******************************************************************************/

#include "stitch_frames_core_gpu.cuh"

#include "module_profile.h"

#include <cuda.h>
#include <cuda_runtime.h>
#include <algorithm>
#include <stdio.h>
#include <stdlib.h>

#define MAX_POSTPROCESSING_ITERATIONS 10
#define MIN_HIT_COUNT 50

/**
 * @brief Function for preprocess stage 1 where we generate point cloud and create our combined object
 * 
 * @param depth - input depth frame
 * @param ir - input IR frame
 * @param range_2_xyz_lut - range to xyz LUT
 * @param transform - camera transform
 * @param combo_depth - combined depth output
 * @param combo_ir - combined IR output
 * @param combo_pc_xyz - combined XYZ output
 * @param lut_3d_to_2d_range - 3D to 2D projection LUT
 * @param lut_3d_to_2d_u - pixl position i for projected points
 * @param lut_3d_to_2d_v - pixl position j for projected points
 * @param lut_3d_to_2d_mapping - pixel mapping output
 * @param range_out_after_projection - range out frame after projection
 * @param depth_out_before_post_process - depth output before post processing
 * @param ir_out_before_post_process - IR output before post processing
 * @param invalid_pixel_value_float - invalid pixel default value float
 * @param invalid_pixel_value - invalid pixel default value INT16
 * @param fov_vertical_rad - Vertical field of view for projection in radians
 * @param fov_horizontal_rad - Horizontal field of view for projection in radians
 * @param out_image_width - output image width
 * @param out_image_height - output image height
 * @param offset - index offset to correctly locate pixel in GPU run
 * @return __global__ 
 */
__global__ void preprocess(unsigned short* depth, unsigned short* ir, float* range_2_xyz_lut, float* transform,
                           unsigned short* combo_depth, unsigned short* combo_ir, float* combo_pc_xyz,
                           float* lut_3d_to_2d_range, int* lut_3d_to_2d_u, int* lut_3d_to_2d_v,
                           int* lut_3d_to_2d_mapping, float* range_out_after_projection,
                           unsigned short* depth_out_before_post_process, unsigned short* ir_out_before_post_process,
                           float invalid_pixel_value_float, unsigned short invalid_pixel_value, float fov_vertical_rad,
                           float fov_horizontal_rad, int out_image_width, int out_image_height, float yaw_correction_in_radians, int offset)
{
  int index = threadIdx.x + blockIdx.x * blockDim.x;
  index += offset;

  float range_in_mtr;
  float pc_x, pc_y, pc_z;
  float pc_xt, pc_yt, pc_zt;
  float fov_vertical_by_2_rad = fov_vertical_rad / 2.0;
  if (depth[index] == 0)
  {
    range_in_mtr = 8000.0f;  // 8mtr
  }
  else
  {
    range_in_mtr = (float)depth[index];
  }
  pc_x = (int(range_in_mtr * range_2_xyz_lut[3 * index])) / 1000.0f;
  pc_y = (int(range_in_mtr * range_2_xyz_lut[3 * index + 1])) / 1000.0f;
  pc_z = (int(range_in_mtr * range_2_xyz_lut[3 * index + 2])) / 1000.0f;

  // Transform
  pc_xt = (transform[0] * pc_x + transform[1] * pc_y + transform[2] * pc_z + transform[3]);
  pc_yt =
      (transform[1 * 4 + 0] * pc_x + transform[1 * 4 + 1] * pc_y + transform[1 * 4 + 2] * pc_z + transform[1 * 4 + 3]);
  pc_zt =
      (transform[2 * 4 + 0] * pc_x + transform[2 * 4 + 1] * pc_y + transform[2 * 4 + 2] * pc_z + transform[2 * 4 + 3]);

  // Populate combo output
  combo_depth[index] = depth[index];
  combo_ir[index] = ir[index];
  combo_pc_xyz[3 * index] = pc_xt;
  combo_pc_xyz[3 * index + 1] = pc_yt;
  combo_pc_xyz[3 * index + 2] = pc_zt;

  float pc_yt_mod = pc_zt - transform[2 * 4 + 3];
  float pc_zt_mod = -pc_yt;

  // First part of the Projection
  // Init buffers
  lut_3d_to_2d_mapping[index] = -1;
  range_out_after_projection[index] = invalid_pixel_value_float;
  depth_out_before_post_process[index] = invalid_pixel_value;
  ir_out_before_post_process[index] = invalid_pixel_value;

  float x = pc_xt;
  float y = pc_yt_mod;
  float z = pc_zt_mod;

  int pixel_v = 0;
  int pixel_u = 0;
  float range = 0.0f;

  range = sqrt(x * x + y * y + z * z);
  auto yaw = atan2(z, x);
  auto pitch = asin(y / range);
  float v = (yaw + M_PIf32 - yaw_correction_in_radians)/fov_horizontal_rad ; // pi - yaw correction defines the starting point of horizontal projection, 
                                                                             //while fov_horizontal_rad defines the horizontal FOV for projection.
  float u = 1.0f - (pitch + fov_vertical_by_2_rad) / fov_vertical_rad;
  v *= out_image_width;
  u *= out_image_height;

  pixel_v = floor(v);
  pixel_v = min(out_image_width - 1, pixel_v);
  pixel_v = max(0, pixel_v);

  pixel_u = floor(u);
  pixel_u = min(out_image_height - 1, pixel_u);
  pixel_u = max(0, pixel_u);

  lut_3d_to_2d_range[index] = range;
  lut_3d_to_2d_u[index] = pixel_u;
  lut_3d_to_2d_v[index] = pixel_v;
}

/**
 * @brief Stage 1 of postprocessing to ensure background pixels dont overlap on foreground pixels after projection
 * 
 * @param lut_3d_to_2d_range - 3D to 2D LUT
 * @param lut_3d_to_2d_u - pixl position i for projected points
 * @param lut_3d_to_2d_v - pixel position j for projected points
 * @param range_out_after_projection - range output after projection
 * @param lut_3d_to_2d_mapping - 3D to 2D LUT
 * @param valid_projection_boundary_x_min - starting index per row for valid projection points
 * @param valid_projection_boundary_x_max - ending index per row for valid projection points
 * @param combined_depth_image - combined depth image output
 * @param combined_ir_image - combined IR image output
 * @param depth_out_before_post_process - depth image out before post process
 * @param ir_out_before_post_process - IR image out before post process
 * @param hit_count - counter to count the number of background pixels still occluding foreground pixels after each iteration of check
 * @param out_image_size - output image size
 * @param out_img_width - output image width
 * @return __global__ 
 */
__global__ void postProcessStage1(float* lut_3d_to_2d_range, int* lut_3d_to_2d_u, int* lut_3d_to_2d_v,
                                  float* range_out_after_projection, int* lut_3d_to_2d_mapping,
                                  int* valid_projection_boundary_x_min, int* valid_projection_boundary_x_max,
                                  unsigned short* combined_depth_image, unsigned short* combined_ir_image,
                                  unsigned short* depth_out_before_post_process,
                                  unsigned short* ir_out_before_post_process, int* hit_count, int out_image_size,
                                  int out_img_width)
{
  int index = threadIdx.x + blockIdx.x * blockDim.x;

  int pixel_u = lut_3d_to_2d_u[index];
  int pixel_v = lut_3d_to_2d_v[index];
  float range = lut_3d_to_2d_range[index];

  // Check to ensure we always project the point nearest to camera
  //(default initialised range value is invalid_pixel_value, which is higher
  // than values possible)
  if (range < range_out_after_projection[pixel_u * out_img_width + pixel_v])
  {
    hit_count[0]++;
    range_out_after_projection[pixel_u * out_img_width + pixel_v] = range;
    valid_projection_boundary_x_min[pixel_u] = min(pixel_v, valid_projection_boundary_x_min[pixel_u]);
    valid_projection_boundary_x_max[pixel_u] = max(pixel_v, valid_projection_boundary_x_max[pixel_u]);

    depth_out_before_post_process[pixel_u * out_img_width + pixel_v] = combined_depth_image[index];
    ir_out_before_post_process[pixel_u * out_img_width + pixel_v] = combined_ir_image[index];
    lut_3d_to_2d_mapping[pixel_u * out_img_width + pixel_v] = index;
  }
}

/**
 * @brief Function for final stage of postprocessing to generate stitched output
 * 
 * @param depth_out_before_post_process - depth input before post processing
 * @param ir_out_before_post_process - IR input before post processing
 * @param valid_projection_boundary_x_min - starting index per row for valid projection points
 * @param valid_projection_boundary_x_max - ending index per row for valid projection points
 * @param invalid_pixel_value - invalid default pixel value INT16
 * @param lut_3d_to_2d_mapping - 3D to 2D LUT
 * @param combined_point_cloud_xyz - combined XYZ frame
 * @param combined_point_cloud_xyz_out - combined XYZ output for publishing
 * @param combined_depth_image - final stitched depth output
 * @param combined_ir_image - final stitched IR output
 * @param out_image_width - output image width
 * @param out_image_height - output image height
 * @param offset - offset to access pixel in GPU run
 * @return __global__ 
 */
__global__ void postProcessStage2(unsigned short* depth_out_before_post_process,
                                  unsigned short* ir_out_before_post_process, int* valid_projection_boundary_x_min,
                                  int* valid_projection_boundary_x_max, unsigned short invalid_pixel_value,
                                  int* lut_3d_to_2d_mapping, float* combined_point_cloud_xyz,
                                  float* combined_point_cloud_xyz_out, unsigned short* combined_depth_image,
                                  unsigned short* combined_ir_image, int out_image_width, int out_image_height,
                                  int offset)
{
  int index = threadIdx.x + blockIdx.x * blockDim.x;
  index += offset;

  int i = index / out_image_width;
  int j = index % out_image_width;

  // Init
  combined_ir_image[index] = 0;
  combined_depth_image[index] = 0;

  // Mark invalid point-cloud
  if (lut_3d_to_2d_mapping[index] == -1 || depth_out_before_post_process[index] == 0)
  {
    combined_point_cloud_xyz_out[3 * index] = 0.0f;
    combined_point_cloud_xyz_out[3 * index + 1] = 0.0f;
    combined_point_cloud_xyz_out[3 * index + 2] = 0.0f;
  }
  else
  {
    int mapped_index = lut_3d_to_2d_mapping[index];
    combined_point_cloud_xyz_out[3 * index] = combined_point_cloud_xyz[3 * mapped_index];
    combined_point_cloud_xyz_out[3 * index + 1] = combined_point_cloud_xyz[3 * mapped_index + 1];
    combined_point_cloud_xyz_out[3 * index + 2] = combined_point_cloud_xyz[3 * mapped_index + 2];
  }

  // for (i = 1; i < out_image_height - 1; ++i)
  // for (j = 1; j < out_image_width - 1; ++j)
  if ((i >= 1) && (i < (out_image_height - 1)) && (j >= 1) && (j < (out_image_width - 1)))
  {
    unsigned short ir_org = ir_out_before_post_process[index];
    unsigned short depth_org = depth_out_before_post_process[index];
    if (ir_org == invalid_pixel_value)
    {
      if ((j < valid_projection_boundary_x_min[i]) || (j > valid_projection_boundary_x_max[i]))
      {
        combined_ir_image[index] = 0;
        combined_depth_image[index] = 0;
      }
      else
      {
        unsigned int sum_ir = 0;
        unsigned int sum_depth = 0;
        int add_count = 0;
        for (int r = -1; r <= 1; r++)
        {
          for (int c = -1; c <= 1; c++)
          {
            if (ir_out_before_post_process[((i - r) * out_image_width) + j + c] != invalid_pixel_value)
            {
              sum_ir += ir_out_before_post_process[((i - r) * out_image_width) + j + c];
              sum_depth += depth_out_before_post_process[((i - r) * out_image_width) + j + c];
              add_count++;
            }
          }
        }

        if (add_count > 0)
        {
          combined_ir_image[index] = (unsigned short)(sum_ir / add_count);
          combined_depth_image[index] = (unsigned short)(sum_depth / add_count);
        }
        else
        {
          combined_ir_image[index] = 0;
          combined_depth_image[index] = 0;
        }
      }
    }
    else
    {
      combined_ir_image[index] = ir_org;
      combined_depth_image[index] = depth_org;
    }
  }
}

/**
 * @brief Construct a new Stitch Frames Core G P U:: Stitch Frames Core G P U object
 * 
 * @param sensor_image_width - input image width
 * @param sensor_image_height - input image height
 * @param out_image_width - output stitched image width
 * @param out_image_height - output stitched image height
 * @param num_sensors - number of sensors
 * @param vertical_fov_in_degrees - Vertical FOV for projection in degrees
 * @param horizontal_fov_in_degrees - Horizontal FOV for projection in degrees
 */
StitchFramesCoreGPU::StitchFramesCoreGPU(int sensor_image_width, int sensor_image_height, int out_image_width, int out_image_height,
                      int num_sensors, float vertical_fov_in_degrees, float horizontal_fov_in_degrees)
    : sensor_image_width_(sensor_image_width)
    , sensor_image_height_(sensor_image_height)
    , out_image_width_(out_image_width)
    , out_image_height_(out_image_height)
    , num_sensors_(num_sensors)
    , vertical_fov_in_degrees_(vertical_fov_in_degrees)
    , horizontal_fov_in_degrees_(horizontal_fov_in_degrees)
{
  // Allocate memory
  // Device Memory
  int out_image_size = out_image_width_ * out_image_height_;
  int in_image_size = sensor_image_width_ * sensor_image_height_;
  yaw_correction_in_radians_ = 0.0f;

  // LUT
  for (int i = 0; i < num_sensors_; i++)
  {
    range_to_xyz_lut_host_[i] = (float*)malloc(3 * in_image_size * sizeof(float));
    cudaMalloc((float**)&range_to_xyz_lut_[i], 3 * in_image_size * sizeof(float));
  }

  cudaMalloc((int**)&hit_count_, sizeof(hit_count_[0]));

  // Depth and IR for individual sensors
  cudaMalloc((unsigned short**)&depth_image_, in_image_size * sizeof(depth_image_[0]));
  cudaMalloc((unsigned short**)&ir_image_, in_image_size * sizeof(ir_image_[0]));

  // Depth and IR for combined image
  cudaMalloc((unsigned short**)&combined_depth_image_, out_image_size * sizeof(combined_depth_image_[0]));
  cudaMalloc((unsigned short**)&combined_ir_image_, out_image_size * sizeof(combined_ir_image_[0]));
  // Point Cloud for combined image
  cudaMalloc((float**)&combined_point_cloud_xyz_, 3 * out_image_size * sizeof(combined_point_cloud_xyz_[0]));
  cudaMalloc((float**)&combined_point_cloud_xyz_out_, 3 * out_image_size * sizeof(combined_point_cloud_xyz_out_[0]));

  // Transform Matrix
  cudaMalloc((float**)&transform_, 16 * sizeof(transform_[0]));

  cudaMalloc((float**)&lut_3d_to_2d_range_, out_image_size * sizeof(lut_3d_to_2d_range_[0]));
  cudaMalloc((int**)&lut_3d_to_2d_u_, out_image_size * sizeof(lut_3d_to_2d_u_[0]));
  cudaMalloc((int**)&lut_3d_to_2d_v_, out_image_size * sizeof(lut_3d_to_2d_v_[0]));

  cudaMalloc((int**)&lut_3d_to_2d_mapping_, out_image_size * sizeof(lut_3d_to_2d_mapping_[0]));
  cudaMalloc((int**)&valid_projection_boundary_x_min_,
              out_image_height_ * sizeof(valid_projection_boundary_x_min_[0]));
  cudaMalloc((int**)&valid_projection_boundary_x_max_,
              out_image_height_ * sizeof(valid_projection_boundary_x_max_[0]));

  cudaMalloc((float**)&range_out_after_projection_, out_image_size * sizeof(range_out_after_projection_[0]));
  cudaMalloc((unsigned short**)&depth_out_before_post_process_,
              out_image_size * sizeof(depth_out_before_post_process_));
  cudaMalloc((unsigned short**)&ir_out_before_post_process_, out_image_size * sizeof(ir_out_before_post_process_));

  for (int i = 0; i < num_cuda_streams_; i++)
  {
    cudaStreamCreate(&cuda_streams_[i]);
  }
}

/**
 * @brief Updating the image stitching parameters
 * 
 * @param out_image_width - output stitched image width
 * @param out_image_height - output stitched image height
 * @param horizontal_fov_in_degrees - horizontal FOV in degrees
 */
void StitchFramesCoreGPU::update_parameters(int out_image_width, int out_image_height,float horizontal_fov_in_degrees, float yaw_correction_in_radians)
{
  // Allocate memory
  // Device Memory
  out_image_width_ = out_image_width;
  out_image_height_ = out_image_height;
  horizontal_fov_in_degrees_ = horizontal_fov_in_degrees;
  yaw_correction_in_radians_ = yaw_correction_in_radians;
}

/**
 * @brief Destroy the Stitch Frames Core G P U:: Stitch Frames Core G P U object
 * 
 */
StitchFramesCoreGPU::~StitchFramesCoreGPU()
{
  for (int i = 0; i < 4; i++)
  {
    cudaFree(range_to_xyz_lut_[i]);
    free(range_to_xyz_lut_host_[i]);
  }
  cudaFree(depth_image_);
  cudaFree(ir_image_);
  cudaFree(combined_depth_image_);
  cudaFree(combined_ir_image_);
  cudaFree(combined_point_cloud_xyz_);
  cudaFree(combined_point_cloud_xyz_out_);

  cudaFree(lut_3d_to_2d_range_);
  cudaFree(lut_3d_to_2d_u_);
  cudaFree(lut_3d_to_2d_v_);

  cudaFree(valid_projection_boundary_x_min_);
  cudaFree(valid_projection_boundary_x_max_);
  cudaFree(lut_3d_to_2d_mapping_);
  cudaFree(range_out_after_projection_);
  cudaFree(depth_out_before_post_process_);
  cudaFree(ir_out_before_post_process_);

  for (int i = 0; i < num_cuda_streams_; i++)
  {
    cudaStreamDestroy(cuda_streams_[i]);
  }
}

/**
 * @brief Function to copy range data to XYZ lut
 * 
 * @param range_2_xyz_lut - range to lut table
 * @param cam_id - camera id
 */
void StitchFramesCoreGPU::copyRange2XYZLUT(float* range_2_xyz_lut, int cam_id)
{
  // Host to Device Copy
  int in_image_size = sensor_image_width_ * sensor_image_height_;
  cudaMemcpy(range_to_xyz_lut_[cam_id], range_2_xyz_lut, 3 * in_image_size * sizeof(float), cudaMemcpyHostToDevice);
  memcpy(range_to_xyz_lut_host_[cam_id], range_2_xyz_lut, 3 * in_image_size * sizeof(float));
}

/**
 * @brief Core function to initiate image stitching
 * 
 * @param image_stitching_input_info - input object to image stitching input 
 * @param num_sensors - number of sensors
 * @param out_xyz_frame - output XYZ frame
 * @param out_depth_frame - output stitched depth
 * @param out_ir_frame - output stitched IR
 */
void StitchFramesCoreGPU::stitchFrames(ADI3DToFImageStitchingInputInfo* image_stitching_input_info, int num_sensors,
                                       float* out_xyz_frame, unsigned short* out_depth_frame,
                                       unsigned short* out_ir_frame, int* out_lut_3d_to_2d_mapping)
{
  int out_image_size = out_image_width_ * out_image_height_;
  int in_image_size = (sensor_image_width_ * sensor_image_height_);
  int threads_per_block = 512;
  unsigned short invalid_pixel_value = 0xFFFF;
  float invalid_pixel_value_float = 9999.0f;
  float fov_vertical_rad = (vertical_fov_in_degrees_ / 180) * M_PI;
  float fov_horizontal_rad = (horizontal_fov_in_degrees_ / 180) * M_PI;

  PROFILE_FUNCTION_START(StitchFramesCoreGPU_stitchFrames)
  PROFILE_FUNCTION_START(StitchFramesCoreGPU_preprocess)
  for (int i = 0; i < num_sensors; i++)
  {
    // Copy Depth, IR and Transformed pointcloud to the combo buffer
    int start_ind_for_sensor = i * in_image_size;
    int num_blocks_in = in_image_size / threads_per_block;
    int in_elements_per_stream = in_image_size / num_cuda_streams_;

    unsigned short* depth_image = image_stitching_input_info->getDepthFrame(i);
    unsigned short* ir_image = image_stitching_input_info->getIRFrame(i);
    float* transform_array = image_stitching_input_info->getTransformMatrix(i);

    cudaMemcpy(transform_, transform_array, 16 * sizeof(transform_[0]), cudaMemcpyHostToDevice);
    int offset = 0;
    for (int s = 0; s < num_cuda_streams_; s++)
    {
      offset = s * in_elements_per_stream;

      cudaMemcpyAsync(&depth_image_[offset], &depth_image[offset], in_elements_per_stream * sizeof(depth_image_[0]),
                      cudaMemcpyHostToDevice, cuda_streams_[i]);
      cudaMemcpyAsync(&ir_image_[offset], &ir_image[offset], in_elements_per_stream * sizeof(ir_image_[0]),
                      cudaMemcpyHostToDevice, cuda_streams_[i]);

      preprocess<<<num_blocks_in / num_cuda_streams_, threads_per_block, 0, cuda_streams_[i]>>>(
          depth_image_, ir_image_, range_to_xyz_lut_[i], transform_, &combined_depth_image_[start_ind_for_sensor],
          &combined_ir_image_[start_ind_for_sensor], &combined_point_cloud_xyz_[3 * start_ind_for_sensor],
          &lut_3d_to_2d_range_[start_ind_for_sensor], &lut_3d_to_2d_u_[start_ind_for_sensor],
          &lut_3d_to_2d_v_[start_ind_for_sensor], &lut_3d_to_2d_mapping_[start_ind_for_sensor],
          &range_out_after_projection_[start_ind_for_sensor], &depth_out_before_post_process_[start_ind_for_sensor],
          &ir_out_before_post_process_[start_ind_for_sensor], invalid_pixel_value_float, invalid_pixel_value, fov_vertical_rad,
          fov_horizontal_rad, out_image_width_, out_image_height_, yaw_correction_in_radians_, offset);
    }
  }
  for (int i = 0; i < num_cuda_streams_; i++)
  {
    cudaStreamSynchronize(cuda_streams_[i]);
  }
  // Wait for the GPU to finish
  cudaDeviceSynchronize();
  PROFILE_FUNCTION_END(StitchFramesCoreGPU_preprocess)

  PROFILE_FUNCTION_START(StitchFramesCoreGPU_postProcessStage1)
  // Post Process : Stage-1
  int num_blocks_out = out_image_size / threads_per_block;
  for (int itr = 0; itr < MAX_POSTPROCESSING_ITERATIONS; itr++)
  {
    int hit_count = 0;
    cudaMemcpy(hit_count_, &hit_count, sizeof(hit_count_[0]), cudaMemcpyHostToDevice);
    postProcessStage1<<<num_blocks_out, threads_per_block>>>(
        lut_3d_to_2d_range_, lut_3d_to_2d_u_, lut_3d_to_2d_v_, range_out_after_projection_, lut_3d_to_2d_mapping_,
        valid_projection_boundary_x_min_, valid_projection_boundary_x_max_, combined_depth_image_, combined_ir_image_,
        depth_out_before_post_process_, ir_out_before_post_process_, hit_count_, out_image_size, out_image_width_);
    cudaDeviceSynchronize();
    cudaMemcpy(&hit_count, hit_count_, sizeof(hit_count_[0]), cudaMemcpyDeviceToHost);

    if (hit_count < MIN_HIT_COUNT || itr == MAX_POSTPROCESSING_ITERATIONS - 1)
    {
      break;
    }
  }

  // Wait for the GPU to finish
  // cudaDeviceSynchronize();
  PROFILE_FUNCTION_END(StitchFramesCoreGPU_postProcessStage1)

  PROFILE_FUNCTION_START(StitchFramesCoreGPU_postProcessStage2)
  // Post Process : Stage-2
  int elements_per_stream = out_image_size / num_cuda_streams_;
  int offset = 0;
  for (int i = 0; i < num_cuda_streams_; i++)
  {
    offset = i * elements_per_stream;

    postProcessStage2<<<num_blocks_out / num_cuda_streams_, threads_per_block, 0, cuda_streams_[i]>>>(
        depth_out_before_post_process_, ir_out_before_post_process_, valid_projection_boundary_x_min_,
        valid_projection_boundary_x_max_, invalid_pixel_value, lut_3d_to_2d_mapping_, combined_point_cloud_xyz_,
        combined_point_cloud_xyz_out_, combined_depth_image_, combined_ir_image_, out_image_width_, out_image_height_,
        offset);

    cudaMemcpyAsync(&out_depth_frame[offset], &combined_depth_image_[offset],
                    elements_per_stream * sizeof(unsigned short), cudaMemcpyDeviceToHost, cuda_streams_[i]);
    cudaMemcpyAsync(&out_ir_frame[offset], &combined_ir_image_[offset], elements_per_stream * sizeof(unsigned short),
                    cudaMemcpyDeviceToHost, cuda_streams_[i]);
    cudaMemcpyAsync(&out_xyz_frame[3 * offset], &combined_point_cloud_xyz_out_[3 * offset],
                    3 * elements_per_stream * sizeof(float), cudaMemcpyDeviceToHost, cuda_streams_[i]);
    cudaMemcpyAsync(&out_lut_3d_to_2d_mapping[offset], &lut_3d_to_2d_mapping_[offset],
                    elements_per_stream * sizeof(int), cudaMemcpyDeviceToHost, cuda_streams_[i]);
  }

  // Wait for the GPU to finish
  for (int i = 0; i < num_cuda_streams_; i++)
  {
    cudaStreamSynchronize(cuda_streams_[i]);
  }
  cudaDeviceSynchronize();
  PROFILE_FUNCTION_END(StitchFramesCoreGPU_postProcessStage2)

  PROFILE_FUNCTION_END(StitchFramesCoreGPU_stitchFrames)
}

//#define STANDALONE_TEST_STUB
#ifdef STANDALONE_TEST_STUB

/**
 * @brief main function for Stand along test code
 * 
 * @return int 
 */
int main(void)
{
  int num_sensors = 4;
  StitchFramesCoreGPU* stitch_frames_gpu = new StitchFramesCoreGPU(512, 512, 2048, 512, num_sensors, 75.0f);
  ADI3DToFImageStitchingInputInfo* image_stitching_input_info = new ADI3DToFImageStitchingInputInfo(512, 512, num_sensors);
  unsigned short *depth_frame_out, *ir_frame_out;
  float* xyz_frame_out;

  cudaMallocHost((unsigned short**)&depth_frame_out, sizeof(unsigned short) * 2048 * 512);
  cudaMallocHost((unsigned short**)&ir_frame_out, sizeof(unsigned short) * 2048 * 512);
  cudaMallocHost((float**)&xyz_frame_out, 3 * sizeof(float) * 2048 * 512);

  for (int i = 0; i < 1; i++)
  {
    // populate input buffers
    for (int s = 0; s < num_sensors; s++)
    {
      unsigned short* depth_frame = image_stitching_input_info->getDepthFrame(s);
      unsigned short* ir_frame = image_stitching_input_info->getIRFrame(s);
      for (int p = 0; p < 512 * 512; p++)
      {
        *depth_frame++ = rand() % 8000;
        *ir_frame++ = rand() % 8000;
      }
    }

    stitch_frames_gpu->stitchFrames(image_stitching_input_info, num_sensors, xyz_frame_out, depth_frame_out, ir_frame_out);
  }

  cudaFreeHost(depth_frame_out);
  cudaFreeHost(ir_frame_out);
  cudaFreeHost(xyz_frame_out);
}

#endif
