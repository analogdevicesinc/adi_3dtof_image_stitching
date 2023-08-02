/******************************************************************************
Copyright (c), 2023 - Analog Devices Inc. All Rights Reserved.
This software is PROPRIETARY & CONFIDENTIAL to Analog Devices, Inc.
and its licensors.
******************************************************************************/
#include "adi_3dtof_image_stitching.h"
#include <ros/ros.h>
#include <tf2_eigen/tf2_eigen.h>
#include <compressed_depth_image_transport/rvl_codec.h>
#include "module_profile.h"

#ifdef ENABLE_OPENMP_OPTIMIZATION
#include <omp.h>
#endif

namespace enc = sensor_msgs::image_encodings;

/**
 * @brief Call back for synchronised topics(depth and IR image pair) for 2
 * camera configuration
 *
 * @param depth_image_cam1 - Cam1 depth image pointer
 * @param ir_image_cam1 - Cam1 IR image pointer
 * @param depth_image_cam2  - Cam2 depth image pointer
 * @param ir_image_cam2  - Cam2 IR image pointer
 */
void ADI3DToFImageStitching::sync2CamerasDepthIRImageCallback(const sensor_msgs::ImageConstPtr& depth_image_cam1,
                                                      const sensor_msgs::ImageConstPtr& ir_image_cam1,
                                                      const sensor_msgs::ImageConstPtr& depth_image_cam2,
                                                      const sensor_msgs::ImageConstPtr& ir_image_cam2)
{
  PROFILE_FUNCTION_START(Message_Callback_processing);
  // Allocate a node
  ADI3DToFImageStitchingInputInfo* image_stitch_input_info =
      new ADI3DToFImageStitchingInputInfo(sensor_image_width_, sensor_image_height_, 2);

  // Call respective callbacks with the id.
  irImageCallback(ir_image_cam1, 0, image_stitch_input_info);
  depthImageCallback(depth_image_cam1, 0, image_stitch_input_info);

  irImageCallback(ir_image_cam2, 1, image_stitch_input_info);
  depthImageCallback(depth_image_cam2, 1, image_stitch_input_info);

  // Add new node to the queue
  addInputNodeToQueue<ADI3DToFImageStitchingInputInfo>(image_stitch_input_info);
  PROFILE_FUNCTION_END(Message_Callback_processing);
}

/**
 * @brief Call back for synchronised topics(depth and IR image pair) for 3
 * camera configuration
 *
 * @param depth_image_cam1 - Cam1 depth image pointer
 * @param ir_image_cam1 - Cam1 IR image pointer
 * @param depth_image_cam2  - Cam2 depth image pointer
 * @param ir_image_cam2  - Cam2 IR image pointer
 * @param depth_image_cam3  - Cam3 depth image pointer
 * @param ir_image_cam3  - Cam3 IR image pointer
 */
void ADI3DToFImageStitching::sync3CamerasDepthIRImageCallback(const sensor_msgs::ImageConstPtr& depth_image_cam1,
                                                      const sensor_msgs::ImageConstPtr& ir_image_cam1,
                                                      const sensor_msgs::ImageConstPtr& depth_image_cam2,
                                                      const sensor_msgs::ImageConstPtr& ir_image_cam2,
                                                      const sensor_msgs::ImageConstPtr& depth_image_cam3,
                                                      const sensor_msgs::ImageConstPtr& ir_image_cam3)
{
  PROFILE_FUNCTION_START(Message_Callback_processing);
  // Allocate a node
  ADI3DToFImageStitchingInputInfo* image_stitch_input_info =
      new ADI3DToFImageStitchingInputInfo(sensor_image_width_, sensor_image_height_, 3);

#ifdef ENABLE_OPENMP_OPTIMIZATION
#pragma omp parallel sections
  {
#pragma omp section
    {
      irImageCallback(ir_image_cam1, 0, image_stitch_input_info);
      depthImageCallback(depth_image_cam1, 0, image_stitch_input_info);
    }

#pragma omp section
    {
      irImageCallback(ir_image_cam2, 1, image_stitch_input_info);
      depthImageCallback(depth_image_cam2, 1, image_stitch_input_info);
    }

#pragma omp section
    {
      irImageCallback(ir_image_cam3, 2, image_stitch_input_info);
      depthImageCallback(depth_image_cam3, 2, image_stitch_input_info);
    }
  }
#else
  // Call respective callbacks with the id.
  irImageCallback(ir_image_cam1, 0, image_stitch_input_info);
  depthImageCallback(depth_image_cam1, 0, image_stitch_input_info);

  irImageCallback(ir_image_cam2, 1, image_stitch_input_info);
  depthImageCallback(depth_image_cam2, 1, image_stitch_input_info);

  irImageCallback(ir_image_cam3, 2, image_stitch_input_info);
  depthImageCallback(depth_image_cam3, 2, image_stitch_input_info);

#endif

  // Add new node to the queue
  addInputNodeToQueue<ADI3DToFImageStitchingInputInfo>(image_stitch_input_info);
  PROFILE_FUNCTION_END(Message_Callback_processing);
}

/**
 * @brief Call back for synchronised topics(depth and IR image pair) for 4
 * camera configuration
 *
 * @param depth_image_cam1 - Cam1 depth image pointer
 * @param ir_image_cam1 - Cam1 IR image pointer
 * @param depth_image_cam2  - Cam2 depth image pointer
 * @param ir_image_cam2  - Cam2 IR image pointer
 * @param depth_image_cam3  - Cam3 depth image pointer
 * @param ir_image_cam3  - Cam3 IR image pointer
 * @param depth_image_cam4  - Cam4 depth image pointer
 * @param ir_image_cam4  - Cam4 IR image pointer
 */
void ADI3DToFImageStitching::sync4CamerasDepthIRImageCallback(
    const sensor_msgs::ImageConstPtr& depth_image_cam1, const sensor_msgs::ImageConstPtr& ir_image_cam1,
    const sensor_msgs::ImageConstPtr& depth_image_cam2, const sensor_msgs::ImageConstPtr& ir_image_cam2,
    const sensor_msgs::ImageConstPtr& depth_image_cam3, const sensor_msgs::ImageConstPtr& ir_image_cam3,
    const sensor_msgs::ImageConstPtr& depth_image_cam4, const sensor_msgs::ImageConstPtr& ir_image_cam4)
{
  PROFILE_FUNCTION_START(Message_Callback_processing);
  // Allocate a node
  ADI3DToFImageStitchingInputInfo* image_stitch_input_info =
      new ADI3DToFImageStitchingInputInfo(sensor_image_width_, sensor_image_height_, 4);

#ifdef ENABLE_OPENMP_OPTIMIZATION
#pragma omp parallel sections
  {
#pragma omp section
    {
      irImageCallback(ir_image_cam1, 0, image_stitch_input_info);
      depthImageCallback(depth_image_cam1, 0, image_stitch_input_info);
    }

#pragma omp section
    {
      irImageCallback(ir_image_cam2, 1, image_stitch_input_info);
      depthImageCallback(depth_image_cam2, 1, image_stitch_input_info);
    }

#pragma omp section
    {
      irImageCallback(ir_image_cam3, 2, image_stitch_input_info);
      depthImageCallback(depth_image_cam3, 2, image_stitch_input_info);
    }

#pragma omp section
    {
      irImageCallback(ir_image_cam4, 3, image_stitch_input_info);
      depthImageCallback(depth_image_cam4, 3, image_stitch_input_info);
    }
  }
#else
  // Call respective callbacks with the id.
  irImageCallback(ir_image_cam1, 0, image_stitch_input_info);
  depthImageCallback(depth_image_cam1, 0, image_stitch_input_info);

  irImageCallback(ir_image_cam2, 1, image_stitch_input_info);
  depthImageCallback(depth_image_cam2, 1, image_stitch_input_info);

  irImageCallback(ir_image_cam3, 2, image_stitch_input_info);
  depthImageCallback(depth_image_cam3, 2, image_stitch_input_info);

  irImageCallback(ir_image_cam4, 3, image_stitch_input_info);
  depthImageCallback(depth_image_cam4, 3, image_stitch_input_info);

#endif

  // Add new node to the queue
  addInputNodeToQueue<ADI3DToFImageStitchingInputInfo>(image_stitch_input_info);
  PROFILE_FUNCTION_END(Message_Callback_processing);
}

/**
 * @brief Call back for synchronised topics(Compressed depth and IR image pair) for 2
 * camera configuration
 *
 * @param compressed_depth_image_cam1 - Cam1 depth image pointer
 * @param compressed_ir_image_cam1 - Cam1 IR image pointer
 * @param compressed_depth_image_cam2  - Cam2 depth image pointer
 * @param compressed_ir_image_cam2  - Cam2 IR image pointer
 */
void ADI3DToFImageStitching::sync2CamerasCompressedDepthIRImageCallback(
    const sensor_msgs::CompressedImageConstPtr& compressed_depth_image_cam1,
    const sensor_msgs::CompressedImageConstPtr& compressed_ir_image_cam1,
    const sensor_msgs::CompressedImageConstPtr& compressed_depth_image_cam2,
    const sensor_msgs::CompressedImageConstPtr& compressed_ir_image_cam2)
{
  PROFILE_FUNCTION_START(Message_Callback_processing);
  // Allocate a node
  ADI3DToFImageStitchingInputInfo* image_stitch_input_info =
      new ADI3DToFImageStitchingInputInfo(sensor_image_width_, sensor_image_height_, 2);

#ifdef ENABLE_OPENMP_OPTIMIZATION
#pragma omp parallel sections
  {
#pragma omp section
    {
      compressedirImageCallback(compressed_ir_image_cam1, 0, image_stitch_input_info);
      compresseddepthImageCallback(compressed_depth_image_cam1, 0, image_stitch_input_info);
    }

#pragma omp section
    {
      compressedirImageCallback(compressed_ir_image_cam2, 1, image_stitch_input_info);
      compresseddepthImageCallback(compressed_depth_image_cam2, 1, image_stitch_input_info);
    }
  }
#else
  // Call respective callbacks with the id.
  compressedirImageCallback(compressed_ir_image_cam1, 0, image_stitch_input_info);
  compresseddepthImageCallback(compressed_depth_image_cam1, 0, image_stitch_input_info);

  compressedirImageCallback(compressed_ir_image_cam2, 1, image_stitch_input_info);
  compresseddepthImageCallback(compressed_depth_image_cam2, 1, image_stitch_input_info);
#endif

  // Add new node to the queue
  addInputNodeToQueue<ADI3DToFImageStitchingInputInfo>(image_stitch_input_info);

  PROFILE_FUNCTION_END(Message_Callback_processing);
}

/**
 * @brief Call back for synchronised topics(Compressed depth and IR image pair) for 3
 * camera configuration
 *
 * @param compressed_depth_image_cam1 - Cam1 depth image pointer
 * @param compressed_ir_image_cam1 - Cam1 IR image pointer
 * @param compressed_depth_image_cam2  - Cam2 depth image pointer
 * @param compressed_ir_image_cam2  - Cam2 IR image pointer
 * @param compressed_depth_image_cam3  - Cam3 depth image pointer
 * @param compressed_ir_image_cam3  - Cam3 IR image pointer
 */
void ADI3DToFImageStitching::sync3CamerasCompressedDepthIRImageCallback(
    const sensor_msgs::CompressedImageConstPtr& compressed_depth_image_cam1,
    const sensor_msgs::CompressedImageConstPtr& compressed_ir_image_cam1,
    const sensor_msgs::CompressedImageConstPtr& compressed_depth_image_cam2,
    const sensor_msgs::CompressedImageConstPtr& compressed_ir_image_cam2,
    const sensor_msgs::CompressedImageConstPtr& compressed_depth_image_cam3,
    const sensor_msgs::CompressedImageConstPtr& compressed_ir_image_cam3)
{
  PROFILE_FUNCTION_START(Message_Callback_processing);
  // Allocate a node
  ADI3DToFImageStitchingInputInfo* image_stitch_input_info =
      new ADI3DToFImageStitchingInputInfo(sensor_image_width_, sensor_image_height_, 3);

#ifdef ENABLE_OPENMP_OPTIMIZATION
#pragma omp parallel sections
  {
#pragma omp section
    {
      compressedirImageCallback(compressed_ir_image_cam1, 0, image_stitch_input_info);
      compresseddepthImageCallback(compressed_depth_image_cam1, 0, image_stitch_input_info);
    }

#pragma omp section
    {
      compressedirImageCallback(compressed_ir_image_cam2, 1, image_stitch_input_info);
      compresseddepthImageCallback(compressed_depth_image_cam2, 1, image_stitch_input_info);
    }

#pragma omp section
    {
      compressedirImageCallback(compressed_ir_image_cam3, 2, image_stitch_input_info);
      compresseddepthImageCallback(compressed_depth_image_cam3, 2, image_stitch_input_info);
    }
  }
#else
  // Call respective callbacks with the id.
  compressedirImageCallback(compressed_ir_image_cam1, 0, image_stitch_input_info);
  compresseddepthImageCallback(compressed_depth_image_cam1, 0, image_stitch_input_info);

  compressedirImageCallback(compressed_ir_image_cam2, 1, image_stitch_input_info);
  compresseddepthImageCallback(compressed_depth_image_cam2, 1, image_stitch_input_info);

  compressedirImageCallback(compressed_ir_image_cam3, 2, image_stitch_input_info);
  compresseddepthImageCallback(compressed_depth_image_cam3, 2, image_stitch_input_info);
#endif

  // Add new node to the queue
  addInputNodeToQueue<ADI3DToFImageStitchingInputInfo>(image_stitch_input_info);
  PROFILE_FUNCTION_END(Message_Callback_processing);
}

/**
 * @brief Call back for synchronised topics(Compressed depth and IR image pair) for 4
 * camera configuration
 *
 * @param compressed_depth_image_cam1 - Cam1 depth image pointer
 * @param compressed_ir_image_cam1 - Cam1 IR image pointer
 * @param compressed_depth_image_cam2  - Cam2 depth image pointer
 * @param compressed_ir_image_cam2  - Cam2 IR image pointer
 * @param compressed_depth_image_cam3  - Cam3 depth image pointer
 * @param compressed_ir_image_cam3  - Cam3 IR image pointer
 * @param compressed_depth_image_cam4  - Cam4 depth image pointer
 * @param compressed_ir_image_cam4  - Cam4 IR image pointer
 */
void ADI3DToFImageStitching::sync4CamerasCompressedDepthIRImageCallback(
    const sensor_msgs::CompressedImageConstPtr& compressed_depth_image_cam1,
    const sensor_msgs::CompressedImageConstPtr& compressed_ir_image_cam1,
    const sensor_msgs::CompressedImageConstPtr& compressed_depth_image_cam2,
    const sensor_msgs::CompressedImageConstPtr& compressed_ir_image_cam2,
    const sensor_msgs::CompressedImageConstPtr& compressed_depth_image_cam3,
    const sensor_msgs::CompressedImageConstPtr& compressed_ir_image_cam3,
    const sensor_msgs::CompressedImageConstPtr& compressed_depth_image_cam4,
    const sensor_msgs::CompressedImageConstPtr& compressed_ir_image_cam4)
{
  PROFILE_FUNCTION_START(Message_Callback_processing);
  // Allocate a node
  ADI3DToFImageStitchingInputInfo* image_stitch_input_info =
      new ADI3DToFImageStitchingInputInfo(sensor_image_width_, sensor_image_height_, 4);

#ifdef ENABLE_OPENMP_OPTIMIZATION
#pragma omp parallel sections
  {
#pragma omp section
    {
      compressedirImageCallback(compressed_ir_image_cam1, 0, image_stitch_input_info);
      compresseddepthImageCallback(compressed_depth_image_cam1, 0, image_stitch_input_info);
    }

#pragma omp section
    {
      compressedirImageCallback(compressed_ir_image_cam2, 1, image_stitch_input_info);
      compresseddepthImageCallback(compressed_depth_image_cam2, 1, image_stitch_input_info);
    }

#pragma omp section
    {
      compressedirImageCallback(compressed_ir_image_cam3, 2, image_stitch_input_info);
      compresseddepthImageCallback(compressed_depth_image_cam3, 2, image_stitch_input_info);
    }

#pragma omp section
    {
      compressedirImageCallback(compressed_ir_image_cam4, 3, image_stitch_input_info);
      compresseddepthImageCallback(compressed_depth_image_cam4, 3, image_stitch_input_info);
    }
  }
#else
  // Call respective callbacks with the id.
  compressedirImageCallback(compressed_ir_image_cam1, 0, image_stitch_input_info);
  compresseddepthImageCallback(compressed_depth_image_cam1, 0, image_stitch_input_info);

  compressedirImageCallback(compressed_ir_image_cam2, 1, image_stitch_input_info);
  compresseddepthImageCallback(compressed_depth_image_cam2, 1, image_stitch_input_info);

  compressedirImageCallback(compressed_ir_image_cam3, 2, image_stitch_input_info);
  compresseddepthImageCallback(compressed_depth_image_cam3, 2, image_stitch_input_info);

  compressedirImageCallback(compressed_ir_image_cam4, 3, image_stitch_input_info);
  compresseddepthImageCallback(compressed_depth_image_cam4, 3, image_stitch_input_info);
#endif
  // Add new node to the queue
  addInputNodeToQueue<ADI3DToFImageStitchingInputInfo>(image_stitch_input_info);

  PROFILE_FUNCTION_END(Message_Callback_processing);
}

/**
 * @brief Call back for synchronised topics(CameraInfo) for 2 camera
 * configuration
 *
 * @param CameraInfo_cam1 - Cam1 CameraInfo pointer
 * @param CameraInfo_cam2 - Cam2 CameraInfo pointer
 */
void ADI3DToFImageStitching::sync2CamerasCamInfoCallback(const sensor_msgs::CameraInfoConstPtr& CameraInfo_cam1,
                                                 const sensor_msgs::CameraInfoConstPtr& CameraInfo_cam2)
{
  camInfoCallback(CameraInfo_cam1, 0);
  camInfoCallback(CameraInfo_cam2, 1);
}

/**
 * @brief Call back for synchronised topics(CameraInfo) for 3 camera
 * configuration
 *
 * @param CameraInfo_cam1 - Cam1 CameraInfo pointer
 * @param CameraInfo_cam2 - Cam2 CameraInfo pointer
 * @param CameraInfo_cam3 - Cam3 CameraInfo pointer
 */
void ADI3DToFImageStitching::sync3CamerasCamInfoCallback(const sensor_msgs::CameraInfoConstPtr& CameraInfo_cam1,
                                                 const sensor_msgs::CameraInfoConstPtr& CameraInfo_cam2,
                                                 const sensor_msgs::CameraInfoConstPtr& CameraInfo_cam3)
{
  // Call respective callbacks with the id.
  camInfoCallback(CameraInfo_cam1, 0);
  camInfoCallback(CameraInfo_cam2, 1);
  camInfoCallback(CameraInfo_cam3, 2);
}

/**
 * @brief Call back for synchronised topics(CameraInfo) for 4 camera
 * configuration
 *
 * @param CameraInfo_cam1 - Cam1 CameraInfo pointer
 * @param CameraInfo_cam2 - Cam2 CameraInfo pointer
 * @param CameraInfo_cam3 - Cam3 CameraInfo pointer
 * @param CameraInfo_cam4 - Cam4 CameraInfo pointer
 */
void ADI3DToFImageStitching::sync4CamerasCamInfoCallback(const sensor_msgs::CameraInfoConstPtr& CameraInfo_cam1,
                                                 const sensor_msgs::CameraInfoConstPtr& CameraInfo_cam2,
                                                 const sensor_msgs::CameraInfoConstPtr& CameraInfo_cam3,
                                                 const sensor_msgs::CameraInfoConstPtr& CameraInfo_cam4)
{
  // Call respective callbacks with the id.
  camInfoCallback(CameraInfo_cam1, 0);
  camInfoCallback(CameraInfo_cam2, 1);
  camInfoCallback(CameraInfo_cam3, 2);
  camInfoCallback(CameraInfo_cam4, 3);
}

/**
 * @brief Low-level callback for Com-Info the ID would indicate the source
 * camera
 *
 * @param cam_info - Cam Info Pointer
 * @param cam_id - Camera Id
 */
void ADI3DToFImageStitching::camInfoCallback(const sensor_msgs::CameraInfoConstPtr& cam_info, int cam_id)
{
  if (!camera_parameters_updated_[cam_id])
  {
    CameraIntrinsics cam_intrinsics;
    populateCameraInstrinsics(cam_intrinsics);
    // Save
    sensor_image_width_ = cam_info->width;
    sensor_image_height_ = cam_info->height;
    if ((sensor_image_width_ != 512) && (sensor_image_height_ != 512))
    {
      return;
    }

    cam_intrinsics.camera_matrix[0] = cam_info->K[0];
    cam_intrinsics.camera_matrix[1] = 0.0f;
    cam_intrinsics.camera_matrix[2] = cam_info->K[2];
    cam_intrinsics.camera_matrix[3] = 0.0f;
    cam_intrinsics.camera_matrix[4] = cam_info->K[4];
    cam_intrinsics.camera_matrix[5] = cam_info->K[5];
    cam_intrinsics.camera_matrix[6] = 0.0f;
    cam_intrinsics.camera_matrix[7] = 0.0f;
    cam_intrinsics.camera_matrix[8] = 1.0f;

    for (int i = 0; i < 8; i++)
    {
      cam_intrinsics.distortion_coeffs[i] = cam_info->D[i];
    }

    image_proc_utils_[cam_id]->generateRangeTo3DLUT(&cam_intrinsics);

#ifdef ENABLE_GPU_OPTIMIZATION
    // Copy the LUT to GPU device
    stitch_frames_core_GPU_->copyRange2XYZLUT(image_proc_utils_[cam_id]->getRangeToXYZLUT(), cam_id);
#else
    stitch_frames_core_CPU_->copyRange2XYZLUT(image_proc_utils_[cam_id]->getRangeToXYZLUT(), cam_id);
#endif

    camera_parameters_updated_[cam_id] = true;
  }
}

/**
 * @brief Low-level callback for depth image
 *
 * @param depth_image - Pointer to depth image
 * @param cam_id - Camera ID
 * @param image_stitch_input_info - input object to load recieved depth data
 * Note: depth image is assumed to be 16 bpp image.
 */
void ADI3DToFImageStitching::depthImageCallback(const sensor_msgs::ImageConstPtr& depth_image, int cam_id,
                                        ADI3DToFImageStitchingInputInfo* image_stitch_input_info)
{
  int image_width = depth_image->width;
  int image_height = depth_image->height;

  if (!camera_parameters_updated_[cam_id])
  {
    return;
  }
  if ((image_width != 512) && (image_height != 512))
  {
    return;
  }
  if (image_stitch_input_info->getDepthFrame(cam_id) == nullptr)
  {
    return;
  }

  memcpy(image_stitch_input_info->getDepthFrame(cam_id), &depth_image->data[0], image_width * image_height * 2);

  // Set frame timestamp
  image_stitch_input_info->setDepthFrameTimestamp((ros::Time)depth_image->header.stamp, cam_id);

  // Store transform matrix(4x4)
  // Transform to map
  // Get the transform wrt to "map"
  if (!tf_recvd_[cam_id])
  {
    if (!tf_buffer_[cam_id].canTransform("map", depth_image->header.frame_id, depth_image->header.stamp,
                                         ros::Duration(30.0)))
    {
      return;
    }
    geometry_msgs::TransformStamped transform =
        tf_buffer_[cam_id].lookupTransform("map", depth_image->header.frame_id, depth_image->header.stamp);

    Eigen::Matrix4f transform4x4 = tf2::transformToEigen(transform.transform).matrix().cast<float>();

    float* transform_matrix = image_stitch_input_info->getTransformMatrix(cam_id);
    int id = 0;
    for (int r = 0; r < 4; r++)
    {
      for (int c = 0; c < 4; c++)
      {
        *transform_matrix++ = transform4x4(r, c);
      }
    }
    // tf_recvd_[cam_id]=true;
  }
  // Set flag
  depth_image_recvd_[cam_id] = true;
}

/**
 * @brief Low-level callback for Compressed depth image
 *
 * @param compressed_depth_image - Pointer to Compressed depth image
 * @param cam_id - Camera ID
 * @param image_stitch_input_info - input object to load recieved depth data
 * Note: depth image is assumed to be 16 bpp image.
 */
void ADI3DToFImageStitching::compresseddepthImageCallback(const sensor_msgs::CompressedImageConstPtr& compressed_depth_image,
                                                  int cam_id, ADI3DToFImageStitchingInputInfo* image_stitch_input_info)
{
  compressed_depth_image_transport::RvlCodec rvl;
  // sensor_msgs::ImageConstPtr depth_image_decompressed;

  if ((compressed_depth_image == nullptr))
  {
    return;
  }

  unsigned char* compressed_image_buf = (unsigned char*)&compressed_depth_image->data[8];
  int compressed_image_buf_size = compressed_depth_image->data.size();
  unsigned short* raw_image_buf = nullptr;

  int* image_width = (int*)&compressed_depth_image->data[0];
  int* image_height = (int*)&compressed_depth_image->data[4];

  if (!camera_parameters_updated_[cam_id])
  {
    return;
  }
  if ((*image_width != 512) && (*image_height != 512))
  {
    return;
  }
  if (image_stitch_input_info->getDepthFrame(cam_id) == nullptr)
  {
    return;
  }

  // decompress
  rvl.DecompressRVL(compressed_image_buf, image_stitch_input_info->getDepthFrame(cam_id), *image_width * *image_height);

  // Set frame timestamp
  image_stitch_input_info->setDepthFrameTimestamp((ros::Time)compressed_depth_image->header.stamp, cam_id);

  if (!tf_recvd_[cam_id])
  {
    // Store transform matrix(4x4)
    // Transform to map
    // Get the transform wrt to "map"
    if (!tf_buffer_[cam_id].canTransform("map", compressed_depth_image->header.frame_id,
                                         compressed_depth_image->header.stamp, ros::Duration(30.0)))
    {
      return;
    }
    geometry_msgs::TransformStamped transform = tf_buffer_[cam_id].lookupTransform(
        "map", compressed_depth_image->header.frame_id, compressed_depth_image->header.stamp);

    Eigen::Matrix4f transform4x4 = tf2::transformToEigen(transform.transform).matrix().cast<float>();

    float* transform_matrix = image_stitch_input_info->getTransformMatrix(cam_id);
    int id = 0;
    for (int r = 0; r < 4; r++)
    {
      for (int c = 0; c < 4; c++)
      {
        *transform_matrix++ = transform4x4(r, c);
      }
    }
    // tf_recvd_[cam_id]=true;
  }
  // Set flag
  depth_image_recvd_[cam_id] = true;
}

/**
 * @brief Low-level callback for IR image
 *
 * @param ir_image - IR image message pointer
 * @param cam_id - Camera ID
 * @param image_stitch_input_info - input object to load recieved IR data
 * Note : IR image is 16bpp image
 */
void ADI3DToFImageStitching::irImageCallback(const sensor_msgs::ImageConstPtr& ir_image, int cam_id,
                                     ADI3DToFImageStitchingInputInfo* image_stitch_input_info)
{
  sensor_image_width_ = ir_image->width;
  sensor_image_height_ = ir_image->height;
  if (!camera_parameters_updated_[cam_id])
  {
    return;
  }
  if ((sensor_image_width_ != 512) && (sensor_image_height_ != 512))
  {
    return;
  }
  if (image_stitch_input_info->getIRFrame(cam_id) != nullptr)
  {
    // copy
    memcpy(image_stitch_input_info->getIRFrame(cam_id), &ir_image->data[0],
           sensor_image_width_ * sensor_image_height_ * 2);

    // Set frame timestamp
    image_stitch_input_info->setIRFrameTimestamp((ros::Time)ir_image->header.stamp, cam_id);

    // Set flag
    ir_image_recvd_[cam_id] = true;
  }
}

/**
 * @brief Low-level callback for Compressed IR image
 *
 * @param compressed_ir_image - Compressed IR image message pointer
 * @param cam_id - Camera ID
 * @param image_stitch_input_info - input object to load recieved IR data
 * Note : IR image is 16bpp image
 */
void ADI3DToFImageStitching::compressedirImageCallback(const sensor_msgs::CompressedImageConstPtr& compressed_ir_image,
                                               int cam_id, ADI3DToFImageStitchingInputInfo* image_stitch_input_info)
{
  compressed_depth_image_transport::RvlCodec rvl;
  // sensor_msgs::ImageConstPtr ir_image_decompressed;
  // sensor_msgs::CompressedImage::Ptr comprsd_ptr(new sensor_msgs::CompressedImage());
  // comprsd_ptr = compressed_ir_image;

  if ((compressed_ir_image == nullptr))
  {
    return;
  }

  unsigned char* compressed_image_buf = (unsigned char*)&compressed_ir_image->data[8];
  int compressed_image_buf_size = compressed_ir_image->data.size();
  unsigned short* raw_image_buf = nullptr;

  int* image_width = (int*)&compressed_ir_image->data[0];
  int* image_height = (int*)&compressed_ir_image->data[4];

  sensor_image_width_ = *image_width;
  sensor_image_height_ = *image_height;
  if (!camera_parameters_updated_[cam_id])
  {
    return;
  }
  if ((sensor_image_width_ != 512) && (sensor_image_height_ != 512))
  {
    return;
  }
  if (image_stitch_input_info->getDepthFrame(cam_id) != nullptr)
  {
    // decompress
    rvl.DecompressRVL(compressed_image_buf, image_stitch_input_info->getIRFrame(cam_id),
                      sensor_image_width_ * sensor_image_height_);

    // Set frame timestamp
    image_stitch_input_info->setIRFrameTimestamp((ros::Time)compressed_ir_image->header.stamp, cam_id);

    // Set flag
    ir_image_recvd_[cam_id] = true;
  }
}

template <typename T>
void ADI3DToFImageStitching::addInputNodeToQueue(T* new_frame)
{
  // Add it to the queue
  input_thread_mtx_.lock();
  int queue_size = input_node_queue_.size();
  input_thread_mtx_.unlock();
  if (queue_size <= (input_queue_length_ - 1))
  {
    input_thread_mtx_.lock();
    input_node_queue_.push(new_frame);
    input_thread_mtx_.unlock();
  }
  else
  {
    std::cout << "Overwrite buffer" << std::endl;
    // If the Queue is full, then overwrite the last buffer with the latest frame
    input_thread_mtx_.lock();
    T* last_node = (T*)input_node_queue_.back();
    input_thread_mtx_.unlock();
    last_node = new_frame;
    delete new_frame;
  }
}
