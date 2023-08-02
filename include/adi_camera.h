/******************************************************************************
Copyright (c), 2023 - Analog Devices Inc. All Rights Reserved.
This software is PROPRIETARY & CONFIDENTIAL to Analog Devices, Inc.
and its licensors.
******************************************************************************/

#ifndef ADI_CAMERA_H
#define ADI_CAMERA_H

#include <array>
/**
 * @brief Intrinsic parameters of the camera
 *
 */
struct CameraIntrinsics
{
  /**
   * @brief Camera matrix = [fx, 0, cx, 0, fy, cy, 0, 0, 1]
   * fx, fy : Camera focal lengths (pixels)
   * cx, cy : Camera principal points or optical centers (pixels)
   *
   */
  float camera_matrix[9];

  /**
   * @brief Distortion vector = [k1, k2, p1, p2, k3, k4, k5, k6]
   * k1, k2, k3, k4, k5, k6 : Camera radial distortion cofficients
   * p1, p2 : Camera tangential distortion coefficients
   *
   */
  float distortion_coeffs[8];
};

/**
 * @brief Extrinsic parameters of the camera
 *
 */
struct CameraExtrinsics
{
  /**
   * @brief Camera rotation parameters
   *
   */
  float rotation_matrix[9];

  /**
   * @brief Camera translation matrix : [Tx,Ty,Tz]
   *
   */
  float translation_matrix[3];
};

#endif