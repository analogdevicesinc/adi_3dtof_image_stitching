/******************************************************************************
Copyright (c), 2023 - Analog Devices Inc. All Rights Reserved.
This software is PROPRIETARY & CONFIDENTIAL to Analog Devices, Inc.
and its licensors.
******************************************************************************/
#include <chrono>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <rclcpp/rclcpp.hpp>

#include "adi_3dtof_image_stitching.h"
#include "module_profile.h"

#ifdef ENABLE_OPENMP_OPTIMIZATION
#include <omp.h>
#endif

namespace enc = sensor_msgs::image_encodings;

/**
 * @brief Stitch function
 *
 * @return true
 * @return false
 */
bool ADI3DToFImageStitching::stitchFrames()
{
  // Make sure we have received frames from all the sensors
  bool all_callbacks_recvd = true;
  curr_frame_timestamp_ = rclcpp::Clock{}.now();

  if (num_sensors_ <= 1) all_callbacks_recvd = false;
  for (int i = 0; i < num_sensors_; i++) {
    if (enable_pointcloud_generation_ == 1 || enable_pointcloud_generation_ == 2) {
      if (!depth_image_recvd_[i] || !ir_image_recvd_[i]) {
        all_callbacks_recvd = false;
      }
    }
  }

  if (all_callbacks_recvd) {
    // Reset flags
    for (int i = 0; i < num_sensors_; i++) {
      ir_image_recvd_[i] = false;
      depth_image_recvd_[i] = false;
    }

    // Read the available node from the queue
    ADI3DToFImageStitchingInputInfo * image_stitch_input_info = getInputNode();

    // Get a output node
    ADI3DToFImageStitchingOutputInfo * new_output_node =
      new ADI3DToFImageStitchingOutputInfo(out_image_width_, out_image_height_);
    if (new_output_node == nullptr) {
      return false;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "adi_3dtof_image_stitching::Running loop");
    PROFILE_FUNCTION_START(ImageStitch_RUN);

    float * out_xyz_frame = new_output_node->xyz_frame_;
    unsigned short * out_depth_frame = new_output_node->depth_frame_;
    unsigned short * out_ir_frame = new_output_node->ir_frame_;
    int * out_lut_3d_to_2d_mapping = new_output_node->lut_3d_to_2d_mapping_;

    // Core processing function
#ifdef ENABLE_GPU_OPTIMIZATION
    stitch_frames_core_GPU_->stitchFrames(
      image_stitch_input_info, num_sensors_, out_xyz_frame, out_depth_frame, out_ir_frame,
      out_lut_3d_to_2d_mapping);
#else
    stitch_frames_core_CPU_->stitchFrames(
      image_stitch_input_info, num_sensors_, out_xyz_frame, out_depth_frame, out_ir_frame,
      out_lut_3d_to_2d_mapping);
#endif

    // Publish the output
    if (new_output_node != nullptr) {
      new_output_node->frame_counter_ = frame_counter_;
      // Push
      pushOutputNode(new_output_node);
    }

    // Update frame count
    frame_counter_++;

    delete image_stitch_input_info;

    PROFILE_FUNCTION_END(ImageStitch_RUN);
  }

  return true;
}
/**
 * @brief Function to convert 3D XYZ frame to PCL point cloud format
 * 
 * @param xyz_frame - Input XYZ frame
 * @param out_pointcloud - PCl point cloud output
 */
void ADI3DToFImageStitching::generatePointCloud(
  float * xyz_frame, int * lut_3d_to_2d_mapping,
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & out_pointcloud)
{
  int i;
#ifdef ENABLE_OPENMP_OPTIMIZATION
#pragma omp parallel for
#endif
  for (i = 0; i < int(out_image_width_ * out_image_height_); i++) {
    pcl::PointXYZ point = {0, 0, 0};
    if (lut_3d_to_2d_mapping[i] != -1) {
      // compensate for the additional transformation
      point.x = xyz_frame[3 * lut_3d_to_2d_mapping[i]];
      point.y = xyz_frame[3 * lut_3d_to_2d_mapping[i] + 1];
      point.z = xyz_frame[3 * lut_3d_to_2d_mapping[i] + 2];
    }
    out_pointcloud->points[i] = point;
  }

  out_pointcloud->width = out_image_width_;
  out_pointcloud->height = out_image_height_;
}

/**
 * @brief Function to read the next node from the input Queue
 *
 * @return ADI3DToFImageStitchingInputInfo* - Pointer to the next node
 */
ADI3DToFImageStitchingInputInfo * ADI3DToFImageStitching::getInputNode()
{
  ADI3DToFImageStitchingInputInfo * innode = nullptr;
  input_thread_mtx_.lock();
  int queue_size = input_node_queue_.size();
  input_thread_mtx_.unlock();

  // We will try till read the buffer, this is just to allow the read thread to fill the queue
  while ((innode == nullptr) && ((!input_read_abort_) || queue_size > 0)) {
    input_thread_mtx_.lock();
    queue_size = input_node_queue_.size();
    input_thread_mtx_.unlock();
    if (queue_size > 0) {
      input_thread_mtx_.lock();
      innode = (ADI3DToFImageStitchingInputInfo *)input_node_queue_.front();
      input_node_queue_.pop();
      input_thread_mtx_.unlock();
    }
    if (innode == nullptr) {
      // Wait for the buffers to be filled
      try {
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
      } catch (...) {
        printf("\n ERROR: Exception running sleep \n");
      }
    }
  }
  return innode;
}
