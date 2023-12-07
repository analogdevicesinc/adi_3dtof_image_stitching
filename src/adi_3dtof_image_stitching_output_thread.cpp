/******************************************************************************
Copyright (c), 2023 - Analog Devices Inc. All Rights Reserved.
This software is PROPRIETARY & CONFIDENTIAL to Analog Devices, Inc.
and its licensors.
******************************************************************************/
#include <sensor_msgs/msg/image.h>

#include <chrono>
#include <thread>

#include "adi_3dtof_image_stitching.h"
#include "module_profile.h"

namespace enc = sensor_msgs::image_encodings;

/**
 * @brief This function sets the abort flag for the output thread,
 * the function is normally called by the main function to abort the output thread.
 *
 */
void ADI3DToFImageStitching::processOutputAbort() { process_output_thread_abort_ = true; }

/**
 * @brief This function pushes the debug node to the output queue.
 * If the queue is full, then the last item in the queue gets replaced
 * with the latest node.
 *
 * @param new_output_node : Pointer to the debug node o be published
 */
void ADI3DToFImageStitching::pushOutputNode(ADI3DToFImageStitchingOutputInfo * new_output_node)
{
  output_thread_mtx_.lock();
  int queue_size = output_node_queue_.size();
  output_thread_mtx_.unlock();

  if (queue_size <= (output_queue_length_ - 1)) {
    // Push this one
    output_thread_mtx_.lock();
    output_node_queue_.push(new_output_node);
    output_thread_mtx_.unlock();
  } else {
    __attribute__((unused)) ADI3DToFImageStitchingOutputInfo * last_node = nullptr;
    // Replace the last item with the current one.
    output_thread_mtx_.lock();
    last_node = (ADI3DToFImageStitchingOutputInfo *)output_node_queue_.back();
    output_thread_mtx_.unlock();

    // Copy the contents of new node into the old one and then delete the new node.
    last_node = new_output_node;

    // Delete this one
    delete new_output_node;
  }
}

/**
 * @brief The output process function, this is running a loop
 * which reads the frame from the putput queue, generates the visualization output
 * and publishes the output as ROS messages.
 *
 */
void ADI3DToFImageStitching::processOutput()
{
  int queue_size = output_node_queue_.size();

  while ((!process_output_thread_abort_) || (queue_size > 0)) {
    // std::cout << "Inside processOutput" << std::endl;
    output_thread_mtx_.lock();
    queue_size = output_node_queue_.size();
    output_thread_mtx_.unlock();
    if (queue_size > 0) {
      PROFILE_FUNCTION_START(processOutput_Thread);

      output_thread_mtx_.lock();
      ADI3DToFImageStitchingOutputInfo * new_frame =
        (ADI3DToFImageStitchingOutputInfo *)output_node_queue_.front();
      output_node_queue_.pop();
      output_thread_mtx_.unlock();

      // Publish the topics.
      // Publish Output
      // convert to 16 bit depth and IR image of CV format.
      cv::Mat m_depth_image, m_ir_image;
      m_depth_image =
        cv::Mat(out_image_height_, out_image_width_, CV_16UC1, new_frame->depth_frame_);
      m_ir_image = cv::Mat(out_image_height_, out_image_width_, CV_16UC1, new_frame->ir_frame_);
      publishImageAsRosMsg(
        m_depth_image, enc::TYPE_16UC1, camera_link_, stitched_depth_image_publisher_);
      publishImageAsRosMsg(m_ir_image, enc::TYPE_16UC1, camera_link_, stitched_ir_image_publisher_);

      // Publish Stitched point cloud
      generatePointCloud(new_frame->xyz_frame_, new_frame->lut_3d_to_2d_mapping_, stitched_pc_pcl_);
      pcl::toROSMsg(*stitched_pc_pcl_, stitched_pc_);
      // Update frame header information for point-cloud
      //stitched_pc_.header.seq = frame_counter_;
      stitched_pc_.header.stamp = curr_frame_timestamp_;
      stitched_pc_.header.frame_id = camera_link_;
      combo_out_point_cloud_publisher_->publish(stitched_pc_);

      // Publish camera info
      fillAndPublishCameraInfo(camera_link_, stitched_camera_info_publisher_);

      // Write outputs
      if (output_sensor_ != nullptr) {
        output_sensor_->write(m_depth_image, m_ir_image, out_image_width_, out_image_height_);
      }

      delete new_frame;
      PROFILE_FUNCTION_END(processOutput_Thread);
    }

    // Sleep
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
}
