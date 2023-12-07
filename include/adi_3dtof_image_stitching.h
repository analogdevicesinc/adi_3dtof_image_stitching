/******************************************************************************
Copyright (c), 2023 - Analog Devices Inc. All Rights Reserved.
This software is PROPRIETARY & CONFIDENTIAL to Analog Devices, Inc.
and its licensors.
******************************************************************************/
#ifndef ADI_3DTOF_IMAGE_STITCH_H
#define ADI_3DTOF_IMAGE_STITCH_H

#include <rclcpp/rclcpp.hpp>

#include "adi_3dtof_image_stitching_input_info.h"
#include "adi_3dtof_image_stitching_output_info.h"
#include "adi_camera.h"
#include "image_proc_utils.h"
#include "module_profile.h"
#include "output_sensor.h"
#include "output_sensor_factory.h"
#ifdef ENABLE_GPU_OPTIMIZATION
#include "stitch_frames_core_gpu.cuh"
#else
#include "stitch_frames_core_cpu.h"
#endif
#include <cv_bridge/cv_bridge.h>
#include <image_geometry/pinhole_camera_model.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_representation.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/msg/image.h>
#include <std_msgs/msg/bool.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <chrono>
#include <functional>
#include <memory>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <pcl_ros/transforms.hpp>
#include <queue>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/distortion_models.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <std_msgs/msg/string.hpp>
#include <string>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <thread>
#include <utility>

#define MAX_QUEUE_SIZE_FOR_TIME_SYNC 10
#define SENSOR_OVERLAP_PERCENT 10.0f

namespace enc = sensor_msgs::image_encodings;

// Declaring synchronization policies
typedef message_filters::sync_policies::ApproximateTime<
  sensor_msgs::msg::CameraInfo, sensor_msgs::msg::CameraInfo>
  Sync_CameraInfo_2sensors;
typedef message_filters::sync_policies::ApproximateTime<
  sensor_msgs::msg::CameraInfo, sensor_msgs::msg::CameraInfo, sensor_msgs::msg::CameraInfo>
  Sync_CameraInfo_3sensors;
typedef message_filters::sync_policies::ApproximateTime<
  sensor_msgs::msg::CameraInfo, sensor_msgs::msg::CameraInfo, sensor_msgs::msg::CameraInfo,
  sensor_msgs::msg::CameraInfo>
  Sync_CameraInfo_4sensors;
typedef message_filters::sync_policies::ApproximateTime<
  sensor_msgs::msg::Image, sensor_msgs::msg::Image, sensor_msgs::msg::Image,
  sensor_msgs::msg::Image>
  sync_depth_ir_2;
typedef message_filters::sync_policies::ApproximateTime<
  sensor_msgs::msg::CompressedImage, sensor_msgs::msg::CompressedImage,
  sensor_msgs::msg::CompressedImage, sensor_msgs::msg::CompressedImage>
  sync_compressed_depth_ir_2;
typedef message_filters::sync_policies::ApproximateTime<
  sensor_msgs::msg::Image, sensor_msgs::msg::Image, sensor_msgs::msg::Image,
  sensor_msgs::msg::Image, sensor_msgs::msg::Image, sensor_msgs::msg::Image>
  sync_depth_ir_3;
typedef message_filters::sync_policies::ApproximateTime<
  sensor_msgs::msg::CompressedImage, sensor_msgs::msg::CompressedImage,
  sensor_msgs::msg::CompressedImage, sensor_msgs::msg::CompressedImage,
  sensor_msgs::msg::CompressedImage, sensor_msgs::msg::CompressedImage>
  sync_compressed_depth_ir_3;
typedef message_filters::sync_policies::ApproximateTime<
  sensor_msgs::msg::Image, sensor_msgs::msg::Image, sensor_msgs::msg::Image,
  sensor_msgs::msg::Image, sensor_msgs::msg::Image, sensor_msgs::msg::Image,
  sensor_msgs::msg::Image, sensor_msgs::msg::Image>
  sync_depth_ir_4;
typedef message_filters::sync_policies::ApproximateTime<
  sensor_msgs::msg::CompressedImage, sensor_msgs::msg::CompressedImage,
  sensor_msgs::msg::CompressedImage, sensor_msgs::msg::CompressedImage,
  sensor_msgs::msg::CompressedImage, sensor_msgs::msg::CompressedImage,
  sensor_msgs::msg::CompressedImage, sensor_msgs::msg::CompressedImage>
  sync_compressed_depth_ir_4;

// Defining Synchronizers
typedef message_filters::Synchronizer<Sync_CameraInfo_2sensors> sync_CamInfo_2cam;
typedef message_filters::Synchronizer<Sync_CameraInfo_3sensors> sync_CamInfo_3cam;
typedef message_filters::Synchronizer<Sync_CameraInfo_4sensors> sync_CamInfo_4cam;
typedef message_filters::Synchronizer<sync_depth_ir_2> sync_depth_ir_2cam;
typedef message_filters::Synchronizer<sync_depth_ir_3> sync_depth_ir_3cam;
typedef message_filters::Synchronizer<sync_depth_ir_4> sync_depth_ir_4cam;
typedef message_filters::Synchronizer<sync_compressed_depth_ir_2> sync_compressed_depth_ir_2cam;
typedef message_filters::Synchronizer<sync_compressed_depth_ir_3> sync_compressed_depth_ir_3cam;
typedef message_filters::Synchronizer<sync_compressed_depth_ir_4> sync_compressed_depth_ir_4cam;

/**
 * @brief This is main class for this package
 *
 */
class ADI3DToFImageStitching : public rclcpp::Node
{
  typedef void (ADI3DToFImageStitching::*ImageCallbackFuncPointer)(
    const sensor_msgs::msg::Image::ConstSharedPtr &);
  typedef void (ADI3DToFImageStitching::*PointCloudCallbackFuncPointer)(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr &);

public:
  static const int MAX_NUM_DEVICES = 4;
  bool camera_parameters_updated_[MAX_NUM_DEVICES] = {false};
  float vertical_fov_sensor_in_degrees_ = 75.0f;
  float horizontal_fov_sensor_in_degrees_ = 75.0f;
  int out_image_height_ = 512;
  int out_image_width_ = 512 * MAX_NUM_DEVICES;

  /**
   * @brief Construct a new ADI3DToFImageStitching object
   *
   */
  ADI3DToFImageStitching() : Node("adi_3dtof_image_stitching")
  {
    RCLCPP_INFO(
      rclcpp::get_logger("rclcpp"),
      "adi_3dtof_image_stitching::Inside "
      "ADI3DToFImageStitching()");

    // Get Parameters
    rcl_interfaces::msg::ParameterDescriptor policy_x_description{};
    policy_x_description.read_only = true;
    this->declare_parameter<int>("param_enable_pointcloud_generation", 2, policy_x_description);
    this->declare_parameter<int>("param_output_mode", 0, policy_x_description);
    this->declare_parameter<std::string>(
      "param_camera_link", "adi_camera_link", policy_x_description);
    this->declare_parameter<std::string>(
      "param_out_file_name", "stitched_output.avi", policy_x_description);
    this->declare_parameter("param_camera_prefixes", rclcpp::PARAMETER_STRING_ARRAY);
    this->declare_parameter<float>(
      "param_sensor_vertical_fov_in_degrees", 75.0f, policy_x_description);
    this->declare_parameter<float>(
      "param_sensor_horizontal_fov_in_degrees", 75.0f, policy_x_description);
    this->declare_parameter<int>("param_sensor_image_height", 512, policy_x_description);
    this->declare_parameter<int>("param_sensor_image_width", 512, policy_x_description);
    this->declare_parameter<bool>("param_enable_autoscaling", true, policy_x_description);
    this->declare_parameter<bool>("param_enable_depth_ir_compression", false, policy_x_description);

    enable_pointcloud_generation_ =
      this->get_parameter("param_enable_pointcloud_generation").get_parameter_value().get<int>();
    enable_depth_ir_compression_ =
      this->get_parameter("param_enable_depth_ir_compression").get_parameter_value().get<bool>();
    output_sensor_mode_ = this->get_parameter("param_output_mode").get_parameter_value().get<int>();
    camera_link_ =
      this->get_parameter("param_camera_link").get_parameter_value().get<std::string>();
    output_file_name_ =
      this->get_parameter("param_out_file_name").get_parameter_value().get<std::string>();

    // Getting Camera prefixes
    rclcpp::Parameter string_array_param = this->get_parameter("param_camera_prefixes");
    std::vector<std::string> cam_prefix = string_array_param.as_string_array();
    for (int i = 0; i < (int)cam_prefix.size(); i++) {
      std::cerr << "camera_prefixes: " << cam_prefix[i] << std::endl;
    }
    // Getting projection parameters
    vertical_fov_sensor_in_degrees_ = this->get_parameter("param_sensor_vertical_fov_in_degrees")
                                        .get_parameter_value()
                                        .get<float>();
    horizontal_fov_sensor_in_degrees_ =
      this->get_parameter("param_sensor_horizontal_fov_in_degrees")
        .get_parameter_value()
        .get<float>();
    sensor_image_height_ =
      this->get_parameter("param_sensor_image_height").get_parameter_value().get<int>();
    sensor_image_width_ =
      this->get_parameter("param_sensor_image_width").get_parameter_value().get<int>();
    enable_autoscaling_ =
      this->get_parameter("param_enable_autoscaling").get_parameter_value().get<bool>();

    // init
    frame_counter_ = 0;
    //flag to indicate autoscaling is not applied yet
    autoscaling_flag_ = false;

    // Do not allow more than the max cameras supported.
    int max_devices_allowed =
      static_cast<int>(sizeof(depth_image_subscriber_) / sizeof(depth_image_subscriber_[0]));
    num_sensors_ = std::min(max_devices_allowed, static_cast<int>(cam_prefix.size()));
    input_read_abort_ = false;

    //Setting output width and height for max horizontal sensor combination
    out_image_width_ = sensor_image_width_ * max_devices_allowed;
    out_image_height_ = sensor_image_height_;

    for (int i = 0; i < num_sensors_; i++) {
      // Synchronized subscribers
      camera_info_subscriber_[i].subscribe(this, "/" + cam_prefix[i] + "/camera_info");
      if (enable_depth_ir_compression_ == 1) {
        compressed_ir_image_subscriber_[i].subscribe(
          this, "/" + cam_prefix[i] + "/ir_image/compressedDepth");
        compressed_depth_image_subscriber_[i].subscribe(
          this, "/" + cam_prefix[i] + "/depth_image/compressedDepth");
      } else {
        ir_image_subscriber_[i].subscribe(this, "/" + cam_prefix[i] + "/ir_image");
        depth_image_subscriber_[i].subscribe(this, "/" + cam_prefix[i] + "/depth_image");
      }

      // Create TF listerner instance
      tf_buffer_[i] = std::make_unique<tf2_ros::Buffer>(this->get_clock());
      tf_listener_[i] = new std::shared_ptr<tf2_ros::TransformListener>(
        std::make_shared<tf2_ros::TransformListener>(*tf_buffer_[i]));

      populateCameraInstrinsics(depth_intrinsics_);
      image_proc_utils_[i] =
        new ImageProcUtils(&depth_intrinsics_, sensor_image_width_, sensor_image_height_);

      // Different flags
      ir_image_recvd_[i] = false;
      depth_image_recvd_[i] = false;
      tf_recvd_[i] = false;
    }

    try {
      // Get output sensor module
      output_sensor_ = OutputSensorFactory::getOutputSensor(output_sensor_mode_);   
    } catch (...) {
        printf("\n ERROR: Exception while creating Output Node \n");
    }

    if (output_sensor_ != nullptr) {
      output_sensor_->open(output_file_name_, out_image_width_, out_image_height_);
    }

    if (num_sensors_ == 2) {
      try {
        sync_CamInfo_2cam_.reset(new sync_CamInfo_2cam(
          Sync_CameraInfo_2sensors(MAX_QUEUE_SIZE_FOR_TIME_SYNC), camera_info_subscriber_[0],
          camera_info_subscriber_[1]));
        sync_CamInfo_2cam_->registerCallback(std::bind(
          &ADI3DToFImageStitching::sync2CamerasCamInfoCallback, this, std::placeholders::_1,
          std::placeholders::_2));

        if (enable_pointcloud_generation_ == 1 || enable_pointcloud_generation_ == 2) {
          if (enable_depth_ir_compression_ == 1) {
            sync_compressed_depth_ir_2cam_.reset(new sync_compressed_depth_ir_2cam(
              sync_compressed_depth_ir_2(MAX_QUEUE_SIZE_FOR_TIME_SYNC),
              compressed_depth_image_subscriber_[0], compressed_ir_image_subscriber_[0],
              compressed_depth_image_subscriber_[1], compressed_ir_image_subscriber_[1]));
            sync_compressed_depth_ir_2cam_->registerCallback(std::bind(
              &ADI3DToFImageStitching::sync2CamerasCompressedDepthIRImageCallback, this,
              std::placeholders::_1, std::placeholders::_2, std::placeholders::_3,
              std::placeholders::_4));
          } else {
            sync_depth_ir_2cam_.reset(new sync_depth_ir_2cam(
              sync_depth_ir_2(MAX_QUEUE_SIZE_FOR_TIME_SYNC), depth_image_subscriber_[0],
              ir_image_subscriber_[0], depth_image_subscriber_[1], ir_image_subscriber_[1]));
            sync_depth_ir_2cam_->registerCallback(std::bind(
              &ADI3DToFImageStitching::sync2CamerasDepthIRImageCallback, this,
              std::placeholders::_1, std::placeholders::_2, std::placeholders::_3,
              std::placeholders::_4));
          }
        }
      } catch (...) {
        printf("\n ERROR: Exception while registering Synchronizers \n");
      }
    } else if (num_sensors_ == 3) {
      try {
        sync_CamInfo_3cam_.reset(new sync_CamInfo_3cam(
          Sync_CameraInfo_3sensors(MAX_QUEUE_SIZE_FOR_TIME_SYNC), camera_info_subscriber_[0],
          camera_info_subscriber_[1], camera_info_subscriber_[2]));
        sync_CamInfo_3cam_->registerCallback(std::bind(
          &ADI3DToFImageStitching::sync3CamerasCamInfoCallback, this, std::placeholders::_1,
          std::placeholders::_2, std::placeholders::_3));

        if (enable_pointcloud_generation_ == 1 || enable_pointcloud_generation_ == 2) {
          if (enable_depth_ir_compression_ == 1) {
            sync_compressed_depth_ir_3cam_.reset(new sync_compressed_depth_ir_3cam(
              sync_compressed_depth_ir_3(MAX_QUEUE_SIZE_FOR_TIME_SYNC),
              compressed_depth_image_subscriber_[0], compressed_ir_image_subscriber_[0],
              compressed_depth_image_subscriber_[1], compressed_ir_image_subscriber_[1],
              compressed_depth_image_subscriber_[2], compressed_ir_image_subscriber_[2]));
            sync_compressed_depth_ir_3cam_->registerCallback(std::bind(
              &ADI3DToFImageStitching::sync3CamerasCompressedDepthIRImageCallback, this,
              std::placeholders::_1, std::placeholders::_2, std::placeholders::_3,
              std::placeholders::_4, std::placeholders::_5, std::placeholders::_6));
          } else {
            sync_depth_ir_3cam_.reset(new sync_depth_ir_3cam(
              sync_depth_ir_3(MAX_QUEUE_SIZE_FOR_TIME_SYNC), depth_image_subscriber_[0],
              ir_image_subscriber_[0], depth_image_subscriber_[1], ir_image_subscriber_[1],
              depth_image_subscriber_[2], ir_image_subscriber_[2]));
            sync_depth_ir_3cam_->registerCallback(std::bind(
              &ADI3DToFImageStitching::sync3CamerasDepthIRImageCallback, this,
              std::placeholders::_1, std::placeholders::_2, std::placeholders::_3,
              std::placeholders::_4, std::placeholders::_5, std::placeholders::_6));
          }
        }
      } catch (...) {
        printf("\n ERROR: Exception while registering Synchronizers \n");
      }
    } else if (num_sensors_ == 4) {
      try {
        sync_CamInfo_4cam_.reset(new sync_CamInfo_4cam(
          Sync_CameraInfo_4sensors(MAX_QUEUE_SIZE_FOR_TIME_SYNC), camera_info_subscriber_[0],
          camera_info_subscriber_[1], camera_info_subscriber_[2], camera_info_subscriber_[3]));
        sync_CamInfo_4cam_->registerCallback(std::bind(
          &ADI3DToFImageStitching::sync4CamerasCamInfoCallback, this, std::placeholders::_1,
          std::placeholders::_2, std::placeholders::_3, std::placeholders::_4));
        // We need depth image for point-cloud generation

        if (enable_pointcloud_generation_ == 1 || enable_pointcloud_generation_ == 2) {
          if (enable_depth_ir_compression_ == 1) {
            sync_compressed_depth_ir_4cam_.reset(new sync_compressed_depth_ir_4cam(
              sync_compressed_depth_ir_4(MAX_QUEUE_SIZE_FOR_TIME_SYNC),
              compressed_depth_image_subscriber_[0], compressed_ir_image_subscriber_[0],
              compressed_depth_image_subscriber_[1], compressed_ir_image_subscriber_[1],
              compressed_depth_image_subscriber_[2], compressed_ir_image_subscriber_[2],
              compressed_depth_image_subscriber_[3], compressed_ir_image_subscriber_[3]));
            sync_compressed_depth_ir_4cam_->registerCallback(std::bind(
              &ADI3DToFImageStitching::sync4CamerasCompressedDepthIRImageCallback, this,
              std::placeholders::_1, std::placeholders::_2, std::placeholders::_3,
              std::placeholders::_4, std::placeholders::_5, std::placeholders::_6,
              std::placeholders::_7, std::placeholders::_8));
          } else {
            sync_depth_ir_4cam_.reset(new sync_depth_ir_4cam(
              sync_depth_ir_4(MAX_QUEUE_SIZE_FOR_TIME_SYNC), depth_image_subscriber_[0],
              ir_image_subscriber_[0], depth_image_subscriber_[1], ir_image_subscriber_[1],
              depth_image_subscriber_[2], ir_image_subscriber_[2], depth_image_subscriber_[3],
              ir_image_subscriber_[3]));
            sync_depth_ir_4cam_->registerCallback(std::bind(
              &ADI3DToFImageStitching::sync4CamerasDepthIRImageCallback, this,
              std::placeholders::_1, std::placeholders::_2, std::placeholders::_3,
              std::placeholders::_4, std::placeholders::_5, std::placeholders::_6,
              std::placeholders::_7, std::placeholders::_8));
          }
        }
      } catch (...) {
        printf("\n ERROR: Exception while registering Synchronizers \n");
      }
    }

    // Create publishers.
    stitched_depth_image_publisher_ =
      this->create_publisher<sensor_msgs::msg::Image>("depth_image", 10);
    stitched_ir_image_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("ir_image", 10);
    combo_out_point_cloud_publisher_ =
      this->create_publisher<sensor_msgs::msg::PointCloud2>("point_cloud", 10);
    stitched_camera_info_publisher_ =
      this->create_publisher<sensor_msgs::msg::CameraInfo>("camera_info", 10);

    // Initialize stitched point cloud buffer
    stitched_pc_pcl_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    stitched_pc_pcl_->points.resize(out_image_width_ * out_image_height_);

    float max_stitched_horizontal_FOV =
      ((horizontal_fov_sensor_in_degrees_ * ((100 - SENSOR_OVERLAP_PERCENT) / 100)) *
       (max_devices_allowed - 1)) +
      horizontal_fov_sensor_in_degrees_;

#ifdef ENABLE_GPU_OPTIMIZATION
    // GPU class init
    stitch_frames_core_GPU_ = new StitchFramesCoreGPU(
      sensor_image_width_, sensor_image_height_, out_image_width_, out_image_height_, num_sensors_,
      vertical_fov_sensor_in_degrees_, max_stitched_horizontal_FOV);
#else
    stitch_frames_core_CPU_ = new StitchFramesCoreCPU(
      sensor_image_width_, sensor_image_height_, out_image_width_, out_image_height_, num_sensors_,
      vertical_fov_sensor_in_degrees_, max_stitched_horizontal_FOV);
#endif

    input_queue_length_ = 2;
    output_queue_length_ = 2;
  }

  /**
   * @brief Destroy the ADI3DToFImageStitching object
   *
   */
  ~ADI3DToFImageStitching()
  {
    for (int i = 0; i < num_sensors_; i++) {
      delete tf_listener_[i];
      delete image_proc_utils_[i];
    }

    // Close outputs
    if (output_sensor_ != nullptr) {
      output_sensor_->close();
    }
  }

  void sync2CamerasDepthIRImageCallback(
    const sensor_msgs::msg::Image::ConstSharedPtr & depth_image_cam1,
    const sensor_msgs::msg::Image::ConstSharedPtr & ir_image_cam1,
    const sensor_msgs::msg::Image::ConstSharedPtr & depth_image_cam2,
    const sensor_msgs::msg::Image::ConstSharedPtr & ir_image_cam2);

  void sync3CamerasDepthIRImageCallback(
    const sensor_msgs::msg::Image::ConstSharedPtr & depth_image_cam1,
    const sensor_msgs::msg::Image::ConstSharedPtr & ir_image_cam1,
    const sensor_msgs::msg::Image::ConstSharedPtr & depth_image_cam2,
    const sensor_msgs::msg::Image::ConstSharedPtr & ir_image_cam2,
    const sensor_msgs::msg::Image::ConstSharedPtr & depth_image_cam3,
    const sensor_msgs::msg::Image::ConstSharedPtr & ir_image_cam3);

  void sync4CamerasDepthIRImageCallback(
    const sensor_msgs::msg::Image::ConstSharedPtr & depth_image_cam1,
    const sensor_msgs::msg::Image::ConstSharedPtr & ir_image_cam1,
    const sensor_msgs::msg::Image::ConstSharedPtr & depth_image_cam2,
    const sensor_msgs::msg::Image::ConstSharedPtr & ir_image_cam2,
    const sensor_msgs::msg::Image::ConstSharedPtr & depth_image_cam3,
    const sensor_msgs::msg::Image::ConstSharedPtr & ir_image_cam3,
    const sensor_msgs::msg::Image::ConstSharedPtr & depth_image_cam4,
    const sensor_msgs::msg::Image::ConstSharedPtr & ir_image_cam4);

  void sync2CamerasCompressedDepthIRImageCallback(
    const sensor_msgs::msg::CompressedImage::ConstSharedPtr & compressed_depth_image_cam1,
    const sensor_msgs::msg::CompressedImage::ConstSharedPtr & compressed_ir_image_cam1,
    const sensor_msgs::msg::CompressedImage::ConstSharedPtr & compressed_depth_image_cam2,
    const sensor_msgs::msg::CompressedImage::ConstSharedPtr & compressed_ir_image_cam2);

  void sync3CamerasCompressedDepthIRImageCallback(
    const sensor_msgs::msg::CompressedImage::ConstSharedPtr & compressed_depth_image_cam1,
    const sensor_msgs::msg::CompressedImage::ConstSharedPtr & compressed_ir_image_cam1,
    const sensor_msgs::msg::CompressedImage::ConstSharedPtr & compressed_depth_image_cam2,
    const sensor_msgs::msg::CompressedImage::ConstSharedPtr & compressed_ir_image_cam2,
    const sensor_msgs::msg::CompressedImage::ConstSharedPtr & compressed_depth_image_cam3,
    const sensor_msgs::msg::CompressedImage::ConstSharedPtr & compressed_ir_image_cam3);

  void sync4CamerasCompressedDepthIRImageCallback(
    const sensor_msgs::msg::CompressedImage::ConstSharedPtr & compressed_depth_image_cam1,
    const sensor_msgs::msg::CompressedImage::ConstSharedPtr & compressed_ir_image_cam1,
    const sensor_msgs::msg::CompressedImage::ConstSharedPtr & compressed_depth_image_cam2,
    const sensor_msgs::msg::CompressedImage::ConstSharedPtr & compressed_ir_image_cam2,
    const sensor_msgs::msg::CompressedImage::ConstSharedPtr & compressed_depth_image_cam3,
    const sensor_msgs::msg::CompressedImage::ConstSharedPtr & compressed_ir_image_cam3,
    const sensor_msgs::msg::CompressedImage::ConstSharedPtr & compressed_depth_image_cam4,
    const sensor_msgs::msg::CompressedImage::ConstSharedPtr & compressed_ir_image_cam4);

  void sync2CamerasCamInfoCallback(
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr & CameraInfo_cam1,
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr & CameraInfo_cam2);

  void sync3CamerasCamInfoCallback(
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr & CameraInfo_cam1,
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr & CameraInfo_cam2,
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr & CameraInfo_cam3);

  void sync4CamerasCamInfoCallback(
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr & CameraInfo_cam1,
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr & CameraInfo_cam2,
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr & CameraInfo_cam3,
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr & CameraInfo_cam4);

  void camInfoCallback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr & cam_info, int cam_id);

  void depthImageCallback(
    const sensor_msgs::msg::Image::ConstSharedPtr & depth_image, int cam_id,
    ADI3DToFImageStitchingInputInfo * image_stitch_input_info);

  void compresseddepthImageCallback(
    const sensor_msgs::msg::CompressedImage::ConstSharedPtr & depth_image, int cam_id,
    ADI3DToFImageStitchingInputInfo * image_stitch_input_info);

  void irImageCallback(
    const sensor_msgs::msg::Image::ConstSharedPtr & ir_image, int cam_id,
    ADI3DToFImageStitchingInputInfo * image_stitch_input_info);

  void compressedirImageCallback(
    const sensor_msgs::msg::CompressedImage::ConstSharedPtr & ir_image, int cam_id,
    ADI3DToFImageStitchingInputInfo * image_stitch_input_info);

  void AutoscaleStitching();

  bool stitchFrames();

  void generatePointCloud(
    float * xyz_frame, int * lut_3d_to_2d_mapping,
    const pcl::PointCloud<pcl::PointXYZ>::Ptr & out_pointcloud);

  void processOutputAbort();

  void pushOutputNode(ADI3DToFImageStitchingOutputInfo * new_output_node);

  void processOutput();

  void inputReadAbort() { input_read_abort_ = true; }

  ADI3DToFImageStitchingInputInfo * getInputNode();

  template <typename T>
  void addInputNodeToQueue(T * image_stitch_input_info);

private:
  IOutputSensor * output_sensor_;
  int sensor_image_width_;
  int sensor_image_height_;
  int frame_counter_;
  std::string camera_link_;
  sensor_msgs::msg::CameraInfo cam_info_msg_;
  CameraIntrinsics depth_intrinsics_;
  ImageProcUtils * image_proc_utils_[MAX_NUM_DEVICES];
  int enable_pointcloud_generation_;
  int output_sensor_mode_;
  std::string output_file_name_;
  bool enable_autoscaling_;
  bool autoscaling_flag_;
  bool enable_depth_ir_compression_;

  rclcpp::TimerBase::SharedPtr timer_;
  //rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_subscriber_[MAX_NUM_DEVICES];
  message_filters::Subscriber<sensor_msgs::msg::CameraInfo>
    camera_info_subscriber_[MAX_NUM_DEVICES];
  message_filters::Subscriber<sensor_msgs::msg::Image> depth_image_subscriber_[MAX_NUM_DEVICES];
  message_filters::Subscriber<sensor_msgs::msg::Image> ir_image_subscriber_[MAX_NUM_DEVICES];
  message_filters::Subscriber<sensor_msgs::msg::CompressedImage>
    compressed_ir_image_subscriber_[MAX_NUM_DEVICES];
  message_filters::Subscriber<sensor_msgs::msg::CompressedImage>
    compressed_depth_image_subscriber_[MAX_NUM_DEVICES];
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_[MAX_NUM_DEVICES];
  std::shared_ptr<tf2_ros::TransformListener> * tf_listener_[MAX_NUM_DEVICES]{nullptr};
  bool ir_image_recvd_[MAX_NUM_DEVICES];
  bool depth_image_recvd_[MAX_NUM_DEVICES];
  bool tf_recvd_[MAX_NUM_DEVICES];
  float transform_matrix_[MAX_NUM_DEVICES][16];  // a 4x4 matrix for all the sensors
  float camera_yaw_[MAX_NUM_DEVICES];
  geometry_msgs::msg::TransformStamped camera_map_transform_[MAX_NUM_DEVICES];
  float camera_yaw_min_;
  float camera_yaw_max_;
  float camera_yaw_correction_;

  // Defining synchronizers
  std::shared_ptr<sync_CamInfo_2cam> sync_CamInfo_2cam_;
  std::shared_ptr<sync_CamInfo_3cam> sync_CamInfo_3cam_;
  std::shared_ptr<sync_CamInfo_4cam> sync_CamInfo_4cam_;
  std::shared_ptr<sync_depth_ir_2cam> sync_depth_ir_2cam_;
  std::shared_ptr<sync_depth_ir_3cam> sync_depth_ir_3cam_;
  std::shared_ptr<sync_depth_ir_4cam> sync_depth_ir_4cam_;
  std::shared_ptr<sync_compressed_depth_ir_2cam> sync_compressed_depth_ir_2cam_;
  std::shared_ptr<sync_compressed_depth_ir_3cam> sync_compressed_depth_ir_3cam_;
  std::shared_ptr<sync_compressed_depth_ir_4cam> sync_compressed_depth_ir_4cam_;

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr stitched_depth_image_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr stitched_ir_image_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr combo_out_point_cloud_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr stitched_camera_info_publisher_;

  sensor_msgs::msg::PointCloud2 stitched_pc_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr stitched_pc_pcl_;

  rclcpp::Time curr_frame_timestamp_ = rclcpp::Clock{}.now();

  int num_sensors_;

  int input_queue_length_;
  int output_queue_length_;

  bool process_output_thread_abort_ = false;

  std::mutex output_thread_mtx_;
  std::mutex input_thread_mtx_;

  bool input_read_abort_;

  std::queue<ADI3DToFImageStitchingInputInfo *> input_node_queue_;
  std::queue<ADI3DToFImageStitchingOutputInfo *> output_node_queue_;
#ifdef ENABLE_GPU_OPTIMIZATION
  StitchFramesCoreGPU * stitch_frames_core_GPU_;
#else
  StitchFramesCoreCPU * stitch_frames_core_CPU_;
#endif
  /**
   * @brief This function publishes images as Ros messages.
   *
   * @param img This is input image
   * @param encoding_type number of bits used to represent one pixel of image.
   * @param frame_id frame id of image
   * @param publisher This is ros publisher
   */
  void publishImageAsRosMsg(
    const cv::Mat & img, const std::string & encoding_type, const std::string & frame_id,
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher)
  {
    cv_bridge::CvImagePtr cv_ptr(new cv_bridge::CvImage);

    cv_ptr->encoding = encoding_type;
    //cv_ptr->header.seq = frame_counter_;
    cv_ptr->header.stamp = curr_frame_timestamp_;
    cv_ptr->header.frame_id = std::move(frame_id);
    cv_ptr->image = std::move(img);

    publisher->publish(*cv_ptr->toImageMsg());
  }

  /**
   * @brief This image fills and publishes the camera information
   *
   * @param frame_id  frame_id of camera_info
   * @param publisher This is Ros publisher
   */
  void fillAndPublishCameraInfo(
    std::string frame_id,
    const rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr publisher)
  {
    //cam_info_msg_.header.seq = frame_counter_;
    cam_info_msg_.header.stamp = curr_frame_timestamp_;
    cam_info_msg_.header.frame_id = std::move(frame_id);

    cam_info_msg_.width = 4 * sensor_image_width_;
    cam_info_msg_.height = sensor_image_height_;

    cam_info_msg_.distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;

    cam_info_msg_.k.fill(0.0f);
    cam_info_msg_.k[0] = 781.291565f / 2;           // fx
    cam_info_msg_.k[2] = cam_info_msg_.width / 2;   // cx
    cam_info_msg_.k[4] = 781.87738f / 2;            // fy
    cam_info_msg_.k[5] = cam_info_msg_.height / 2;  // cy
    cam_info_msg_.k[8] = 1.0f;

    cam_info_msg_.p.fill(0.0);
    cam_info_msg_.d.resize(8);
    cam_info_msg_.r.fill(0.0f);
    cam_info_msg_.binning_x = 0;
    cam_info_msg_.binning_y = 0;
    cam_info_msg_.roi.do_rectify = false;
    cam_info_msg_.roi.height = 0;
    cam_info_msg_.roi.width = 0;
    cam_info_msg_.roi.x_offset = 0;
    cam_info_msg_.roi.y_offset = 0;

    publisher->publish(cam_info_msg_);
  }

  /**
   * @brief Populate camera Intrinsic data
   * 
   * @param camera_intrinsics - input camera intrinsic
   */
  void populateCameraInstrinsics(CameraIntrinsics & camera_intrinsics)
  {
    camera_intrinsics.camera_matrix[0] = 781.291565f;
    camera_intrinsics.camera_matrix[1] = 0.0f;
    camera_intrinsics.camera_matrix[2] = 520.714905f;
    camera_intrinsics.camera_matrix[3] = 0.0f;
    camera_intrinsics.camera_matrix[4] = 781.87738f;
    camera_intrinsics.camera_matrix[5] = 513.001709f;
    camera_intrinsics.camera_matrix[6] = 0.0f;
    camera_intrinsics.camera_matrix[7] = 0.0f;
    camera_intrinsics.camera_matrix[8] = 1.0f;

    camera_intrinsics.distortion_coeffs[0] = -0.0693829656f;
    camera_intrinsics.distortion_coeffs[1] = 0.115561306f;
    camera_intrinsics.distortion_coeffs[2] = 0.000196631983f;
    camera_intrinsics.distortion_coeffs[3] = -0.00011414945f;
    camera_intrinsics.distortion_coeffs[4] = 0.0944529548f;
    camera_intrinsics.distortion_coeffs[5] = 0.269195855f;
    camera_intrinsics.distortion_coeffs[6] = -0.00811018609f;
    camera_intrinsics.distortion_coeffs[7] = 0.190516844f;

    // Scale
    camera_intrinsics.camera_matrix[0] /= 2;
    camera_intrinsics.camera_matrix[2] /= 2;
    camera_intrinsics.camera_matrix[4] /= 2;
    camera_intrinsics.camera_matrix[5] /= 2;
  }
};

#endif
