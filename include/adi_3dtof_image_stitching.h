/******************************************************************************
Copyright (c), 2023 - Analog Devices Inc. All Rights Reserved.
This software is PROPRIETARY & CONFIDENTIAL to Analog Devices, Inc.
and its licensors.
******************************************************************************/
#ifndef ADI_3DTOF_IMAGE_STITCH_H
#define ADI_3DTOF_IMAGE_STITCH_H

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
#include <boost/make_shared.hpp>
#include <cv_bridge/cv_bridge.h>
#include <eigen_conversions/eigen_msg.h>
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
#include <pcl_ros/transforms.h>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/distortion_models.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <std_msgs/Bool.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <utility>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>

#include <boost/chrono.hpp>
#include <boost/thread/thread.hpp>
#include <queue>

#define MAX_QUEUE_SIZE_FOR_TIME_SYNC 10

namespace enc = sensor_msgs::image_encodings;

// Declaring synchronization policies
typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::CameraInfo, sensor_msgs::CameraInfo>
    Sync_CameraInfo_2sensors;
typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::CameraInfo, sensor_msgs::CameraInfo,
                                                        sensor_msgs::CameraInfo>
    Sync_CameraInfo_3sensors;
typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::CameraInfo, sensor_msgs::CameraInfo,
                                                        sensor_msgs::CameraInfo, sensor_msgs::CameraInfo>
    Sync_CameraInfo_4sensors;
typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image,
                                                        sensor_msgs::Image>
    sync_depth_ir_2;
typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::CompressedImage, sensor_msgs::CompressedImage,
                                                        sensor_msgs::CompressedImage, sensor_msgs::CompressedImage>
    sync_compressed_depth_ir_2;
typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image,
                                                        sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image>
    sync_depth_ir_3;
typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::CompressedImage, sensor_msgs::CompressedImage,
                                                        sensor_msgs::CompressedImage, sensor_msgs::CompressedImage,
                                                        sensor_msgs::CompressedImage, sensor_msgs::CompressedImage>
    sync_compressed_depth_ir_3;
typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image,
                                                        sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image,
                                                        sensor_msgs::Image, sensor_msgs::Image>
    sync_depth_ir_4;
typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::CompressedImage, sensor_msgs::CompressedImage,
                                                        sensor_msgs::CompressedImage, sensor_msgs::CompressedImage,
                                                        sensor_msgs::CompressedImage, sensor_msgs::CompressedImage,
                                                        sensor_msgs::CompressedImage, sensor_msgs::CompressedImage>
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
class ADI3DToFImageStitching : public ros::NodeHandle
{
  typedef void (ADI3DToFImageStitching::*ImageCallbackFuncPointer)(const sensor_msgs::ImageConstPtr&);
  typedef void (ADI3DToFImageStitching::*ObjectDetectCallbackFuncPointer)(const std_msgs::BoolConstPtr&);
  typedef void (ADI3DToFImageStitching::*PointCloudCallbackFuncPointer)(const sensor_msgs::PointCloud2ConstPtr&);

public:
  static const int MAX_NUM_DEVICES = 4;
  bool camera_parameters_updated_[MAX_NUM_DEVICES] = { false };
  float vertical_fov_in_degrees_ = 75.0f;
  int out_image_height_ = 512;
  int out_image_width_ = 512 * MAX_NUM_DEVICES;

  /**
   * @brief Construct a new ADI3DToFImageStitching object
   *
   */
  ADI3DToFImageStitching()
  {
    ROS_INFO("adi_3dtof_image_stitching::Inside ADI3DToFImageStitching()");
    ros::NodeHandle nh("~");

    // Get Parameters
    nh.param<int>("param_enable_pointcloud_generation", enable_pointcloud_generation_, 2);
    nh.param<int>("param_enable_compressed_data_input", enable_compressed_data_input_, 0);
    nh.param<int>("param_output_mode", output_sensor_mode_, 0);
    nh.param<std::string>("param_camera_link", camera_link_, "adi_camera_link");
    nh.param<std::string>("param_out_file_name", output_file_name_, "stitched_output.avi");

    // Getting Camera prefixes
    std::vector<std::string> cam_prefix;
    XmlRpc::XmlRpcValue cam_prefix_arr;
    nh.param("param_camera_prefixes", cam_prefix_arr, cam_prefix_arr);
    std::vector<std::string> cam_prefix_1;
    for (int i = 0; i < cam_prefix_arr.size(); i++)
    {
      cam_prefix.push_back(cam_prefix_arr[i]);
      std::cerr << "camera_prefixes: " << cam_prefix[i] << std::endl;
    }
    // Getting projection parameters
    nh.param<float>("param_vertical_fov_in_degrees", vertical_fov_in_degrees_, 75.0);
    nh.param<int>("param_out_image_height", out_image_height_, 512);
    nh.param<int>("param_out_image_width", out_image_width_, 2048);

    //@todo: Image width and height is hard coded.
    sensor_image_width_ = 512;
    sensor_image_height_ = 512;

    // init
    frame_counter_ = 0;

    // Do not allow more than the max cameras supported.
    int max_devices_allowed = static_cast<int>(sizeof(depth_image_subscriber_) / sizeof(depth_image_subscriber_[0]));
    num_sensors_ = std::min(max_devices_allowed, static_cast<int>(cam_prefix.size()));
    input_read_abort_ = false;

    for (int i = 0; i < num_sensors_; i++)
    {

      // Synchronized subscribers
      camera_info_subscriber_[i].subscribe(nh, "/" + cam_prefix[i] + "/camera_info", 5);
      if (enable_compressed_data_input_ == 1)
      {
        compressed_ir_image_subscriber_[i].subscribe(nh, "/" + cam_prefix[i] + "/compressed_ir_image", 5);
        compressed_depth_image_subscriber_[i].subscribe(nh, "/" + cam_prefix[i] + "/compressed_depth_image", 5);
      }
      else
      {
        ir_image_subscriber_[i].subscribe(nh, "/" + cam_prefix[i] + "/ir_image", 5);
        depth_image_subscriber_[i].subscribe(nh, "/" + cam_prefix[i] + "/depth_image", 5);
      }

      // Create TF listerner instance
      tf_listener_[i] = new tf2_ros::TransformListener(tf_buffer_[i]);

      populateCameraInstrinsics(depth_intrinsics_);
      image_proc_utils_[i] = new ImageProcUtils(&depth_intrinsics_, sensor_image_width_, sensor_image_height_);

      // Different flags
      ir_image_recvd_[i] = false;
      depth_image_recvd_[i] = false;
      tf_recvd_[i] = false;
    }

    // Get output sensor module
    output_sensor_ = OutputSensorFactory::getOutputSensor(output_sensor_mode_);
    /**
     * @todo Add Exception handling codes
     *
     */
    if (output_sensor_ != nullptr)
    {
      // TODO: check how to better pass the output image width and height, currently
      // the output size values an be changed from main function after initialization.
      output_sensor_->open(output_file_name_, out_image_width_,
                           out_image_height_);  
    }

    if (num_sensors_ == 2)
    {
      try {
        sync_CamInfo_2cam_.reset(new sync_CamInfo_2cam(Sync_CameraInfo_2sensors(MAX_QUEUE_SIZE_FOR_TIME_SYNC),
                                                      camera_info_subscriber_[0], camera_info_subscriber_[1]));
        sync_CamInfo_2cam_->registerCallback(boost::bind(&ADI3DToFImageStitching::sync2CamerasCamInfoCallback, this, _1, _2));

        if (enable_pointcloud_generation_ == 1 || enable_pointcloud_generation_ == 2)
        {
          if (enable_compressed_data_input_ == 1)
          {
            sync_compressed_depth_ir_2cam_.reset(new sync_compressed_depth_ir_2cam(
                sync_compressed_depth_ir_2(MAX_QUEUE_SIZE_FOR_TIME_SYNC), compressed_depth_image_subscriber_[0],
                compressed_ir_image_subscriber_[0], compressed_depth_image_subscriber_[1],
                compressed_ir_image_subscriber_[1]));
            sync_compressed_depth_ir_2cam_->registerCallback(
                boost::bind(&ADI3DToFImageStitching::sync2CamerasCompressedDepthIRImageCallback, this, _1, _2, _3, _4));
          }
          else
          {
            sync_depth_ir_2cam_.reset(new sync_depth_ir_2cam(sync_depth_ir_2(MAX_QUEUE_SIZE_FOR_TIME_SYNC),
                                                            depth_image_subscriber_[0], ir_image_subscriber_[0],
                                                            depth_image_subscriber_[1], ir_image_subscriber_[1]));
            sync_depth_ir_2cam_->registerCallback(
                boost::bind(&ADI3DToFImageStitching::sync2CamerasDepthIRImageCallback, this, _1, _2, _3, _4));
          }
        }
      }
      catch (...)  {
        printf("\n ERROR: Exception while registering Synchronizers \n");
      }
    }
    else if (num_sensors_ == 3)
    {
      try {
        sync_CamInfo_3cam_.reset(new sync_CamInfo_3cam(Sync_CameraInfo_3sensors(MAX_QUEUE_SIZE_FOR_TIME_SYNC),
                                                      camera_info_subscriber_[0], camera_info_subscriber_[1],
                                                      camera_info_subscriber_[2]));
        sync_CamInfo_3cam_->registerCallback(boost::bind(&ADI3DToFImageStitching::sync3CamerasCamInfoCallback, this, _1, _2, _3));

        if (enable_pointcloud_generation_ == 1 || enable_pointcloud_generation_ == 2)
        {
          if (enable_compressed_data_input_ == 1)
          {
            sync_compressed_depth_ir_3cam_.reset(new sync_compressed_depth_ir_3cam(
                sync_compressed_depth_ir_3(MAX_QUEUE_SIZE_FOR_TIME_SYNC), compressed_depth_image_subscriber_[0],
                compressed_ir_image_subscriber_[0], compressed_depth_image_subscriber_[1],
                compressed_ir_image_subscriber_[1], compressed_depth_image_subscriber_[2],
                compressed_ir_image_subscriber_[2]));
            sync_compressed_depth_ir_3cam_->registerCallback(
                boost::bind(&ADI3DToFImageStitching::sync3CamerasCompressedDepthIRImageCallback, this, _1, _2, _3, _4, _5, _6));
          }
          else
          {
            sync_depth_ir_3cam_.reset(new sync_depth_ir_3cam(sync_depth_ir_3(MAX_QUEUE_SIZE_FOR_TIME_SYNC),
                                                            depth_image_subscriber_[0], ir_image_subscriber_[0],
                                                            depth_image_subscriber_[1], ir_image_subscriber_[1],
                                                            depth_image_subscriber_[2], ir_image_subscriber_[2]));
            sync_depth_ir_3cam_->registerCallback(
                boost::bind(&ADI3DToFImageStitching::sync3CamerasDepthIRImageCallback, this, _1, _2, _3, _4, _5, _6));
          }
        }
      }
      catch (...)  {
        printf("\n ERROR: Exception while registering Synchronizers \n");
      }
    }
    else if (num_sensors_ == 4)
    {
      try {
        sync_CamInfo_4cam_.reset(new sync_CamInfo_4cam(Sync_CameraInfo_4sensors(MAX_QUEUE_SIZE_FOR_TIME_SYNC),
                                                      camera_info_subscriber_[0], camera_info_subscriber_[1],
                                                      camera_info_subscriber_[2], camera_info_subscriber_[3]));
        sync_CamInfo_4cam_->registerCallback(
            boost::bind(&ADI3DToFImageStitching::sync4CamerasCamInfoCallback, this, _1, _2, _3, _4));
        // We need depth image for point-cloud generation

        if (enable_pointcloud_generation_ == 1 || enable_pointcloud_generation_ == 2)
        {
          if (enable_compressed_data_input_ == 1)
          {
            sync_compressed_depth_ir_4cam_.reset(new sync_compressed_depth_ir_4cam(
                sync_compressed_depth_ir_4(MAX_QUEUE_SIZE_FOR_TIME_SYNC), compressed_depth_image_subscriber_[0],
                compressed_ir_image_subscriber_[0], compressed_depth_image_subscriber_[1],
                compressed_ir_image_subscriber_[1], compressed_depth_image_subscriber_[2],
                compressed_ir_image_subscriber_[2], compressed_depth_image_subscriber_[3],
                compressed_ir_image_subscriber_[3]));
            sync_compressed_depth_ir_4cam_->registerCallback(boost::bind(
                &ADI3DToFImageStitching::sync4CamerasCompressedDepthIRImageCallback, this, _1, _2, _3, _4, _5, _6, _7, _8));
          }
          else
          {
            sync_depth_ir_4cam_.reset(new sync_depth_ir_4cam(
                sync_depth_ir_4(MAX_QUEUE_SIZE_FOR_TIME_SYNC), depth_image_subscriber_[0], ir_image_subscriber_[0],
                depth_image_subscriber_[1], ir_image_subscriber_[1], depth_image_subscriber_[2], ir_image_subscriber_[2],
                depth_image_subscriber_[3], ir_image_subscriber_[3]));
            sync_depth_ir_4cam_->registerCallback(
                boost::bind(&ADI3DToFImageStitching::sync4CamerasDepthIRImageCallback, this, _1, _2, _3, _4, _5, _6, _7, _8));
          }
        }
      }
      catch (...)  {
        printf("\n ERROR: Exception while registering Synchronizers \n");
      }
    }

    // Create publishers.
    combo_out_point_cloud_publisher_ = this->advertise<sensor_msgs::PointCloud2>("point_cloud", 10);
    stitched_depth_image_publisher_ = this->advertise<sensor_msgs::Image>("depth_image", 10);
    stitched_ir_image_publisher_ = this->advertise<sensor_msgs::Image>("ir_image", 10);
    stitched_camera_info_publisher_ = this->advertise<sensor_msgs::CameraInfo>("camera_info", 10);

    // Initialize stitched point cloud buffer
    stitched_pc_pcl_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    stitched_pc_pcl_->points.resize(out_image_width_ * out_image_height_);

#ifdef ENABLE_GPU_OPTIMIZATION
    // GPU class init
    stitch_frames_core_GPU_ = new StitchFramesCoreGPU(sensor_image_width_, sensor_image_height_, out_image_width_,
                                                      out_image_height_, num_sensors_, vertical_fov_in_degrees_);
#else
     stitch_frames_core_CPU_ = new StitchFramesCoreCPU(sensor_image_width_, sensor_image_height_, out_image_width_,
                                                      out_image_height_, num_sensors_, vertical_fov_in_degrees_); 
#endif

    input_queue_length_ = 2;
    output_queue_length_ = 2;
  }

  /**
   * @brief This function shuts down all the nodes running in a roscore
   *
   */
  void shutDownAllNodes()
  {
    int status = system("rosnode kill -a");
    if (status < 0)
    {
      ROS_INFO_STREAM("Error in \"rosnode kill -a\": " << status);
    }
    ros::shutdown();
  }

  /**
   * @brief Destroy the ADI3DToFImageStitching object
   *
   */
  ~ADI3DToFImageStitching()
  {
    for (int i = 0; i < num_sensors_; i++)
    {
      delete tf_listener_[i];
      delete image_proc_utils_[i];
    }

    // Close outputs
    if (output_sensor_ != nullptr)
    {
      output_sensor_->close();
    }
  }
  /**
   * @brief COnvery XYZ 3D fram to ROS point cloud message format
   * 
   * @param xyz_frame - input xyz frame
   * @param sensor_header - sensor header data
   * @param img_width - xyz frame width
   * @param img_height - xyz frame height
   * @return sensor_msgs::PointCloud2::Ptr 
   */
  sensor_msgs::PointCloud2::Ptr convert2ROSPointCloudMsg(short* xyz_frame, std_msgs::Header sensor_header,
                                                         int img_width, int img_height)
  {
    sensor_msgs::PointCloud2::Ptr pointcloud_msg(new sensor_msgs::PointCloud2);

    pointcloud_msg->header.seq = frame_counter_;
    pointcloud_msg->header.stamp = sensor_header.stamp;
    pointcloud_msg->header.frame_id = sensor_header.frame_id;
    pointcloud_msg->width = img_width;
    pointcloud_msg->height = img_height;
    pointcloud_msg->is_dense = false;
    pointcloud_msg->is_bigendian = false;

    // XYZ data from sensor.
    // This data is in 16 bpp format.
    short* xyz_sensor_buf;
    xyz_sensor_buf = xyz_frame;
    sensor_msgs::PointCloud2Modifier pcd_modifier(*pointcloud_msg);
    pcd_modifier.setPointCloud2FieldsByString(1, "xyz");

    sensor_msgs::PointCloud2Iterator<float> iter_x(*pointcloud_msg, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(*pointcloud_msg, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(*pointcloud_msg, "z");
    // TODO:Parallelize this below loop with openmp
    for (int i = 0; i < img_height; i++)
    {
      for (int j = 0; j < img_width; j++)
      {
        *iter_x = (float)(*xyz_sensor_buf++) / 1000.0f;
        *iter_y = (float)(*xyz_sensor_buf++) / 1000.0f;
        *iter_z = (float)(*xyz_sensor_buf++) / 1000.0f;
        ++iter_x;
        ++iter_y;
        ++iter_z;
      }
    }

    return pointcloud_msg;
  }

  void sync2CamerasDepthIRImageCallback(const sensor_msgs::ImageConstPtr& depth_image_cam1,
                                        const sensor_msgs::ImageConstPtr& ir_image_cam1,
                                        const sensor_msgs::ImageConstPtr& depth_image_cam2,
                                        const sensor_msgs::ImageConstPtr& ir_image_cam2);

  void sync3CamerasDepthIRImageCallback(const sensor_msgs::ImageConstPtr& depth_image_cam1,
                                        const sensor_msgs::ImageConstPtr& ir_image_cam1,
                                        const sensor_msgs::ImageConstPtr& depth_image_cam2,
                                        const sensor_msgs::ImageConstPtr& ir_image_cam2,
                                        const sensor_msgs::ImageConstPtr& depth_image_cam3,
                                        const sensor_msgs::ImageConstPtr& ir_image_cam3);

  void sync4CamerasDepthIRImageCallback(
      const sensor_msgs::ImageConstPtr& depth_image_cam1, const sensor_msgs::ImageConstPtr& ir_image_cam1,
      const sensor_msgs::ImageConstPtr& depth_image_cam2, const sensor_msgs::ImageConstPtr& ir_image_cam2,
      const sensor_msgs::ImageConstPtr& depth_image_cam3, const sensor_msgs::ImageConstPtr& ir_image_cam3,
      const sensor_msgs::ImageConstPtr& depth_image_cam4, const sensor_msgs::ImageConstPtr& ir_image_cam4);

  void
  sync2CamerasCompressedDepthIRImageCallback(const sensor_msgs::CompressedImageConstPtr& compressed_depth_image_cam1,
                                             const sensor_msgs::CompressedImageConstPtr& compressed_ir_image_cam1,
                                             const sensor_msgs::CompressedImageConstPtr& compressed_depth_image_cam2,
                                             const sensor_msgs::CompressedImageConstPtr& compressed_ir_image_cam2);

  void
  sync3CamerasCompressedDepthIRImageCallback(const sensor_msgs::CompressedImageConstPtr& compressed_depth_image_cam1,
                                             const sensor_msgs::CompressedImageConstPtr& compressed_ir_image_cam1,
                                             const sensor_msgs::CompressedImageConstPtr& compressed_depth_image_cam2,
                                             const sensor_msgs::CompressedImageConstPtr& compressed_ir_image_cam2,
                                             const sensor_msgs::CompressedImageConstPtr& compressed_depth_image_cam3,
                                             const sensor_msgs::CompressedImageConstPtr& compressed_ir_image_cam3);

  void
  sync4CamerasCompressedDepthIRImageCallback(const sensor_msgs::CompressedImageConstPtr& compressed_depth_image_cam1,
                                             const sensor_msgs::CompressedImageConstPtr& compressed_ir_image_cam1,
                                             const sensor_msgs::CompressedImageConstPtr& compressed_depth_image_cam2,
                                             const sensor_msgs::CompressedImageConstPtr& compressed_ir_image_cam2,
                                             const sensor_msgs::CompressedImageConstPtr& compressed_depth_image_cam3,
                                             const sensor_msgs::CompressedImageConstPtr& compressed_ir_image_cam3,
                                             const sensor_msgs::CompressedImageConstPtr& compressed_depth_image_cam4,
                                             const sensor_msgs::CompressedImageConstPtr& compressed_ir_image_cam4);

  void sync2CamerasCamInfoCallback(const sensor_msgs::CameraInfoConstPtr& CameraInfo_cam1,
                                   const sensor_msgs::CameraInfoConstPtr& CameraInfo_cam2);

  void sync3CamerasCamInfoCallback(const sensor_msgs::CameraInfoConstPtr& CameraInfo_cam1,
                                   const sensor_msgs::CameraInfoConstPtr& CameraInfo_cam2,
                                   const sensor_msgs::CameraInfoConstPtr& CameraInfo_cam3);

  void sync4CamerasCamInfoCallback(const sensor_msgs::CameraInfoConstPtr& CameraInfo_cam1,
                                   const sensor_msgs::CameraInfoConstPtr& CameraInfo_cam2,
                                   const sensor_msgs::CameraInfoConstPtr& CameraInfo_cam3,
                                   const sensor_msgs::CameraInfoConstPtr& CameraInfo_cam4);

  void camInfoCallback(const sensor_msgs::CameraInfoConstPtr& cam_info, int cam_id);

  void depthImageCallback(const sensor_msgs::ImageConstPtr& depth_image, int cam_id,
                          ADI3DToFImageStitchingInputInfo* image_stitch_input_info);

  void compresseddepthImageCallback(const sensor_msgs::CompressedImageConstPtr& depth_image, int cam_id,
                                    ADI3DToFImageStitchingInputInfo* image_stitch_input_info);

  void irImageCallback(const sensor_msgs::ImageConstPtr& ir_image, int cam_id,
                       ADI3DToFImageStitchingInputInfo* image_stitch_input_info);

  void compressedirImageCallback(const sensor_msgs::CompressedImageConstPtr& ir_image, int cam_id,
                                 ADI3DToFImageStitchingInputInfo* image_stitch_input_info);

  bool stitchFrames();

  void generatePointCloud(float* xyz_frame, int* lut_3d_to_2d_mapping, const pcl::PointCloud<pcl::PointXYZ>::Ptr& out_pointcloud);

  void processOutputAbort();

  void pushOutputNode(ADI3DToFImageStitchingOutputInfo* new_output_node);

  void processOutput();

  void inputReadAbort()
  {
    input_read_abort_ = true;
  }

  ADI3DToFImageStitchingInputInfo* getInputNode();

  template <typename T>
  void addInputNodeToQueue(T* image_stitch_input_info);

private:
  IOutputSensor* output_sensor_;
  int sensor_image_width_;
  int sensor_image_height_;
  int frame_counter_;
  std::string camera_link_;
  sensor_msgs::CameraInfo cam_info_msg_;
  CameraIntrinsics depth_intrinsics_;
  ImageProcUtils* image_proc_utils_[MAX_NUM_DEVICES];
  int enable_pointcloud_generation_;
  int output_sensor_mode_;
  int enable_compressed_data_input_;
  std::string output_file_name_;

  message_filters::Subscriber<sensor_msgs::CameraInfo> camera_info_subscriber_[MAX_NUM_DEVICES];
  message_filters::Subscriber<sensor_msgs::Image> ir_image_subscriber_[MAX_NUM_DEVICES];
  message_filters::Subscriber<sensor_msgs::Image> depth_image_subscriber_[MAX_NUM_DEVICES];
  message_filters::Subscriber<sensor_msgs::CompressedImage> compressed_ir_image_subscriber_[MAX_NUM_DEVICES];
  message_filters::Subscriber<sensor_msgs::CompressedImage> compressed_depth_image_subscriber_[MAX_NUM_DEVICES];
  tf2_ros::Buffer tf_buffer_[MAX_NUM_DEVICES];
  tf2_ros::TransformListener* tf_listener_[MAX_NUM_DEVICES];
  bool ir_image_recvd_[MAX_NUM_DEVICES];
  bool depth_image_recvd_[MAX_NUM_DEVICES];
  bool tf_recvd_[MAX_NUM_DEVICES];

  // Defining synchronizers
  boost::shared_ptr<sync_CamInfo_2cam> sync_CamInfo_2cam_;
  boost::shared_ptr<sync_CamInfo_3cam> sync_CamInfo_3cam_;
  boost::shared_ptr<sync_CamInfo_4cam> sync_CamInfo_4cam_;
  boost::shared_ptr<sync_depth_ir_2cam> sync_depth_ir_2cam_;
  boost::shared_ptr<sync_depth_ir_3cam> sync_depth_ir_3cam_;
  boost::shared_ptr<sync_depth_ir_4cam> sync_depth_ir_4cam_;
  boost::shared_ptr<sync_compressed_depth_ir_2cam> sync_compressed_depth_ir_2cam_;
  boost::shared_ptr<sync_compressed_depth_ir_3cam> sync_compressed_depth_ir_3cam_;
  boost::shared_ptr<sync_compressed_depth_ir_4cam> sync_compressed_depth_ir_4cam_;

  ros::Publisher stitched_depth_image_publisher_;
  ros::Publisher stitched_ir_image_publisher_;
  ros::Publisher combo_out_point_cloud_publisher_;
  ros::Publisher stitched_camera_info_publisher_;

  sensor_msgs::PointCloud2 stitched_pc_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr stitched_pc_pcl_;

  ros::Time curr_frame_timestamp_ = ros::Time::now();

  int num_sensors_;

  int input_queue_length_;
  int output_queue_length_;

  bool process_output_thread_abort_ = false;

  boost::mutex output_thread_mtx_;
  boost::mutex input_thread_mtx_;

  bool input_read_abort_;

  std::queue<ADI3DToFImageStitchingInputInfo*> input_node_queue_;
  std::queue<ADI3DToFImageStitchingOutputInfo*> output_node_queue_;
#ifdef ENABLE_GPU_OPTIMIZATION
  StitchFramesCoreGPU* stitch_frames_core_GPU_;
#else
  StitchFramesCoreCPU* stitch_frames_core_CPU_;
#endif
  /**
   * @brief This function publishes images as Ros messages.
   *
   * @param img This is input image
   * @param encoding_type number of bits used to represent one pixel of image.
   * @param frame_id frame id of image
   * @param publisher This is ros publisher
   */
  void publishImageAsRosMsg(cv::Mat img, const std::string& encoding_type, std::string frame_id,
                            const ros::Publisher& publisher)
  {
    cv_bridge::CvImagePtr cv_ptr(new cv_bridge::CvImage);

    cv_ptr->encoding = encoding_type;
    cv_ptr->header.seq = frame_counter_;
    cv_ptr->header.stamp = curr_frame_timestamp_;
    cv_ptr->header.frame_id = std::move(frame_id);
    cv_ptr->image = std::move(img);

    publisher.publish(cv_ptr->toImageMsg());
  }

  /**
   * @brief This image fills and publishes the camera information
   *
   * @param frame_id  frame_id of camera_info
   * @param publisher This is Ros publisher
   */
  void fillAndPublishCameraInfo(std::string frame_id, const ros::Publisher& publisher)
  {
    cam_info_msg_.header.seq = frame_counter_;
    cam_info_msg_.header.stamp = curr_frame_timestamp_;
    cam_info_msg_.header.frame_id = std::move(frame_id);

    cam_info_msg_.width = 4 * sensor_image_width_;
    cam_info_msg_.height = sensor_image_height_;

    cam_info_msg_.distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;

    cam_info_msg_.K.fill(0.0f);
    cam_info_msg_.K[0] = 781.291565f / 2;           // fx
    cam_info_msg_.K[2] = cam_info_msg_.width / 2;   // cx
    cam_info_msg_.K[4] = 781.87738f / 2;            // fy
    cam_info_msg_.K[5] = cam_info_msg_.height / 2;  // cy
    cam_info_msg_.K[8] = 1.0f;

    cam_info_msg_.P.fill(0.0);
    cam_info_msg_.D.resize(8);
    cam_info_msg_.R.fill(0.0f);
    cam_info_msg_.binning_x = 0;
    cam_info_msg_.binning_y = 0;
    cam_info_msg_.roi.do_rectify = false;
    cam_info_msg_.roi.height = 0;
    cam_info_msg_.roi.width = 0;
    cam_info_msg_.roi.x_offset = 0;
    cam_info_msg_.roi.y_offset = 0;

    publisher.publish(cam_info_msg_);
  }
  
  /**
   * @brief Populate camera Intrinsic data
   * 
   * @param camera_intrinsics - input camera intrinsic
   */
  void populateCameraInstrinsics(CameraIntrinsics& camera_intrinsics)
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
