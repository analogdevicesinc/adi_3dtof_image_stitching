/******************************************************************************
Copyright (c), 2023 - Analog Devices Inc. All Rights Reserved.
This software is PROPRIETARY & CONFIDENTIAL to Analog Devices, Inc.
and its licensors.
******************************************************************************/
#ifndef OUTPUT_SENSOR_FACTORY_H
#define OUTPUT_SENSOR_FACTORY_H

#include "output_sensor_file.h"
#include "output_sensor.h"

/**
 * @brief This is output sensor factory class
 *
 */
class OutputSensorFactory
{
public:
  /**
   * @brief Get the Output Sensor object
   *
   * @param output_sensor_type
   * @return IOutputSensor*
   */
  static IOutputSensor* getOutputSensor(int output_sensor_type)
  {
    switch (output_sensor_type)
    {
      case 0:
        // Camera :Todo
        // return new OutputSensorADTF31XX;
        return nullptr;
      case 1:
        // File
        return new OutputSensorFile;
      case 2:
        // ROS Bag
        ROS_INFO_STREAM("ROSBAG support TBD.");
        return nullptr;
        break;
      default:
        ROS_INFO_STREAM("Not a valid senor type.");
        return nullptr;
    }
  }
};
#endif