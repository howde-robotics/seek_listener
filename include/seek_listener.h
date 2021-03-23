// Author(s): Daniel Bronstein (dbronste@andrew.cmu.edu)
// This file is subject to the terms and conditions defined in the file 'LICENSE',
// which is part of this source code package

#pragma once

// ROS libraries
#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>  
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include "seek_driver/telemetryData.h"

// Include other header files
#include <opencv2/opencv.hpp>


class SeekListener
{
public:

  SeekListener();

  ~SeekListener();

private:
  /////////////////////// State variables ///////////////////////

  /////////////////////// calculation variables ///////////////////////

  void displayDataCallback(const sensor_msgs::ImageConstPtr& msg);
  void temperatureDataCallback(const sensor_msgs::ImageConstPtr& msg);
  void filteredDataCallback(const sensor_msgs::ImageConstPtr& msg);
  void telemetryDataCallback(const seek_driver::telemetryDataConstPtr& msg);

  // ROS and node related variables
  ros::NodeHandle nh_, private_nh_;

  image_transport::Subscriber displayImageSub_;//uses cv_bridge
  image_transport::Subscriber filteredImageSub_;//uses cv_bridge
  image_transport::Subscriber temperatureImageSub_;//cv bridge
  ros::Subscriber telemetryDataSub_;//custom message

  // Boiler plate ROS functions
  void initRos();
  void timerCallback(const ros::TimerEvent& e);

};
