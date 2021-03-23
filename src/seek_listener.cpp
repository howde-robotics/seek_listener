// Author(s): Daniel Bronstein (dbronste@andrew.cmu.edu)
// This file is subject to the terms and conditions defined in the file 'LICENSE',
// which is part of this source code package

#include "seek_listener.h"

SeekListener::SeekListener() : private_nh_("~")
{
  initRos();
}

void SeekListener::displayDataCallback(const sensor_msgs::ImageConstPtr& msg)
{
  cv::Mat displayImage = cv_bridge::toCvShare(msg, "bgra8")->image;
  //Do something with the displayImage, which is a gain controlled
  //and colored version of the filtered Data
}

void SeekListener::temperatureDataCallback(const sensor_msgs::ImageConstPtr& msg) 
{
  cv::Mat temperatureData = cv_bridge::toCvShare(msg, "32FC1")->image;
  //Do something with the temperature data, which is an image of float
  //values representing degrees celcius

}

void SeekListener::telemetryDataCallback(const seek_driver::telemetryDataConstPtr& msg)
{
  float env_temp = msg->env_temp;
  unsigned int field_count = msg->field_count;
  std_msgs::Header head = msg->header;
  unsigned short diode_mv = msg->temp_diode_mv;
  unsigned long internal_ts_micros = msg->internal_timestamp_micros;
  //Do something with the telemetry data
}

void SeekListener::filteredDataCallback(const sensor_msgs::ImageConstPtr& msg) 
{
  cv::Mat filteredData = cv_bridge::toCvShare(msg, "mono16")->image;
  //Do something with the filtered data, which is an image of 16 bit
  //values representing partially filter sensor readings.
  //NOTE: These are not gain controlled images, 
  //and NOT temperature readings

}

SeekListener::~SeekListener()
{

}

void SeekListener::initRos()
{
  //subscribers
  image_transport::ImageTransport it(nh_);
  displayImageSub_ = it.subscribe("seek_camera/displayImage", 10, &SeekListener::displayDataCallback, this);
  temperatureImageSub_ = it.subscribe("seek_camera/temperatureImageCelcius", 10, &SeekListener::temperatureDataCallback, this);
  telemetryDataSub_ = nh_.subscribe("seek_camera/telemetry", 10, &SeekListener::telemetryDataCallback, this);
  filteredImageSub_ = it.subscribe("seek_camera/filteredImage", 10, &SeekListener::filteredDataCallback, this);
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "seek_listener");
  SeekListener node;
  while (ros::ok())
    ros::spinOnce();
  return 0;
}
