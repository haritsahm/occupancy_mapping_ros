#ifndef RINGBUFFER_H
#define RINGBUFFER_H

#include <ros/ros.h>
#include <octomap_ros/conversions.h>
#include <octomap/octomap.h>
#include <cv_bridge/cv_bridge.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>
#include <image_transport/image_transport.h>

#include <eigen3/Eigen/Eigen>
#include <opencv2/imgproc.hpp>

class RingBuffer
{
public:
  RingBuffer(ros::NodeHandle& nh, int pow, double res);

  void process();

  void depthImageSubscriber(const sensor_msgs::ImageConstPtr& image, const sensor_msgs::CameraInfoConstPtr& camera_info);


private:

  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;

  cv::Mat depth_image_;
  sensor_msgs::CameraInfo depth_info_;

  int N_; //pow
  double resolution_;

  Eigen::Vector4d offset_;
};

#endif // RINGBUFFER_H
