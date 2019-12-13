#include "rgbd_local_map/ringbuffer.h"

RingBuffer::RingBuffer(ros::NodeHandle &nh, int pow, double res)
  : nh_(nh),
    it_(nh_)
{
  N_=(1<<pow);
  resolution_=res;

  image_transport::CameraSubscriber depthImageSubs_ = it_.subscribeCamera("camera/depth/image_rect", 100, &RingBuffer::depthImageSubscriber,this);

}


void RingBuffer::depthImageSubscriber(const sensor_msgs::ImageConstPtr& image_msgs, const sensor_msgs::CameraInfoConstPtr& camera_info)
{
  cv_bridge::CvImageConstPtr ptr_depth;

  ptr_depth = cv_bridge::toCvShare(image_msgs, "mono16");
  depth_info_ = *camera_info;
  depth_image_ = ptr_depth->image;
}

void RingBuffer::process()
{
  if(depth_image_.empty())
    return;

  for(std::size_t row=0; row < depth_image_.rows; row++)
    for(std::size_t col=0; col< depth_image_.cols; col++)
    {
      unsigned short d = depth_image_.at<unsigned short>(row,col);
      if (d == 0)
        continue;
//      pcl::PointXYZ p;
//      p.z = double(d) / camera_factor;
//      p.x = (n - camera_cx) * p.z / camera_fx;
//      p.y = (m - camera_cy) * p.z / camera_fy;
//      cloudPtr->points.push_back(p);

    }
}
