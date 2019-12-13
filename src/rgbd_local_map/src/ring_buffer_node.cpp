#include <ros/ros.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ring_buffer_node");
  ros::NodeHandle nh;

  ROS_INFO("Hello world!");
}
