#include <ros/ros.h>
#include <occupancy_mapping/occupancymap.h>
#include <sensor_msgs/LaserScan.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "occupancymap_node");
  ros::NodeHandle nh;

  OccupancyMap map_(nh);

  ros::Subscriber laser_subs_ = nh.subscribe("/scan", 10, &OccupancyMap::laserScanSubs, &map_);
  ros::Subscriber imu_subs_ = nh.subscribe("/imu", 10, &OccupancyMap::imuSubs, &map_);
  ros::Subscriber joint_subs_ = nh.subscribe("/joint_states", 10, &OccupancyMap::jointStateSubs, &map_);

  map_.loop();

  return 0;
}
