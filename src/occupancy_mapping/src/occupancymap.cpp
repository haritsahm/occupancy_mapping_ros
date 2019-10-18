#include "occupancy_mapping/occupancymap.h"

OccupancyMap::OccupancyMap(ros::NodeHandle &nh)
  :nh_(nh)
{
  nh_.param("map_width", width, 5000);
  nh_.param("map_height", height, 5000);
  nh_.param("map_res", res, 0.01);

}

void OccupancyMap::laserScanSubs(const sensor_msgs::LaserScanConstPtr &msg)
{

}

double OccupancyMap::inverseSensorModel()
{

}
