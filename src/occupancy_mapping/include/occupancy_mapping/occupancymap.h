#ifndef OCCUPANYMAP_H
#define OCCUPANYMAP_H

#include <ros/ros.h>
#include <ros/param.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/LaserScan.h>

#include <nav_msgs/OccupancyGrid.h>

#include <eigen3/Eigen/Eigen>

class OccupancyMap
{
public:

  typedef std::pair<double,double> Point2D;

  OccupancyMap(ros::NodeHandle &nh);

  void laserScanSubs(const sensor_msgs::LaserScanConstPtr &msg);
  double inverseSensorModel();


private:
  ros::NodeHandle nh_;
  sensor_msgs::LaserScan laser_data;


  // map
  double width, height;
  double res;
  Eigen::MatrixXd map;
  Point2D map_center;

};

#endif // OCCUPANYMAP_H
