#ifndef OCCUPANYMAP_H
#define OCCUPANYMAP_H

#include <ros/ros.h>
#include <ros/param.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/JointState.h>

#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/MapMetaData.h>

#include <eigen3/Eigen/Eigen>
#include <mutex>
#include <math.h>
#include <pthread.h>
#include "properties.h"
#include <boost/thread.hpp>

class OccupancyMap
{
public:
  typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> MatrixXd;
  typedef Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> MatrixXi;

  OccupancyMap(ros::NodeHandle &nh);

  void loop();
  void publishData();

  void laserScanSubs(const sensor_msgs::LaserScanConstPtr &msg);
  void imuSubs(const sensor_msgs::ImuConstPtr &msg);
  void jointStateSubs(const sensor_msgs::JointStateConstPtr &msg);

  void updateMap();
  std::vector<Point2D> bresenhamLineP(Point2D start, Point2D end);
  std::vector<int> bresenhamLine(Point2D start, Point2D end);
  Point2D getLaserPos(int &index, double &dist);
  double ProbLog(double x);
  double loggOddRatio(bool occ);

  int toIndex(MatrixXd map, Point2D p){return (int)p.y*map.cols()+p.x;}
  Point2D fromIndex(MatrixXd map, int index)
  {
      Point2D cell;
      cell.x = index % map.cols();
      cell.y = (int)floor(index / map.cols());
      return cell;
  }

  double clip(double val, double upper=1, double lower=0)
  {
      if(val > upper) return upper;
      if(val < lower) return lower;
      return val;
  }


private:
  ros::NodeHandle nh_;
  sensor_msgs::LaserScan laser_data;
  sensor_msgs::Imu imu_;
  sensor_msgs::JointState joint_;

  ros::Publisher map_msgs;

  tf::TransformListener base_listener;
  tf::TransformListener scan_listener;
  tf::TransformListener listener;

  Eigen::Affine3d base_, scanner, laser_ref;

  std::mutex mutex_;
  bool get_laser;

  // map
  int map_width, map_height;
  double map_res;
  Eigen::MatrixXd map, logs_map;
  std::vector<double> map_v, logs_map_v;
  Point2D map_center;
  double lo, l_occ, l_free;
  bool data_updated;

  nav_msgs::OccupancyGrid oGrid;
  geometry_msgs::Pose origin;

};

#endif // OCCUPANYMAP_H
