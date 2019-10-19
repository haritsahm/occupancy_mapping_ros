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

typedef std::pair<double,double> Point2D;
inline double x (Point2D &p){return p.first;}
inline double y (Point2D &p){return p.second;}

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
  double inverseSensorModel();
  std::vector<Point2D> bresenhamLineP(Point2D start, Point2D end);
  std::vector<int> bresenhamLine(Point2D start, Point2D end);


  int toIndex(MatrixXd map, Point2D p){return y(p)*map.rows()+x(p);}
  Point2D fromIndex(MatrixXd map, int index)
  {
      Point2D cell;
      cell.first = (int)(index / map.cols());
      cell.second = index % map.cols();
      return cell;
  }


private:
  ros::NodeHandle nh_;
  sensor_msgs::LaserScan laser_data;
  sensor_msgs::Imu imu_;
  sensor_msgs::JointState joint_;

  ros::Publisher map_msgs;

  tf::TransformListener base_listener;
  tf::TransformListener scan_listener;


  // map
  int map_width, map_height;
  double map_res;
  Eigen::MatrixXd map;
  Point2D map_center;

};

#endif // OCCUPANYMAP_H
