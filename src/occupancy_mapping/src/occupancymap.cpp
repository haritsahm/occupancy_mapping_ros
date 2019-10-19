#include "occupancy_mapping/occupancymap.h"

OccupancyMap::OccupancyMap(ros::NodeHandle &nh)
    :nh_(nh)
{
    map_width = map_height = 10;
    nh_.param<int>("map_width", map_width, 5000);
    nh_.param<int>("map_height", map_height, 5000);
    nh_.param<double>("map_res", map_res, 0.01);

    // initialize map
    map = MatrixXd(map_height, map_width);
    map.setOnes();
    map = map*0.5;
    map_center = Point2D(map_height/2, map_width/2);

    map_msgs = nh_.advertise<nav_msgs::OccupancyGrid>("/map_data", 10);
}

void OccupancyMap::laserScanSubs(const sensor_msgs::LaserScanConstPtr &msg)
{
    laser_data = *msg;
}

void OccupancyMap::imuSubs(const sensor_msgs::ImuConstPtr &msg)
{
    imu_ = *msg;
}

void OccupancyMap::jointStateSubs(const sensor_msgs::JointStateConstPtr &msg)
{
    joint_ = *msg;
}

void OccupancyMap::publishData()
{
    nav_msgs::OccupancyGrid oGrid;
    geometry_msgs::Pose origin;
    origin.position.x = (-map_height/2)*map_res; origin.position.y = (-map.rows()/2)*map_res; origin.position.z = 0;
    origin.orientation.w = 1; origin.orientation.x = origin.orientation.y = origin.orientation.z = 0;
    oGrid.header.frame_id = "odom";
    oGrid.header.stamp = ros::Time::now();
    oGrid.info.width = map.cols();
    oGrid.info.height = map.rows();
    oGrid.info.origin = origin;
    oGrid.info.resolution = map_res;

    std::vector<int> vec;
    vec.resize(map.size());
    MatrixXd d = map.array()*100; d = d.array().round();
    MatrixXi m = d.cast<int>();
    //    Eigen::Matrix<int, map.rows(), map.cols(), Eigen::RowMajor>::Map<Eigen::Matrix<int, map.rows(), map.cols(), Eigen::RowMajor>>
    //            (vec.data(), map.rows(), map.cols()) = m;
    Eigen::VectorXi vec_(Eigen::Map<Eigen::VectorXi>(m.data(), map.rows()*map.cols()));
    Eigen::VectorXi::Map(&vec[0], vec_.size()) = vec_;

    oGrid.data.clear();
    oGrid.data.insert(oGrid.data.begin(), vec.begin(), vec.end());
    map_msgs.publish(oGrid);

}

std::vector<Point2D> OccupancyMap::bresenhamLineP(Point2D start, Point2D end)
{

}

std::vector<int> OccupancyMap::bresenhamLine(Point2D start, Point2D end)
{
    double dx = x(end)-x(start);
    double dy = y(end)-y(start);
    double D = 2*dy-dx;
    double y = y(start);

}

double OccupancyMap::inverseSensorModel()
{

}

void OccupancyMap::loop()
{
    ros::Rate r(30);

    while(ros::ok())
    {
        tf::StampedTransform base_transform, laser_transform;

        try{
            base_listener.lookupTransform("odom", "base_link",
                                          ros::Time(0), base_transform);
            scan_listener.lookupTransform("base_link", "base_scan", ros::Time(0), laser_transform);
        }


        catch (tf::TransformException ex){
            ROS_ERROR("%s",ex.what());
            ros::Duration(1.0).sleep();
        }

        publishData();
        ros::spinOnce();
        r.sleep();
    }

    ros::shutdown();

}
