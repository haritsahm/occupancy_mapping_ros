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
    origin.position.x = -(map_height/2)*map_res; origin.position.y = -(map.rows()/2)*map_res; origin.position.z = 0;
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
    // https://github.com/kghose/moulick/blob/master/fractional-bresenham.ipynb
    int x0 = int(round(start.x));
    int y0 = int(round(start.y));
    int x1 = int(round(end.x));
    int y1 = int(round(end.y));

    int dx = abs(x1-x0);
    int sx = x0 < x1 ? 1 : -1;
    int dy = -abs(y1-y0);
    int sy = y0 < y1 ? 1 :-1;

    int err = dx+dy;
    int e2 = 0;

    std::vector<Point2D> points;

    while(1)
    {
        points.push_back(Point2D(x0, y0));
        if((x0 == x1) && (y0 == y1)) break;
        e2 = 2*err;
        if(e2>=dy)
        {
            err+=dy;
            x0+=sx;
        }
        if(e2<=dx)
        {
            err+=dx;
            y0+=sy;
        }
    }

    return points;

}

std::vector<int> OccupancyMap::bresenhamLine(Point2D start, Point2D end)
{
    // https://github.com/kghose/moulick/blob/master/fractional-bresenham.ipynb
    int x0 = int(round(start.x));
    int y0 = int(round(start.y));
    int x1 = int(round(end.x));
    int y1 = int(round(end.y));

    int dx = abs(x1-x0);
    int sx = x0 < x1 ? 1 : -1;
    int dy = -abs(y1-y0);
    int sy = y0 < y1 ? 1 :-1;

    int err = dx+dy;
    int e2 = 0;

    std::vector<int> indexes;

    while(1)
    {
        indexes.push_back(toIndex(map, Point2D(x0, y0)));
        if((x0 == x1) && (y0 == y1)) break;
        e2 = 2*err;
        if(e2>=dy)
        {
            err+=dy;
            x0+=sx;
        }
        if(e2<=dx)
        {
            err+=dx;
            y0+=sy;
        }
    }

    return indexes;


}

Point2D OccupancyMap::getLaserPos(int &index, double &dist)
{
    double angle= laser_data.angle_increment*index;

    Point2D p;
    p.x = cos(angle)*dist;
    p.y = sin(angle)*dist;

    return p;
}

double OccupancyMap::inverseSensorModel()
{

    for(int index=0; index < laser_data.ranges.size(); index++)
    {
        double dist = laser_data.ranges[index];
//        if(dist == 0) continue;

        Point2D l_pos = getLaserPos(index, dist); //respect to laser_base

        //sensor Pos w.r.t odom
        Eigen::Vector3d laser_pos = scanner.translation();
        Eigen::Matrix3d rot = scanner.linear();
        Eigen::Vector3d laser_rpy = convertRotationToRPY(rot);

        double c = cos(laser_rpy.z());
        double s = sin(laser_rpy.z());

        Point2D l_pos_w; //laser w.r.t odom
        l_pos_w.x= c*l_pos.x -s*l_pos.y + laser_pos.x;
        l_pos_w.y= s*l_pos.x +c*l_pos.y + laser_pos.y;


    }



    return 0;
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
            scan_listener.lookupTransform("odom", "base_scan", ros::Time(0), laser_transform);
        }


        catch (tf::TransformException ex){
            ROS_ERROR("%s",ex.what());
            ros::Duration(1.0).sleep();
        }

        tf::Quaternion q = base_transform.getRotation();
        tf::Vector3 p = base_transform.getOrigin();
        Eigen::Quaterniond rot(q.w(), q.x(), q.y(), q.z());
        base_.linear() = rot.toRotationMatrix();
        base_.translation() = Eigen::Vector3d(p.x(), p.y(), p.z());

        q = laser_transform.getRotation();
        p = laser_transform.getOrigin();
        rot = Eigen::Quaterniond(q.w(), q.x(), q.y(), q.z());
        scanner.linear() = rot.toRotationMatrix();
        scanner.translation() = Eigen::Vector3d(p.x(), p.y(), p.z());




        publishData();
        ros::spinOnce();
        r.sleep();
    }

    ros::shutdown();

}
