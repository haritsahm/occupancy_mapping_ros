#ifndef PROPERTIES_H
#define PROPERTIES_H

#include <eigen3/Eigen/Eigen>
#include <math.h>
#include <vector>
#include <iostream>
#include <ostream>

struct Point2D
{
    double x,y;

    Point2D(){x = y = 0;}
    Point2D(double x_, double y_){x=x_;y=y_;}

    inline Point2D operator+(const Point2D& v) const {return Point2D(x+v.x, y+v.y);}
    inline Point2D operator-() const {return Point2D(-x, -y);}
    inline Point2D operator-(const Point2D& v) const {return Point2D(x-v.x, y-v.y);}
    inline Point2D operator%(const Point2D& v) const {return Point2D(x*v.x, y*v.y);}
    inline Point2D operator%=(const Point2D& v) const {return Point2D(x*v.x, y*v.y);}
    inline bool operator==(const Point2D& v) const {return (x==v.x) && (y==v.y);}
    inline bool operator!=(const Point2D& v) const {return (x!=v.x) || (y!=v.y);}
    inline Point2D operator/(const Point2D& v) const {return Point2D(x/v.x, y/v.y);}

    inline Point2D& operator=(const Point2D& v){	x=v.x; y=v.y;	return *this;}
    inline Point2D& operator*=(const double scalar){	x*=scalar;y*=scalar; return *this;}
    inline Point2D& operator/=(const double scalar){	x/=scalar;y/=scalar; return *this;}
    inline Point2D& operator+=(const Point2D& v){	x+=v.x; y+=v.y; return *this;}
    inline Point2D& operator-=(const Point2D& v){	x-=v.x; y-=v.y; return *this;}
    inline Point2D& round(){x=std::round(x); y = std::round(y); return *this;}

    friend std::ostream& operator<<(std::ostream& os, const Point2D& dt){
        os << "(" <<dt.x << ',' << dt.y << ')';return os;}
};

Eigen::Matrix3d getRotationX(double angle);
Eigen::Matrix3d getRotationY(double angle);
Eigen::Matrix3d getRotationZ(double angle);

Eigen::Vector3d convertRotationToRPY(const Eigen::Matrix3d& rotation);
Eigen::Matrix3d convertRPYToRotation(double roll, double pitch, double yaw);
Eigen::Quaterniond convertRPYToQuaternion(double roll, double pitch, double yaw);
Eigen::Quaterniond convertRotationToQuaternion(const Eigen::Matrix3d& rotation);
Eigen::Vector3d convertQuaternionToRPY(const Eigen::Quaterniond& quaternion);
Eigen::Matrix3d convertQuaternionToRotation(const Eigen::Quaterniond& quaternion);

#endif // PROPERTIES_H
