#include <occupancy_mapping/properties.h>

Eigen::Matrix3d getRotationX(double angle)
{
  Eigen::Matrix3d rotation(3,3);

  rotation <<
      1.0, 0.0, 0.0,
      0.0, cos(angle), -sin(angle),
      0.0, sin(angle), cos(angle);

  return rotation;
}

Eigen::Matrix3d getRotationY(double angle)
{
  Eigen::Matrix3d rotation(3,3);

  rotation <<
      cos(angle), 0.0, sin(angle),
      0.0, 1.0, 0.0,
      -sin(angle), 0.0, cos(angle);

  return rotation;
}

Eigen::Matrix3d getRotationZ(double angle)
{
  Eigen::Matrix3d rotation(3,3);

  rotation <<
      cos(angle), -sin(angle), 0.0,
      sin(angle), cos(angle), 0.0,
      0.0, 0.0, 1.0;

  return rotation;
}

Eigen::Vector3d convertRotationToRPY(const Eigen::Matrix3d& rotation)
{
  Eigen::Vector3d rpy;// = Eigen::MatrixXd::Zero(3,1);

  rpy.coeffRef(0,0) = atan2(rotation.coeff(2,1), rotation.coeff(2,2));
  rpy.coeffRef(1,0) = atan2(-rotation.coeff(2,0), sqrt(pow(rotation.coeff(2,1), 2) + pow(rotation.coeff(2,2),2)));
  rpy.coeffRef(2,0) = atan2 (rotation.coeff(1,0), rotation.coeff(0,0));

  return rpy;
}

Eigen::Matrix3d convertRPYToRotation(double roll, double pitch, double yaw)
{
  Eigen::Matrix3d rotation = getRotationZ(yaw)*getRotationY(pitch)*getRotationX(roll);

  return rotation;
}

Eigen::Quaterniond convertRPYToQuaternion(double roll, double pitch, double yaw)
{
  Eigen::Quaterniond quaternion;
  quaternion = convertRPYToRotation(roll,pitch,yaw);

  return quaternion;
}

Eigen::Quaterniond convertRotationToQuaternion(const Eigen::Matrix3d& rotation)
{
  Eigen::Quaterniond quaternion;
  quaternion = rotation;

  return quaternion;
}

Eigen::Vector3d convertQuaternionToRPY(const Eigen::Quaterniond& quaternion)
{
  Eigen::Vector3d rpy = convertRotationToRPY(quaternion.toRotationMatrix());

  return rpy;
}

Eigen::Matrix3d convertQuaternionToRotation(const Eigen::Quaterniond& quaternion)
{
  Eigen::Matrix3d rotation = quaternion.toRotationMatrix();

  return rotation;
}

