#ifndef WAVEMAP_ROS_CONVERSIONS_GEOMETRY_MSG_CONVERSIONS_H_
#define WAVEMAP_ROS_CONVERSIONS_GEOMETRY_MSG_CONVERSIONS_H_

#include <eigen_conversions/eigen_msg.h>
#include <geometry_msgs/Point32.h>

namespace wavemap::convert {
inline Point3D pointMsgToPoint3D(const geometry_msgs::Point& msg) {
  Eigen::Vector3d point_double;
  tf::pointMsgToEigen(msg, point_double);
  return point_double.cast<FloatingPoint>();
}

inline geometry_msgs::Point point3DToPointMsg(const Point3D& point) {
  geometry_msgs::Point msg;
  tf::pointEigenToMsg(point.cast<double>(), msg);
  return msg;
}

inline Point3D point32MsgToPoint3D(const geometry_msgs::Point32& msg) {
  return {msg.x, msg.y, msg.z};
}

inline geometry_msgs::Point32 point3DToPoint32Msg(const Point3D& point) {
  geometry_msgs::Point32 msg;
  msg.x = point.x();
  msg.y = point.y();
  msg.z = point.z();
  return msg;
}

inline Vector3D vector3MsgToVector3D(const geometry_msgs::Vector3& msg) {
  Eigen::Vector3d vector_double;
  tf::vectorMsgToEigen(msg, vector_double);
  return vector_double.cast<FloatingPoint>();
}

inline geometry_msgs::Vector3 vector3DToVector3Msg(const Vector3D& vector) {
  geometry_msgs::Vector3 msg;
  tf::vectorEigenToMsg(vector.cast<double>(), msg);
  return msg;
}

inline Rotation3D quaternionMsgToRotation3D(
    const geometry_msgs::Quaternion& msg) {
  Eigen::Quaterniond rotation_double;
  tf::quaternionMsgToEigen(msg, rotation_double);
  return Rotation3D{rotation_double.cast<FloatingPoint>()};
}

inline Transformation3D transformMsgToTransformation3D(
    const geometry_msgs::Transform& msg) {
  return Transformation3D{quaternionMsgToRotation3D(msg.rotation),
                          vector3MsgToVector3D(msg.translation)};
}
}  // namespace wavemap::convert

#endif  // WAVEMAP_ROS_CONVERSIONS_GEOMETRY_MSG_CONVERSIONS_H_
