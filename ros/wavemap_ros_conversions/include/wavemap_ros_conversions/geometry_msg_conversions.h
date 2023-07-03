#ifndef WAVEMAP_ROS_CONVERSIONS_GEOMETRY_MSG_CONVERSIONS_H_
#define WAVEMAP_ROS_CONVERSIONS_GEOMETRY_MSG_CONVERSIONS_H_

#include <eigen_conversions/eigen_msg.h>

namespace wavemap::convert {
inline void rosMsgToPoint3D(const geometry_msgs::Vector3& msg, Point3D& point) {
  Eigen::Vector3d point_double;
  tf::vectorMsgToEigen(msg, point_double);
  point = point_double.cast<FloatingPoint>();
}

inline void rosMsgToRotation3D(const geometry_msgs::Quaternion& msg,
                               Rotation3D& rotation) {
  Eigen::Quaterniond rotation_double;
  tf::quaternionMsgToEigen(msg, rotation_double);
  rotation = Rotation3D{rotation.cast<FloatingPoint>()};
}

inline void rosMsgToTransformation3D(const geometry_msgs::Transform& msg,
                                     Transformation3D& transform) {
  rosMsgToPoint3D(msg.translation, transform.getPosition());
  rosMsgToRotation3D(msg.rotation, transform.getRotation());
}
}  // namespace wavemap::convert

#endif  // WAVEMAP_ROS_CONVERSIONS_GEOMETRY_MSG_CONVERSIONS_H_
