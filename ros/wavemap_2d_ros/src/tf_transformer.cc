#include "wavemap_2d_ros/tf_transformer.h"

#include <string>

#include <minkindr_conversions/kindr_msg.h>

namespace wavemap {
bool TfTransformer::isTransformAvailable(
    const std::string& to_frame_id, const std::string& from_frame_id,
    const ros::Time& frame_timestamp) const {
  return tf_buffer_.canTransform(to_frame_id, from_frame_id, frame_timestamp);
}

bool TfTransformer::waitForTransform(const std::string& to_frame_id,
                                     const std::string& from_frame_id,
                                     const ros::Time& frame_timestamp) {
  return waitForTransformImpl(sanitizeFrameId(to_frame_id),
                              sanitizeFrameId(from_frame_id), frame_timestamp);
}

bool TfTransformer::lookupTransform(const std::string& to_frame_id,
                                    const std::string& from_frame_id,
                                    const ros::Time& frame_timestamp,
                                    Transformation3D& transform) {
  return lookupTransformImpl(sanitizeFrameId(to_frame_id),
                             sanitizeFrameId(from_frame_id), frame_timestamp,
                             transform);
}

std::string TfTransformer::sanitizeFrameId(const std::string& string) {
  // Strip leading slashes if needed to avoid TF errors
  if (string[0] == '/') {
    return string.substr(1, string.length());
  } else {
    return string;
  }
}

bool TfTransformer::waitForTransformImpl(
    const std::string& to_frame_id, const std::string& from_frame_id,
    const ros::Time& frame_timestamp) const {
  // Total time spent waiting for the updated pose
  ros::WallDuration t_waited(0.0);
  while (t_waited < transform_lookup_max_time_) {
    if (tf_buffer_.canTransform(to_frame_id, from_frame_id, frame_timestamp)) {
      return true;
    }
    transform_lookup_retry_period_.sleep();
    t_waited += transform_lookup_retry_period_;
  }
  ROS_WARN(
      "Waited %.3fs, but still could not get the TF from %s to %s at timestamp "
      "%u seconds",
      t_waited.toSec(), from_frame_id.c_str(), to_frame_id.c_str(),
      frame_timestamp.sec);
  return false;
}

bool TfTransformer::lookupTransformImpl(const std::string& to_frame_id,
                                        const std::string& from_frame_id,
                                        const ros::Time& frame_timestamp,
                                        Transformation3D& transform) {
  if (!isTransformAvailable(to_frame_id, from_frame_id, frame_timestamp)) {
    return false;
  }
  geometry_msgs::TransformStamped transform_msg =
      tf_buffer_.lookupTransform(to_frame_id, from_frame_id, frame_timestamp);
  tf::transformMsgToKindr(transform_msg.transform, &transform);
  return true;
}
}  // namespace wavemap
