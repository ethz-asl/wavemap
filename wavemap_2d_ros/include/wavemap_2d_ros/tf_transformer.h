#ifndef WAVEMAP_2D_ROS_TF_TRANSFORMER_H_
#define WAVEMAP_2D_ROS_TF_TRANSFORMER_H_

#include <map>
#include <string>

#include <tf2_ros/transform_listener.h>
#include <wavemap_2d/common.h>

namespace wavemap_2d {
class TfTransformer {
 public:
  explicit TfTransformer(FloatingPoint tf_buffer_cache_time = 10.f)
      : tf_buffer_(ros::Duration(tf_buffer_cache_time)),
        tf_listener_(tf_buffer_),
        transform_lookup_retry_period_(0.02),
        transform_lookup_max_time_(0.25) {}

  // Check whether a transform is available
  bool isTransformAvailable(const std::string& to_frame_id,
                            const std::string& from_frame_id,
                            const ros::Time& frame_timestamp);

  // Waits for a transform to become available, while doing less aggressive
  // polling that ROS's standard tf2_ros::Buffer::canTransform(...)
  bool waitForTransform(const std::string& to_frame_id,
                        const std::string& from_frame_id,
                        const ros::Time& frame_timestamp);

  // Lookup transforms and convert them to Kindr
  bool lookupTransform(const std::string& to_frame_id,
                       const std::string& from_frame_id,
                       const ros::Time& frame_timestamp,
                       Transformation3D& transform);

 protected:
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  // Transform lookup timers
  // Timeout between each update attempt
  const ros::WallDuration transform_lookup_retry_period_;
  // Maximum time to wait before giving up
  const ros::WallDuration transform_lookup_max_time_;

  static std::string sanitizeFrameId(const std::string& string);
  bool waitForTransformImpl(const std::string& to_frame_id,
                            const std::string& from_frame_id,
                            const ros::Time& frame_timestamp);
  bool lookupTransformImpl(const std::string& to_frame_id,
                           const std::string& from_frame_id,
                           const ros::Time& frame_timestamp,
                           Transformation3D& transform);
};
}  // namespace wavemap_2d

#endif  // WAVEMAP_2D_ROS_TF_TRANSFORMER_H_
