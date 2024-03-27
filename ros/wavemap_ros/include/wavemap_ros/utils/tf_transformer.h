#ifndef WAVEMAP_ROS_UTILS_TF_TRANSFORMER_H_
#define WAVEMAP_ROS_UTILS_TF_TRANSFORMER_H_

#include <map>
#include <string>

#include <tf2_ros/transform_listener.h>
#include <wavemap/common.h>

namespace wavemap {
class TfTransformer {
 public:
  explicit TfTransformer(FloatingPoint tf_buffer_cache_time = 10.f)
      : tf_buffer_(ros::Duration(tf_buffer_cache_time)),
        tf_listener_(tf_buffer_) {}

  // Check whether a transform is available
  bool isTransformAvailable(const std::string& to_frame_id,
                            const std::string& from_frame_id,
                            const ros::Time& frame_timestamp) const;

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
  bool lookupLatestTransform(const std::string& to_frame_id,
                             const std::string& from_frame_id,
                             Transformation3D& transform);

  // Strip leading slashes if needed to avoid TF errors
  static std::string sanitizeFrameId(const std::string& string);

 private:
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  // Transform lookup timers
  // Timeout between each update attempt
  const ros::WallDuration transform_lookup_retry_period_{0.02};
  // Maximum time to wait before giving up
  const ros::WallDuration transform_lookup_max_time_{0.25};

  bool waitForTransformImpl(const std::string& to_frame_id,
                            const std::string& from_frame_id,
                            const ros::Time& frame_timestamp) const;
  bool lookupTransformImpl(const std::string& to_frame_id,
                           const std::string& from_frame_id,
                           const ros::Time& frame_timestamp,
                           Transformation3D& transform);
};
}  // namespace wavemap

#endif  // WAVEMAP_ROS_UTILS_TF_TRANSFORMER_H_
