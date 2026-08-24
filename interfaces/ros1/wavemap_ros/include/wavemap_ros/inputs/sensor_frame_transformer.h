#ifndef WAVEMAP_ROS_INPUTS_SENSOR_FRAME_TRANSFORMER_H_
#define WAVEMAP_ROS_INPUTS_SENSOR_FRAME_TRANSFORMER_H_

#include <memory>
#include <optional>
#include <string>
#include <utility>

#include <ros/ros.h>
#include <std_msgs/Header.h>

#include "wavemap_ros/utils/tf_transformer.h"

namespace wavemap {

// Reuses the RosServer's TF buffer to convert custom sensor observations into
// the map's world frame. This helper owns neither a subscriber nor a TF
// listener and can therefore be stored directly by a user sensor-input class.
class SensorFrameTransformer {
 public:
  SensorFrameTransformer(std::shared_ptr<TfTransformer> transformer,
                         std::string world_frame)
      : transformer_(std::move(transformer)),
        world_frame_(std::move(world_frame)) {}

  const std::string& worldFrame() const { return world_frame_; }

  std::optional<Transformation3D> lookup(
      const std_msgs::Header& header) const {
    if (header.frame_id.empty()) {
      ROS_WARN_THROTTLE(2.0,
                        "Cannot transform layered observations with an empty "
                        "source frame.");
      return std::nullopt;
    }
    if (TfTransformer::sanitizeFrameId(header.frame_id) ==
        TfTransformer::sanitizeFrameId(world_frame_)) {
      return Transformation3D();
    }
    const auto transform = transformer_->lookupTransform(
        world_frame_, header.frame_id, header.stamp);
    if (!transform) {
      ROS_WARN_STREAM_THROTTLE(
          2.0, "No transform from '" << header.frame_id << "' to '"
                                      << world_frame_ << "' at "
                                      << header.stamp << "; skipping layered "
                                         "sensor observations.");
    }
    return transform;
  }

  std::optional<Point3D> transformPosition(
      const Point3D& sensor_position,
      const std_msgs::Header& header) const {
    const auto transform = lookup(header);
    if (!transform) {
      return std::nullopt;
    }
    return *transform * sensor_position;
  }

  template <typename ObservationBatchT>
  bool transformBatch(ObservationBatchT& batch,
                      const std_msgs::Header& header) const {
    const auto transform = lookup(header);
    if (!transform) {
      return false;
    }
    batch.transformPositions(*transform);
    return true;
  }

 private:
  std::shared_ptr<TfTransformer> transformer_;
  std::string world_frame_;
};

}  // namespace wavemap

#endif  // WAVEMAP_ROS_INPUTS_SENSOR_FRAME_TRANSFORMER_H_
