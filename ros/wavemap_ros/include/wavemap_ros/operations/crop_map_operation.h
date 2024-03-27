#ifndef WAVEMAP_ROS_OPERATIONS_CROP_MAP_OPERATION_H_
#define WAVEMAP_ROS_OPERATIONS_CROP_MAP_OPERATION_H_

#include <memory>
#include <string>
#include <utility>

#include <wavemap/config/config_base.h>
#include <wavemap/map/map_base.h>

#include "wavemap_ros/operations/operation_base.h"
#include "wavemap_ros/utils/tf_transformer.h"

namespace wavemap {
/**
 * Config struct for map cropping operations.
 */
struct CropMapOperationConfig : public ConfigBase<CropMapOperationConfig, 3> {
  //! Time period controlling how often the map is cropped.
  Seconds<FloatingPoint> once_every = 10.f;

  //! Name of the TF frame to treat as the center point. Usually the robot's
  //! body frame. When the cropper runs, all blocks that are further than
  //! remove_blocks_beyond_distance from this point are deleted.
  std::string body_frame = "body";

  //! Distance beyond which blocks are deleted when the cropper is executed.
  Meters<FloatingPoint> remove_blocks_beyond_distance;

  static MemberMap memberMap;

  bool isValid(bool verbose) const override;
};

class CropMapOperation : public OperationBase {
 public:
  CropMapOperation(const CropMapOperationConfig& config,
                   std::string world_frame,
                   std::shared_ptr<TfTransformer> transformer,
                   MapBase::Ptr occupancy_map)
      : config_(config.checkValid()),
        world_frame_(std::move(world_frame)),
        transformer_(std::move(transformer)),
        occupancy_map_(std::move(occupancy_map)) {}

  OperationType getType() const override { return OperationType::kCropMap; }

  bool shouldRun(const ros::Time& current_time) {
    return config_.once_every < (current_time - last_run_timestamp_).toSec();
  }

  void run(const ros::Time& current_time, bool force_run) override;

 private:
  const CropMapOperationConfig config_;
  const std::string world_frame_;
  const std::shared_ptr<TfTransformer> transformer_;
  const MapBase::Ptr occupancy_map_;
  ros::Time last_run_timestamp_;
};
}  // namespace wavemap

#endif  // WAVEMAP_ROS_OPERATIONS_CROP_MAP_OPERATION_H_
