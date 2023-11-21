#ifndef WAVEMAP_ROS_OPERATIONS_CROP_MAP_OPERATION_H_
#define WAVEMAP_ROS_OPERATIONS_CROP_MAP_OPERATION_H_

#include <memory>
#include <string>
#include <utility>

#include <wavemap/config/config_base.h>
#include <wavemap/data_structure/volumetric/volumetric_data_structure_base.h>

#include "wavemap_ros/operations/operation_base.h"
#include "wavemap_ros/utils/tf_transformer.h"

namespace wavemap {
/**
 * Config struct for map pruning operations.
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
                   VolumetricDataStructureBase::Ptr occupancy_map)
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
  const VolumetricDataStructureBase::Ptr occupancy_map_;
  ros::Time last_run_timestamp_;

  template <typename MapT, typename IndicatorFunctionT>
  void eraseBlockIf(MapT* map, IndicatorFunctionT indicator_fn) {
    auto& block_map = map->getBlocks();
    for (auto block_it = block_map.begin(); block_it != block_map.end();) {
      const auto& block_index = block_it->first;
      if (std::invoke(indicator_fn, block_index)) {
        block_it = block_map.erase(block_it);
      } else {
        ++block_it;
      }
    }
  }
};
}  // namespace wavemap

#endif  // WAVEMAP_ROS_OPERATIONS_CROP_MAP_OPERATION_H_
