#include "wavemap_ros/operations/operation_factory.h"

#include "wavemap_ros/operations/crop_map_operation.h"
#include "wavemap_ros/operations/prune_map_operation.h"
#include "wavemap_ros/operations/publish_map_operation.h"
#include "wavemap_ros/operations/threshold_map_operation.h"

namespace wavemap {
std::unique_ptr<OperationBase> OperationFactory::create(
    const param::Value& params, std::string world_frame,
    VolumetricDataStructureBase::Ptr occupancy_map,
    std::shared_ptr<TfTransformer> transformer,
    std::shared_ptr<ThreadPool> thread_pool, ros::NodeHandle nh_private,
    std::optional<OperationType> default_operation_type) {
  if (const auto type = OperationType::from(params); type) {
    return create(type.value(), params, std::move(world_frame),
                  std::move(occupancy_map), std::move(transformer),
                  std::move(thread_pool), nh_private);
  }

  if (default_operation_type.has_value()) {
    ROS_WARN_STREAM("Default type \"" << default_operation_type.value().toStr()
                                      << "\" will be created instead.");
    return create(default_operation_type.value(), params,
                  std::move(world_frame), std::move(occupancy_map),
                  std::move(transformer), std::move(thread_pool), nh_private);
  }

  ROS_ERROR("No default was set. Returning nullptr.");
  return nullptr;
}

std::unique_ptr<OperationBase> OperationFactory::create(
    OperationType operation_type, const param::Value& params,
    std::string world_frame, VolumetricDataStructureBase::Ptr occupancy_map,
    std::shared_ptr<TfTransformer> transformer,
    std::shared_ptr<ThreadPool> thread_pool, ros::NodeHandle nh_private) {
  if (!operation_type.isValid()) {
    ROS_ERROR("Received request to create operation with invalid type.");
    return nullptr;
  }

  // Create the operation handler
  switch (operation_type.toTypeId()) {
    case OperationType::kThresholdMap:
      if (const auto config = ThresholdMapOperationConfig::from(params);
          config) {
        return std::make_unique<ThresholdMapOperation>(
            config.value(), std::move(occupancy_map));
      } else {
        ROS_ERROR("Threshold map operation config could not be loaded.");
        return nullptr;
      }
    case OperationType::kPruneMap:
      if (const auto config = PruneMapOperationConfig::from(params); config) {
        return std::make_unique<PruneMapOperation>(config.value(),
                                                   std::move(occupancy_map));
      } else {
        ROS_ERROR("Prune map operation config could not be loaded.");
        return nullptr;
      }
    case OperationType::kPublishMap:
      if (const auto config = PublishMapOperationConfig::from(params); config) {
        return std::make_unique<PublishMapOperation>(
            config.value(), std::move(world_frame), std::move(occupancy_map),
            std::move(thread_pool), nh_private);
      } else {
        ROS_ERROR("Publish map operation config could not be loaded.");
        return nullptr;
      }
    case OperationType::kCropMap:
      if (const auto config = CropMapOperationConfig::from(params); config) {
        return std::make_unique<CropMapOperation>(
            config.value(), std::move(world_frame), std::move(transformer),
            std::move(occupancy_map));
      } else {
        ROS_ERROR("Crop map operation config could not be loaded.");
        return nullptr;
      }
  }

  ROS_ERROR_STREAM("Factory does not support creation of operation type "
                   << operation_type.toStr() << ".");
  return nullptr;
}
}  // namespace wavemap
