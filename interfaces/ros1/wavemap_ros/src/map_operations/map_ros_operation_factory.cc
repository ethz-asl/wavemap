#include "wavemap_ros/map_operations/map_ros_operation_factory.h"

#include <memory>
#include <string>
#include <utility>

#include "wavemap_ros/map_operations/crop_map_operation.h"
#include "wavemap_ros/map_operations/publish_map_operation.h"
#include "wavemap_ros/map_operations/publish_pointcloud_operation.h"

namespace wavemap {
std::unique_ptr<MapOperationBase> MapRosOperationFactory::create(
    const param::Value& params, MapBase::Ptr occupancy_map,
    std::shared_ptr<ThreadPool> thread_pool,
    std::shared_ptr<TfTransformer> transformer, std::string world_frame,
    ros::NodeHandle nh_private) {
  if (const auto type = MapRosOperationType::from(params); type) {
    return create(type.value(), params, std::move(occupancy_map),
                  std::move(thread_pool), std::move(transformer),
                  std::move(world_frame), nh_private);
  }

  LOG(ERROR) << "Could not create map operation. Returning nullptr.";
  return nullptr;
}

std::unique_ptr<MapOperationBase> MapRosOperationFactory::create(
    MapRosOperationType ros_operation_type, const param::Value& params,
    MapBase::Ptr occupancy_map, std::shared_ptr<ThreadPool> thread_pool,
    std::shared_ptr<TfTransformer> transformer, std::string world_frame,
    ros::NodeHandle nh_private) {
  if (!ros_operation_type.isValid()) {
    LOG(ERROR) << "Received request to create map operation with invalid type.";
    return nullptr;
  }

  // Create the operation handler
  switch (ros_operation_type) {
    case MapRosOperationType::kPublishMap:
      if (const auto config = PublishMapOperationConfig::from(params); config) {
        return std::make_unique<PublishMapOperation>(
            config.value(), std::move(occupancy_map), std::move(thread_pool),
            std::move(world_frame), nh_private);
      } else {
        ROS_ERROR("Publish map operation config could not be loaded.");
        return nullptr;
      }
    case MapRosOperationType::kPublishPointcloud:
      if (const auto config = PublishPointcloudOperationConfig::from(params);
          config) {
        return std::make_unique<PublishPointcloudOperation>(
            config.value(), std::move(occupancy_map), std::move(world_frame),
            nh_private);
      } else {
        ROS_ERROR("Publish pointcloud operation config could not be loaded.");
        return nullptr;
      }
    case MapRosOperationType::kCropMap:
      if (const auto config = CropMapOperationConfig::from(params); config) {
        return std::make_unique<CropMapOperation>(
            config.value(), std::move(occupancy_map), std::move(transformer),
            std::move(world_frame));
      } else {
        ROS_ERROR("Crop map operation config could not be loaded.");
        return nullptr;
      }
  }

  LOG(ERROR) << "Factory does not (yet) support creation of map operation type "
             << ros_operation_type.toStr() << ".";
  return nullptr;
}
}  // namespace wavemap
