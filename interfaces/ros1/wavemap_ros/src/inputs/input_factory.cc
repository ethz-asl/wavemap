#include "wavemap_ros/inputs/input_factory.h"

#include "wavemap_ros/inputs/depth_image_input.h"
#include "wavemap_ros/inputs/pointcloud_input.h"

namespace wavemap {
std::unique_ptr<InputBase> InputFactory::create(
    const param::Value& params, std::string world_frame,
    MapBase::Ptr occupancy_map, std::shared_ptr<TfTransformer> transformer,
    std::shared_ptr<ThreadPool> thread_pool, ros::NodeHandle nh,
    ros::NodeHandle nh_private, std::optional<InputType> default_input_type,
    std::function<void(const ros::Time&)> map_update_callback) {
  if (const auto type = InputType::from(params); type) {
    return create(type.value(), params, world_frame, occupancy_map,
                  std::move(transformer), std::move(thread_pool), nh,
                  nh_private, std::move(map_update_callback));
  }

  if (default_input_type.has_value()) {
    ROS_WARN_STREAM("Default type \"" << default_input_type.value().toStr()
                                      << "\" will be created instead.");
    return create(default_input_type.value(), params, std::move(world_frame),
                  std::move(occupancy_map), std::move(transformer),
                  std::move(thread_pool), nh, nh_private,
                  std::move(map_update_callback));
  }

  ROS_ERROR("No default was set. Returning nullptr.");
  return nullptr;
}

std::unique_ptr<InputBase> InputFactory::create(
    InputType input_type, const param::Value& params, std::string world_frame,
    MapBase::Ptr occupancy_map, std::shared_ptr<TfTransformer> transformer,
    std::shared_ptr<ThreadPool> thread_pool, ros::NodeHandle nh,
    ros::NodeHandle nh_private,
    std::function<void(const ros::Time&)> map_update_callback) {
  if (!input_type.isValid()) {
    ROS_ERROR("Received request to create input handler with invalid type.");
    return nullptr;
  }

  // Create the input handler
  switch (input_type) {
    case InputType::kPointcloud:
      if (const auto config = PointcloudInputConfig::from(params, "general");
          config) {
        return std::make_unique<PointcloudInput>(
            config.value(), params, std::move(world_frame),
            std::move(occupancy_map), std::move(transformer),
            std::move(thread_pool), nh, nh_private,
            std::move(map_update_callback));
      } else {
        ROS_ERROR("Pointcloud input handler config could not be loaded.");
        return nullptr;
      }
    case InputType::kDepthImage:
      if (const auto config = DepthImageInputConfig::from(params, "general");
          config) {
        return std::make_unique<DepthImageInput>(
            config.value(), params, std::move(world_frame),
            std::move(occupancy_map), std::move(transformer),
            std::move(thread_pool), nh, nh_private,
            std::move(map_update_callback));
      } else {
        ROS_ERROR("Depth image input handler config could not be loaded.");
        return nullptr;
      }
  }

  ROS_ERROR_STREAM("Factory does not support creation of input type "
                   << input_type.toStr() << ".");
  return nullptr;
}
}  // namespace wavemap
