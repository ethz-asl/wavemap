#include "wavemap_ros/input_handler/input_handler_factory.h"

#include "wavemap_ros/input_handler/depth_image_input_handler.h"
#include "wavemap_ros/input_handler/pointcloud_input_handler.h"

namespace wavemap {
std::unique_ptr<InputHandler> InputHandlerFactory::create(
    const param::Value& params, std::string world_frame,
    VolumetricDataStructureBase::Ptr occupancy_map,
    std::shared_ptr<TfTransformer> transformer,
    std::shared_ptr<ThreadPool> thread_pool, ros::NodeHandle nh,
    ros::NodeHandle nh_private,
    std::optional<InputHandlerType> default_input_handler_type) {
  if (const auto type = InputHandlerType::from(params); type) {
    return create(type.value(), params, world_frame, occupancy_map,
                  std::move(transformer), std::move(thread_pool), nh,
                  nh_private);
  }

  if (default_input_handler_type.has_value()) {
    ROS_WARN_STREAM("Default type \""
                    << default_input_handler_type.value().toStr()
                    << "\" will be created instead.");
    return create(default_input_handler_type.value(), params,
                  std::move(world_frame), std::move(occupancy_map),
                  std::move(transformer), std::move(thread_pool), nh,
                  nh_private);
  }

  ROS_ERROR("No default was set. Returning nullptr.");
  return nullptr;
}

std::unique_ptr<InputHandler> InputHandlerFactory::create(
    InputHandlerType input_handler_type, const param::Value& params,
    std::string world_frame, VolumetricDataStructureBase::Ptr occupancy_map,
    std::shared_ptr<TfTransformer> transformer,
    std::shared_ptr<ThreadPool> thread_pool, ros::NodeHandle nh,
    ros::NodeHandle nh_private) {
  // Create the input handler
  switch (input_handler_type) {
    case InputHandlerType::kPointcloud: {
      if (const auto config =
              PointcloudInputHandlerConfig::from(params, "general");
          config) {
        return std::make_unique<PointcloudInputHandler>(
            config.value(), params, std::move(world_frame),
            std::move(occupancy_map), std::move(transformer),
            std::move(thread_pool), nh, nh_private);
      } else {
        ROS_ERROR("Pointcloud input handler config could not be loaded.");
        return nullptr;
      }
    }
    case InputHandlerType::kDepthImage: {
      if (const auto config =
              DepthImageInputHandlerConfig::from(params, "general");
          config) {
        return std::make_unique<DepthImageInputHandler>(
            config.value(), params, std::move(world_frame),
            std::move(occupancy_map), std::move(transformer),
            std::move(thread_pool), nh, nh_private);
      } else {
        ROS_ERROR("Depth image input handler config could not be loaded.");
        return nullptr;
      }
    }
  }
  return nullptr;
}
}  // namespace wavemap
