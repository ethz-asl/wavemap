#include "wavemap_ros/input_handler/input_handler_factory.h"

#include "wavemap_ros/input_handler/depth_image_input_handler.h"
#include "wavemap_ros/input_handler/pointcloud_input_handler.h"

namespace wavemap {
std::unique_ptr<InputHandler> InputHandlerFactory::create(
    const param::Map& params, std::string world_frame,
    VolumetricDataStructureBase::Ptr occupancy_map,
    std::shared_ptr<TfTransformer> transformer, ros::NodeHandle nh,
    ros::NodeHandle nh_private,
    std::optional<InputHandlerType> default_input_handler_type) {
  std::string error_msg;

  auto type = InputHandlerType::fromParamMap(params, error_msg);
  if (type.isValid()) {
    return create(type, params, world_frame, occupancy_map,
                  std::move(transformer), nh, nh_private);
  }

  if (default_input_handler_type.has_value()) {
    LOG(WARNING) << error_msg << " Default type \""
                 << default_input_handler_type.value().toStr()
                 << "\" will be created instead.";
    return create(default_input_handler_type.value(), params,
                  std::move(world_frame), std::move(occupancy_map),
                  std::move(transformer), nh, nh_private);
  }

  LOG(ERROR) << error_msg << "No default was set. Returning nullptr.";
  return nullptr;
}

std::unique_ptr<InputHandler> InputHandlerFactory::create(
    InputHandlerType input_handler_type, const param::Map& params,
    std::string world_frame, VolumetricDataStructureBase::Ptr occupancy_map,
    std::shared_ptr<TfTransformer> transformer, ros::NodeHandle nh,
    ros::NodeHandle nh_private) {
  // Load the input handler config
  if (!param::map::hasKey(params, "general")) {
    LOG(ERROR) << "Integrator params must have section named \"general\".";
    return nullptr;
  }
  if (!param::map::keyHoldsValue<param::Map>(params, "general")) {
    LOG(ERROR) << "Integrator param key \"general\" must contain a map "
                  "(dictionary) of parameters.";
    return nullptr;
  }

  // Create the input handler
  switch (input_handler_type.toTypeId()) {
    case InputHandlerType::kPointcloud: {
      const auto input_handler_config = PointcloudInputHandlerConfig::from(
          param::map::keyGetValue<param::Map>(params, "general"));
      return std::make_unique<PointcloudInputHandler>(
          input_handler_config, params, std::move(world_frame),
          std::move(occupancy_map), std::move(transformer), nh, nh_private);
    }
    case InputHandlerType::kDepthImage: {
      const auto input_handler_config = DepthImageInputHandlerConfig::from(
          param::map::keyGetValue<param::Map>(params, "general"));
      return std::make_unique<DepthImageInputHandler>(
          input_handler_config, params, std::move(world_frame),
          std::move(occupancy_map), std::move(transformer), nh, nh_private);
    }
  }
  return nullptr;
}
}  // namespace wavemap
