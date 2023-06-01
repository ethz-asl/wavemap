#include "wavemap_ros/input_handler/input_handler_factory.h"

#include "wavemap_ros/input_handler/depth_image_input_handler.h"
#include "wavemap_ros/input_handler/livox_input_handler.h"
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
  const auto input_handler_config = InputHandlerConfig::from(
      param::map::keyGetValue<param::Map>(params, "general"));

  // Create the input handler
  switch (input_handler_type.toTypeId()) {
    case InputHandlerType::kPointcloud:
      return std::make_unique<PointcloudInputHandler>(
          input_handler_config, params, std::move(world_frame),
          std::move(occupancy_map), std::move(transformer), nh, nh_private);
    case InputHandlerType::kDepthImage:
      return std::make_unique<DepthImageInputHandler>(
          input_handler_config, params, std::move(world_frame),
          std::move(occupancy_map), std::move(transformer), nh, nh_private);
    case InputHandlerType::kLivox:
#ifdef LIVOX_AVAILABLE
      return std::make_unique<LivoxInputHandler>(
          input_handler_config, params, std::move(world_frame),
          std::move(occupancy_map), std::move(transformer), nh, nh_private);
#else
      LOG(ERROR) << "Livox support is currently not available. Please install "
                    "livox_ros_driver2 and rebuild wavemap.";
      return nullptr;
#endif
  }
  return nullptr;
}
}  // namespace wavemap
