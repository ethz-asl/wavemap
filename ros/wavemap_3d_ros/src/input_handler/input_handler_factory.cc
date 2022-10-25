#include "wavemap_3d_ros/input_handler/input_handler_factory.h"

#include "wavemap_3d_ros/input_handler/depth_image_input_handler.h"
#include "wavemap_3d_ros/input_handler/pointcloud_input_handler.h"

namespace wavemap {
std::unique_ptr<InputHandler> InputHandlerFactory::create(
    const param::Map& integrator_params, std::string world_frame,
    VolumetricDataStructure3D::Ptr occupancy_map,
    std::shared_ptr<TfTransformer> transformer, const ros::NodeHandle& nh,
    std::optional<InputHandlerType> default_input_handler_type) {
  std::string error_msg;

  if (param::map::keyHoldsValue<param::Map>(integrator_params, "input")) {
    const auto& input_params =
        param::map::keyGetValue<param::Map>(integrator_params, "input");
    auto type = InputHandlerType::fromParamMap(input_params, error_msg);
    if (type.isValid()) {
      return create(type, integrator_params, world_frame, occupancy_map,
                    transformer, nh);
    }
  }

  if (default_input_handler_type.has_value()) {
    LOG(WARNING) << error_msg << " Default type \""
                 << default_input_handler_type.value().toStr()
                 << "\" will be created instead.";
    return create(default_input_handler_type.value(), integrator_params,
                  world_frame, occupancy_map, transformer, nh);
  }

  LOG(ERROR) << error_msg << "No default was set. Returning nullptr.";
  return nullptr;
}

std::unique_ptr<InputHandler> InputHandlerFactory::create(
    InputHandlerType input_handler_type, const param::Map& integrator_params,
    std::string world_frame, VolumetricDataStructure3D::Ptr occupancy_map,
    std::shared_ptr<TfTransformer> transformer, const ros::NodeHandle& nh) {
  // Load the input handler config
  if (!param::map::keyHoldsValue<param::Map>(integrator_params, "input")) {
    return nullptr;
  }
  const auto input_handler_config = InputHandler::Config::from(
      param::map::keyGetValue<param::Map>(integrator_params, "input"));

  // Create the input handler
  switch (input_handler_type.toTypeId()) {
    case InputHandlerType::kPointcloud:
      return std::make_unique<PointcloudInputHandler>(
          input_handler_config, integrator_params, world_frame, occupancy_map,
          transformer, nh);
    case InputHandlerType::kDepthImage:
      return std::make_unique<DepthImageInputHandler>(
          input_handler_config, integrator_params, world_frame, occupancy_map,
          transformer, nh);
  }
  return nullptr;
}
}  // namespace wavemap
