#include "wavemap_ros/inputs/input_factory.h"

#include "wavemap_ros/inputs/depth_image_input.h"
#include "wavemap_ros/inputs/pointcloud_input.h"

namespace wavemap {
std::unique_ptr<InputBase> InputFactory::create(
    const param::Value& params, std::shared_ptr<Pipeline> pipeline,
    std::shared_ptr<TfTransformer> transformer, std::string world_frame,
    ros::NodeHandle nh, ros::NodeHandle nh_private) {
  if (const auto type = InputType::from(params); type) {
    return create(type.value(), params, std::move(pipeline),
                  std::move(transformer), std::move(world_frame), nh,
                  nh_private);
  }

  LOG(ERROR) << "Could not create input handler. Returning nullptr.";
  return nullptr;
}

std::unique_ptr<InputBase> InputFactory::create(
    InputType input_type, const param::Value& params,
    std::shared_ptr<Pipeline> pipeline,
    std::shared_ptr<TfTransformer> transformer, std::string world_frame,
    ros::NodeHandle nh, ros::NodeHandle nh_private) {
  if (!input_type.isValid()) {
    ROS_ERROR("Received request to create input handler with invalid type.");
    return nullptr;
  }

  // Get the integrator names
  std::vector<std::string> integrator_names;
  if (const auto names_param = params.getChild("measurement_integrator");
      names_param) {
    if (const auto name_str = names_param->as<std::string>(); name_str) {
      integrator_names.emplace_back(name_str.value());
    } else if (const auto name_arr = names_param->as<param::Array>();
               name_arr) {
      size_t name_idx = 0u;
      for (const auto& name_el : name_arr.value()) {
        if (const auto name_el_str = name_el.as<std::string>(); name_el_str) {
          integrator_names.emplace_back(name_el_str.value());
        } else {
          ROS_WARN_STREAM(
              "Skipping \"measurement_integrator\" element at index "
              << name_idx << ". Could not be parsed as string.");
        }
        ++name_idx;
      }
    } else {
      ROS_ERROR(
          "Param \"measurement_integrator\" should be a string "
          "or list of strings.");
    }
  } else {
    ROS_ERROR(
        "No integrator name(s) specified. Provide them by setting param  "
        "\"measurement_integrator\" to a string or list of strings.");
  }
  if (integrator_names.empty()) {
    ROS_ERROR("Could not create input. Returning nullptr.");
    return nullptr;
  }

  // Create the input handler
  switch (input_type) {
    case InputType::kPointcloud:
      if (const auto config = PointcloudInputConfig::from(params); config) {
        return std::make_unique<PointcloudInput>(
            config.value(), std::move(pipeline), std::move(integrator_names),
            std::move(transformer), std::move(world_frame), nh, nh_private);
      } else {
        ROS_ERROR("Pointcloud input handler config could not be loaded.");
        return nullptr;
      }
    case InputType::kDepthImage:
      if (const auto config = DepthImageInputConfig::from(params); config) {
        return std::make_unique<DepthImageInput>(
            config.value(), std::move(pipeline), std::move(integrator_names),
            std::move(transformer), std::move(world_frame), nh, nh_private);
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
