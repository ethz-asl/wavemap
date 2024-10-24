#include "wavemap_ros/inputs/ros_input_factory.h"

#include <memory>
#include <string>
#include <utility>

#include "wavemap_ros/inputs/depth_image_topic_input.h"
#include "wavemap_ros/inputs/pointcloud_topic_input.h"

namespace wavemap {
std::unique_ptr<RosInputBase> RosInputFactory::create(
    const param::Value& params, std::shared_ptr<Pipeline> pipeline,
    std::shared_ptr<TfTransformer> transformer, std::string world_frame,
    ros::NodeHandle nh, ros::NodeHandle nh_private) {
  if (const auto type = RosInputType::from(params); type) {
    return create(type.value(), params, std::move(pipeline),
                  std::move(transformer), std::move(world_frame), nh,
                  nh_private);
  }

  LOG(ERROR) << "Could not create input handler. Returning nullptr.";
  return nullptr;
}

std::unique_ptr<RosInputBase> RosInputFactory::create(
    RosInputType input_type, const param::Value& params,
    std::shared_ptr<Pipeline> pipeline,
    std::shared_ptr<TfTransformer> transformer, std::string world_frame,
    ros::NodeHandle nh, ros::NodeHandle nh_private) {
  if (!input_type.isValid()) {
    ROS_ERROR("Received request to create input handler with invalid type.");
    return nullptr;
  }

  // Create the input handler
  switch (input_type) {
    case RosInputType::kPointcloudTopic:
      if (const auto config = PointcloudTopicInputConfig::from(params);
          config) {
        return std::make_unique<PointcloudTopicInput>(
            config.value(), std::move(pipeline), std::move(transformer),
            std::move(world_frame), nh, nh_private);
      } else {
        ROS_ERROR("Pointcloud input handler config could not be loaded.");
        return nullptr;
      }
    case RosInputType::kDepthImageTopic:
      if (const auto config = DepthImageTopicInputConfig::from(params);
          config) {
        return std::make_unique<DepthImageTopicInput>(
            config.value(), std::move(pipeline), std::move(transformer),
            std::move(world_frame), nh, nh_private);
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
