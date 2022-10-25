#include "wavemap_3d_ros/input_handler/input_handler.h"

namespace wavemap {
InputHandler::InputHandler(const param::Map& params, std::string world_frame,
                           VolumetricDataStructure3D::Ptr occupancy_map,
                           std::shared_ptr<TfTransformer> transformer,
                           const ros::NodeHandle& nh)
    : InputHandler(
          Config::from(param::map::keyGetValue<param::Map>(params, "input")),
          params, world_frame, occupancy_map, transformer, nh) {}

InputHandler::InputHandler(const InputHandler::Config& config,
                           const param::Map& params, std::string world_frame,
                           VolumetricDataStructure3D::Ptr occupancy_map,
                           std::shared_ptr<TfTransformer> transformer,
                           const ros::NodeHandle& nh)
    : config_(config.checkValid()),
      world_frame_(std::move(world_frame)),
      transformer_(std::move(transformer)) {
  // Create the integrator
  integrator_ = PointcloudIntegrator3DFactory::create(
      params, std::move(occupancy_map),
      PointcloudIntegrator3DType::kSingleRayIntegrator);
  CHECK_NOTNULL(integrator_);

  // Start the queue processing retry timer
  queue_processing_retry_timer_ =
      nh.createTimer(ros::Duration(config_.processing_retry_period),
                     [this](const auto& /*event*/) { processQueue(); });
}

InputHandler::Config InputHandler::Config::from(const param::Map& params) {
  Config config;

  config.topic_name = param::map::keyGetValue(params, NAMEOF(config.topic_name),
                                              config.topic_name);

  config.topic_queue_length = param::map::keyGetValue(
      params, NAMEOF(config.topic_queue_length), config.topic_queue_length);

  config.processing_retry_period =
      param::map::keyGetValue(params, NAMEOF(config.processing_retry_period),
                              config.processing_retry_period);

  config.max_wait_for_pose = param::convert::toSeconds(
      params, NAMEOF(config.max_wait_for_pose), config.max_wait_for_pose);

  return config;
}

bool InputHandler::Config::isValid(bool verbose) const {
  bool all_valid = true;

  all_valid &= IS_PARAM_NE(topic_name, std::string(""), verbose);
  all_valid &= IS_PARAM_GT(topic_queue_length, 0, verbose);
  all_valid &= IS_PARAM_GT(processing_retry_period, 0.f, verbose);
  all_valid &= IS_PARAM_GE(max_wait_for_pose, 0.f, verbose);

  return all_valid;
}
}  // namespace wavemap
