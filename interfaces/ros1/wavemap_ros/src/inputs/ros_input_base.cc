#include "wavemap_ros/inputs/ros_input_base.h"

namespace wavemap {
DECLARE_CONFIG_MEMBERS(RosInputBaseConfig,
                      (topic_name)
                      (topic_queue_length)
                      (measurement_integrator_names)
                      (processing_retry_period));

bool RosInputBaseConfig::isValid(bool verbose) const {
  bool all_valid = true;

  all_valid &= IS_PARAM_NE(topic_name, "", verbose);
  all_valid &= IS_PARAM_GT(topic_queue_length, 0, verbose);
  all_valid &= IS_PARAM_FALSE(measurement_integrator_names.empty(), verbose);
  all_valid &= IS_PARAM_GT(processing_retry_period, 0.f, verbose);

  return all_valid;
}

RosInputBase::RosInputBase(const RosInputBaseConfig& config,
                           std::shared_ptr<Pipeline> pipeline,
                           std::shared_ptr<TfTransformer> transformer,
                           std::string world_frame, const ros::NodeHandle& nh,
                           ros::NodeHandle /*nh_private*/)
    : config_(config.checkValid()),
      pipeline_(std::move(pipeline)),
      transformer_(std::move(transformer)),
      world_frame_(std::move(world_frame)) {
  // Start the queue processing retry timer
  queue_processing_retry_timer_ =
      nh.createTimer(ros::Duration(config_.processing_retry_period),
                     [this](const auto& /*event*/) { processQueue(); });
}
}  // namespace wavemap
