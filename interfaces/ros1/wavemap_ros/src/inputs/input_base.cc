#include "wavemap_ros/inputs/input_base.h"

namespace wavemap {
DECLARE_CONFIG_MEMBERS(InputBaseConfig,
                      (topic_name)
                      (topic_queue_length)
                      (processing_retry_period));

bool InputBaseConfig::isValid(bool verbose) const {
  bool all_valid = true;

  all_valid &= IS_PARAM_NE(topic_name, std::string(""), verbose);
  all_valid &= IS_PARAM_GT(topic_queue_length, 0, verbose);
  all_valid &= IS_PARAM_GT(processing_retry_period, 0.f, verbose);

  return all_valid;
}

InputBase::InputBase(const InputBaseConfig& config,
                     std::shared_ptr<Pipeline> pipeline,
                     std::vector<std::string> integrator_names,
                     std::shared_ptr<TfTransformer> transformer,
                     std::string world_frame, const ros::NodeHandle& nh,
                     ros::NodeHandle /*nh_private*/)
    : config_(config.checkValid()),
      pipeline_(std::move(pipeline)),
      integrator_names_(std::move(integrator_names)),
      transformer_(std::move(transformer)),
      world_frame_(std::move(world_frame)) {
  // Start the queue processing retry timer
  queue_processing_retry_timer_ =
      nh.createTimer(ros::Duration(config_.processing_retry_period),
                     [this](const auto& /*event*/) { processQueue(); });
}
}  // namespace wavemap
