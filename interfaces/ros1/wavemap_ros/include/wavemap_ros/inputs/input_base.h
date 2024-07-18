#ifndef WAVEMAP_ROS_INPUTS_INPUT_BASE_H_
#define WAVEMAP_ROS_INPUTS_INPUT_BASE_H_

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <wavemap/core/config/config_base.h>
#include <wavemap/core/config/string_list.h>
#include <wavemap/pipeline/pipeline.h>

#include "wavemap_ros/utils/tf_transformer.h"

namespace wavemap {
struct InputType : public TypeSelector<InputType> {
  using TypeSelector<InputType>::TypeSelector;

  enum Id : TypeId { kPointcloud, kDepthImage };

  static constexpr std::array names = {"pointcloud", "depth_image"};
};

struct InputBaseConfig : public ConfigBase<InputBaseConfig, 4, StringList> {
  std::string topic_name = "scan";
  int topic_queue_length = 10;

  StringList measurement_integrator_names;

  Seconds<FloatingPoint> processing_retry_period = 0.05f;

  static MemberMap memberMap;

  // Constructors
  InputBaseConfig() = default;
  InputBaseConfig(std::string topic_name, int topic_queue_length,
                  StringList measurement_integrator_names,
                  FloatingPoint processing_retry_period)
      : topic_name(std::move(topic_name)),
        topic_queue_length(topic_queue_length),
        measurement_integrator_names(std::move(measurement_integrator_names)),
        processing_retry_period(processing_retry_period) {}

  bool isValid(bool verbose) const override;
};

class InputBase {
 public:
  InputBase(const InputBaseConfig& config, std::shared_ptr<Pipeline> pipeline,
            std::shared_ptr<TfTransformer> transformer, std::string world_frame,
            const ros::NodeHandle& nh, ros::NodeHandle nh_private);
  virtual ~InputBase() = default;

  virtual InputType getType() const = 0;
  const std::string& getTopicName() { return config_.topic_name; }

 protected:
  const InputBaseConfig config_;

  std::shared_ptr<Pipeline> pipeline_;

  const std::shared_ptr<TfTransformer> transformer_;
  const std::string world_frame_;

  virtual void processQueue() = 0;
  ros::Timer queue_processing_retry_timer_;
};
}  // namespace wavemap

#endif  // WAVEMAP_ROS_INPUTS_INPUT_BASE_H_
