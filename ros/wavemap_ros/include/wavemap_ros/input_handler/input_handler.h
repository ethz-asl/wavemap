#ifndef WAVEMAP_ROS_INPUT_HANDLER_INPUT_HANDLER_H_
#define WAVEMAP_ROS_INPUT_HANDLER_INPUT_HANDLER_H_

#include <memory>
#include <string>
#include <vector>

#include <wavemap/config/config_base.h>
#include <wavemap/integrator/integrator_factory.h>
#include <wavemap/integrator/projection_model/projector_base.h>

#include "wavemap_ros/tf_transformer.h"
#include "wavemap_ros/utils/timer.h"

namespace wavemap {
struct InputHandlerType : public TypeSelector<InputHandlerType> {
  using TypeSelector<InputHandlerType>::TypeSelector;

  enum Id : TypeId { kPointcloud, kDepthImage, kLivox };

  static constexpr std::array names = {"pointcloud", "depth_image", "livox"};
};

struct InputHandlerConfig : public ConfigBase<InputHandlerConfig, 9> {
  std::string topic_name = "scan";
  int topic_queue_length = 10;

  FloatingPoint processing_retry_period = 0.05f;
  FloatingPoint max_wait_for_pose = 1.f;

  std::string sensor_frame_id;  // Leave blank to use frame_id from msg header
  std::string image_transport_hints = "raw";
  FloatingPoint depth_scale_factor = 1.f;
  FloatingPoint time_delay = 0.f;

  std::string reprojected_topic_name;  // Leave blank to disable

  static MemberMap memberMap;

  bool isValid(bool verbose) const override;
};

class InputHandler {
 public:
  InputHandler(const InputHandlerConfig& config, const param::Map& params,
               std::string world_frame,
               VolumetricDataStructureBase::Ptr occupancy_map,
               std::shared_ptr<TfTransformer> transformer,
               const ros::NodeHandle& nh, ros::NodeHandle nh_private);
  virtual ~InputHandler() = default;

  virtual InputHandlerType getType() const = 0;
  const InputHandlerConfig& getConfig() const { return config_; }

  bool isReprojectionEnabled() const {
    return !config_.reprojected_topic_name.empty() &&
           0 < reprojection_pub_.getNumSubscribers();
  }

 protected:
  const InputHandlerConfig config_;
  const std::string world_frame_;

  std::vector<IntegratorBase::Ptr> integrators_;
  Timer integration_timer_;

  std::shared_ptr<TfTransformer> transformer_;

  virtual void processQueue() = 0;
  ros::Timer queue_processing_retry_timer_;

  void publishReprojected(const ros::Time& stamp,
                          const PosedPointcloud<>& posed_pointcloud);
  ProjectorBase::ConstPtr projection_model_;
  ros::Publisher reprojection_pub_;
};
}  // namespace wavemap

#endif  // WAVEMAP_ROS_INPUT_HANDLER_INPUT_HANDLER_H_
