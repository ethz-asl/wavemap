#ifndef WAVEMAP_ROS_INPUT_HANDLER_INPUT_HANDLER_H_
#define WAVEMAP_ROS_INPUT_HANDLER_INPUT_HANDLER_H_

#include <memory>
#include <string>

#include <wavemap/config/config_base.h>
#include <wavemap/integrator/integrator_factory.h>

#include "wavemap_ros/tf_transformer.h"
#include "wavemap_ros/utils/config_conversions.h"
#include "wavemap_ros/utils/timer.h"

namespace wavemap {
struct InputHandlerType : public TypeSelector<InputHandlerType> {
  using TypeSelector<InputHandlerType>::TypeSelector;

  enum Id : TypeId { kPointcloud, kDepthImage };

  static constexpr std::array names = {"pointcloud", "depth_image"};
};

class InputHandler {
 public:
  struct Config : public ConfigBase<Config> {
    std::string topic_name = "scan";
    int topic_queue_length = 10;

    float processing_retry_period = 0.05f;
    float max_wait_for_pose = 1.f;

    std::string sensor_frame_id;  // Leave blank to use frame_id from msg header
    std::string image_transport_hints = "raw";
    float depth_scale_factor = 1.f;

    std::string reprojected_topic_name;  // Leave blank to disable

    static Config from(const param::Map& params);
    bool isValid(bool verbose) const override;
  };

  InputHandler(const Config& config, const param::Map& params,
               std::string world_frame,
               VolumetricDataStructureBase::Ptr occupancy_map,
               std::shared_ptr<TfTransformer> transformer,
               const ros::NodeHandle& nh, ros::NodeHandle nh_private);
  virtual ~InputHandler() = default;

  virtual InputHandlerType getType() const = 0;
  const Config& getConfig() const { return config_; }

  bool isReprojectionEnabled() const {
    return !config_.reprojected_topic_name.empty();
  }

 protected:
  const Config config_;
  const std::string world_frame_;

  IntegratorBase::Ptr integrator_;
  CpuTimer integration_timer_;

  std::shared_ptr<TfTransformer> transformer_;

  virtual void processQueue() = 0;
  ros::Timer queue_processing_retry_timer_;

  void publishReprojected(const ros::Time& stamp,
                          const PosedPointcloud<Point3D>& posed_pointcloud);
  ProjectorBase::ConstPtr projection_model_;
  ros::Publisher reprojection_pub_;
};
}  // namespace wavemap

#endif  // WAVEMAP_ROS_INPUT_HANDLER_INPUT_HANDLER_H_
