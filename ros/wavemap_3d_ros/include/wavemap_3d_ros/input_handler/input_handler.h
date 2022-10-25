#ifndef WAVEMAP_3D_ROS_INPUT_HANDLER_INPUT_HANDLER_H_
#define WAVEMAP_3D_ROS_INPUT_HANDLER_INPUT_HANDLER_H_

#include <memory>
#include <string>

#include <wavemap_3d/integrator/pointcloud_integrator_3d_factory.h>
#include <wavemap_common/utils/config_utils.h>
#include <wavemap_common_ros/tf_transformer.h>
#include <wavemap_common_ros/utils/config_conversions.h>
#include <wavemap_common_ros/utils/timer.h>

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

    float processing_retry_period = 0.1f;
    float max_wait_for_pose = 1.f;

    // TODO(victorr): Add option to reproject and publish the input pointclouds
    //                or depth images for debugging

    static Config from(const param::Map& params);
    bool isValid(bool verbose) const override;
  };

  InputHandler(const Config& config, const param::Map& params,
               std::string world_frame,
               VolumetricDataStructure3D::Ptr occupancy_map,
               std::shared_ptr<TfTransformer> transformer,
               const ros::NodeHandle& nh);
  virtual ~InputHandler() = default;

  virtual InputHandlerType getType() const = 0;
  const Config& getConfig() const { return config_; }

 protected:
  const Config config_;
  const std::string world_frame_;

  PointcloudIntegrator3D::Ptr integrator_;
  CpuTimer integration_timer_;

  std::shared_ptr<TfTransformer> transformer_;

  virtual void processQueue() = 0;
  ros::Timer queue_processing_retry_timer_;
};
}  // namespace wavemap

#endif  // WAVEMAP_3D_ROS_INPUT_HANDLER_INPUT_HANDLER_H_
