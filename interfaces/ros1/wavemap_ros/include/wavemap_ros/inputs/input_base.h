#ifndef WAVEMAP_ROS_INPUTS_INPUT_BASE_H_
#define WAVEMAP_ROS_INPUTS_INPUT_BASE_H_

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <image_transport/image_transport.h>
#include <wavemap/core/config/config_base.h>
#include <wavemap/core/data_structure/image.h>
#include <wavemap/core/data_structure/pointcloud.h>
#include <wavemap/core/integrator/integrator_base.h>
#include <wavemap/core/map/map_base.h>
#include <wavemap/core/utils/thread_pool.h>
#include <wavemap/core/utils/time/stopwatch.h>

#include "wavemap_ros/utils/tf_transformer.h"

namespace wavemap {
struct InputType : public TypeSelector<InputType> {
  using TypeSelector<InputType>::TypeSelector;

  enum Id : TypeId { kPointcloud, kDepthImage };

  static constexpr std::array names = {"pointcloud", "depth_image"};
};

struct InputBaseConfig : public ConfigBase<InputBaseConfig, 5> {
  std::string topic_name = "scan";
  int topic_queue_length = 10;

  Seconds<FloatingPoint> processing_retry_period = 0.05f;

  std::string reprojected_pointcloud_topic_name;  // Leave blank to disable
  std::string projected_range_image_topic_name;   // Leave blank to disable

  static MemberMap memberMap;

  // Constructors
  InputBaseConfig() = default;
  InputBaseConfig(std::string topic_name, int topic_queue_length,
                  FloatingPoint processing_retry_period,
                  std::string reprojected_pointcloud_topic_name,
                  std::string projected_range_image_topic_name)
      : topic_name(std::move(topic_name)),
        topic_queue_length(topic_queue_length),
        processing_retry_period(processing_retry_period),
        reprojected_pointcloud_topic_name(
            std::move(reprojected_pointcloud_topic_name)),
        projected_range_image_topic_name(
            std::move(projected_range_image_topic_name)) {}

  bool isValid(bool verbose) const override;
};

class InputBase {
 public:
  InputBase(const InputBaseConfig& config, const param::Value& params,
            std::string world_frame, MapBase::Ptr occupancy_map,
            std::shared_ptr<TfTransformer> transformer,
            std::shared_ptr<ThreadPool> thread_pool, const ros::NodeHandle& nh,
            ros::NodeHandle nh_private,
            std::function<void(const ros::Time&)> map_update_callback = {});
  virtual ~InputBase() = default;

  virtual InputType getType() const = 0;
  const std::string& getTopicName() { return config_.topic_name; }

 protected:
  const InputBaseConfig config_;
  const std::string world_frame_;

  const std::shared_ptr<TfTransformer> transformer_;

  std::vector<IntegratorBase::Ptr> integrators_;
  Stopwatch integration_timer_;

  virtual void processQueue() = 0;
  ros::Timer queue_processing_retry_timer_;

  std::function<void(const ros::Time&)> map_update_callback_;

  bool shouldPublishReprojectedPointcloud() const;
  void publishReprojectedPointcloud(const ros::Time& stamp,
                                    const PosedPointcloud<>& posed_pointcloud);
  ros::Publisher reprojected_pointcloud_pub_;

  bool shouldPublishProjectedRangeImage() const;
  void publishProjectedRangeImage(const ros::Time& stamp,
                                  const Image<>& range_image);
  image_transport::Publisher projected_range_image_pub_;
};
}  // namespace wavemap

#endif  // WAVEMAP_ROS_INPUTS_INPUT_BASE_H_
