#ifndef WAVEMAP_ROS_INPUT_HANDLER_INPUT_HANDLER_H_
#define WAVEMAP_ROS_INPUT_HANDLER_INPUT_HANDLER_H_

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <image_transport/image_transport.h>
#include <wavemap/config/config_base.h>
#include <wavemap/data_structure/image.h>
#include <wavemap/data_structure/pointcloud.h>
#include <wavemap/integrator/integrator_base.h>
#include <wavemap/map/map_base.h>
#include <wavemap/utils/thread_pool.h>
#include <wavemap/utils/time/stopwatch.h>

#include "wavemap_ros/tf_transformer.h"

namespace wavemap {
struct InputHandlerType : public TypeSelector<InputHandlerType> {
  using TypeSelector<InputHandlerType>::TypeSelector;

  enum Id : TypeId { kPointcloud, kDepthImage };

  static constexpr std::array names = {"pointcloud", "depth_image"};
};

struct InputHandlerConfig : public ConfigBase<InputHandlerConfig, 5> {
  std::string topic_name = "scan";
  int topic_queue_length = 10;

  Seconds<FloatingPoint> processing_retry_period = 0.05f;

  std::string reprojected_pointcloud_topic_name;  // Leave blank to disable
  std::string projected_range_image_topic_name;   // Leave blank to disable

  static MemberMap memberMap;

  // Constructors
  InputHandlerConfig() = default;
  InputHandlerConfig(std::string topic_name, int topic_queue_length,
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

class InputHandler {
 public:
  InputHandler(const InputHandlerConfig& config, const param::Value& params,
               std::string world_frame, MapBase::Ptr occupancy_map,
               std::shared_ptr<TfTransformer> transformer,
               std::shared_ptr<ThreadPool> thread_pool,
               const ros::NodeHandle& nh, ros::NodeHandle nh_private);
  virtual ~InputHandler() = default;

  virtual InputHandlerType getType() const = 0;

  const std::string& getTopicName() { return config_.topic_name; }

  bool shouldPublishReprojectedPointcloud() const {
    return !config_.reprojected_pointcloud_topic_name.empty() &&
           0 < reprojected_pointcloud_pub_.getNumSubscribers();
  }
  bool shouldPublishProjectedRangeImage() const {
    return !config_.projected_range_image_topic_name.empty() &&
           0 < projected_range_image_pub_.getNumSubscribers();
  }

 protected:
  const InputHandlerConfig config_;
  const std::string world_frame_;

  std::vector<IntegratorBase::Ptr> integrators_;
  Stopwatch integration_timer_;

  std::shared_ptr<TfTransformer> transformer_;

  virtual void processQueue() = 0;
  ros::Timer queue_processing_retry_timer_;

  void publishReprojectedPointcloud(const ros::Time& stamp,
                                    const PosedPointcloud<>& posed_pointcloud);
  ros::Publisher reprojected_pointcloud_pub_;

  void publishProjectedRangeImage(const ros::Time& stamp,
                                  const Image<>& range_image);
  image_transport::Publisher projected_range_image_pub_;
};
}  // namespace wavemap

#endif  // WAVEMAP_ROS_INPUT_HANDLER_INPUT_HANDLER_H_
