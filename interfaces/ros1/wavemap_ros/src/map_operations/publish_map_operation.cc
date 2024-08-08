#include "wavemap_ros/map_operations/publish_map_operation.h"

#include <std_srvs/Empty.h>
#include <wavemap/core/utils/profiler_interface.h>
#include <wavemap_msgs/Map.h>
#include <wavemap_ros_conversions/map_msg_conversions.h>

namespace wavemap {
DECLARE_CONFIG_MEMBERS(PublishMapOperationConfig,
                      (once_every)
                      (max_num_blocks_per_msg)
                      (topic));

bool PublishMapOperationConfig::isValid(bool verbose) const {
  bool all_valid = true;

  all_valid &= IS_PARAM_GT(once_every, 0.f, verbose);
  all_valid &= IS_PARAM_GT(max_num_blocks_per_msg, 0, verbose);
  all_valid &= IS_PARAM_NE(topic, "", verbose);

  return all_valid;
}

PublishMapOperation::PublishMapOperation(
    const PublishMapOperationConfig& config, MapBase::Ptr occupancy_map,
    std::shared_ptr<ThreadPool> thread_pool, std::string world_frame,
    ros::NodeHandle nh_private)
    : MapOperationBase(std::move(occupancy_map)),
      config_(config.checkValid()),
      thread_pool_(std::move(thread_pool)),
      world_frame_(std::move(world_frame)) {
  map_pub_ = nh_private.advertise<wavemap_msgs::Map>(config_.topic, 10);
  republish_whole_map_srv_ =
      nh_private.advertiseService<std_srvs::Empty::Request,
                                  std_srvs::Empty::Response>(
          config_.topic + "_request_full",
          [this](auto& /*request*/, auto& /*response*/) {
            publishMap(ros::Time::now(), true);
            return true;
          });
}

bool PublishMapOperation::shouldRun(const ros::Time& current_time) {
  return config_.once_every < (current_time - last_run_timestamp_).toSec();
}

void PublishMapOperation::run(bool force_run) {
  const ros::Time current_time = ros::Time::now();
  if (force_run || shouldRun(current_time)) {
    publishMap(current_time, false);
    last_run_timestamp_ = current_time;
  }
}

void PublishMapOperation::publishMap(const ros::Time& current_time,
                                     bool republish_whole_map) {
  ProfilerZoneScoped;
  // If the map is empty, there's no work to do
  if (occupancy_map_->empty()) {
    return;
  }

  if (auto* hashed_wavelet_octree =
          dynamic_cast<HashedWaveletOctree*>(occupancy_map_.get());
      hashed_wavelet_octree) {
    publishHashedMap(current_time, hashed_wavelet_octree, republish_whole_map);
  } else if (auto* hashed_chunked_wavelet_octree =
                 dynamic_cast<HashedChunkedWaveletOctree*>(
                     occupancy_map_.get());
             hashed_chunked_wavelet_octree) {
    publishHashedMap(current_time, hashed_chunked_wavelet_octree,
                     republish_whole_map);
  } else {
    occupancy_map_->threshold();
    wavemap_msgs::Map map_msg;
    if (convert::mapToRosMsg(*occupancy_map_, world_frame_, current_time,
                             map_msg)) {
      map_pub_.publish(map_msg);
    }
  }
}
}  // namespace wavemap
