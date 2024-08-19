#ifndef WAVEMAP_ROS_MAP_OPERATIONS_PUBLISH_MAP_OPERATION_H_
#define WAVEMAP_ROS_MAP_OPERATIONS_PUBLISH_MAP_OPERATION_H_

#include <memory>
#include <string>

#include <ros/ros.h>
#include <wavemap/core/config/config_base.h>
#include <wavemap/core/map/map_base.h>
#include <wavemap/core/utils/thread_pool.h>
#include <wavemap/core/utils/time/time.h>
#include <wavemap/pipeline/map_operations/map_operation_base.h>

namespace wavemap {
/**
 * Config struct for map publishing operations.
 */
struct PublishMapOperationConfig
    : public ConfigBase<PublishMapOperationConfig, 3> {
  //! Time period controlling how often the map is published.
  Seconds<FloatingPoint> once_every = 2.f;

  //! Maximum number of blocks to transmit per wavemap map message.
  //! Used to control the maximum message size. Only works in combination with
  //! hash-based map data structures.
  int max_num_blocks_per_msg = 1000;

  //! Name of the topic the map will be published on.
  //! Note that the name of the service to request full map retransmissions will
  //! be the name of this topic suffixed with "_request_full".
  std::string topic = "map";

  static MemberMap memberMap;

  bool isValid(bool verbose) const override;
};

class PublishMapOperation : public MapOperationBase {
 public:
  PublishMapOperation(const PublishMapOperationConfig& config,
                      MapBase::Ptr occupancy_map,
                      std::shared_ptr<ThreadPool> thread_pool,
                      std::string world_frame, ros::NodeHandle nh_private);

  bool shouldRun(const ros::Time& current_time);

  void run(bool force_run) override;

 private:
  const PublishMapOperationConfig config_;
  const std::shared_ptr<ThreadPool> thread_pool_;
  const std::string world_frame_;
  ros::Time last_run_timestamp_;

  // Map publishing
  ros::Publisher map_pub_;
  Timestamp last_run_timestamp_internal_;
  void publishMap(const ros::Time& current_time, bool republish_whole_map);

  // NOTE: For hashed map types, such as HashedWaveletOctree and
  //       HashedChunkedWaveletOctree, we support incremental map transmissions
  //       which only include the blocks that changed since the last
  //       transmission, unless republish_whole_map is set to true.
  //       In case the number of blocks that changed exceeds
  //       config_.max_num_blocks_per_msg, the map update is transferred using
  //       multiple messages. This can be useful when transmitting the maps over
  //       unreliable networks, where smaller packets can perform better in
  //       terms of packet loss, or when the change is so large that
  //       transmitting it as a single message would exceed the maximum ROS
  //       message size (1GB).
  template <typename HashedMapT>
  void publishHashedMap(const ros::Time& current_time, HashedMapT* hashed_map,
                        bool republish_whole_map = false);

  ros::ServiceServer republish_whole_map_srv_;
};
}  // namespace wavemap

#include "wavemap_ros/map_operations/impl/publish_map_operation_inl.h"

#endif  // WAVEMAP_ROS_MAP_OPERATIONS_PUBLISH_MAP_OPERATION_H_
