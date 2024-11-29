#include "wavemap_ros/map_operations/decay_map_operation.h"

#include <memory>
#include <utility>

#include <wavemap/core/map/hashed_chunked_wavelet_octree.h>
#include <wavemap/core/map/hashed_wavelet_octree.h>

#include "wavemap/core/utils/edit/multiply.h"

namespace wavemap {
DECLARE_CONFIG_MEMBERS(DecayMapOperationConfig,
                      (once_every)
                      (decay_rate));

bool DecayMapOperationConfig::isValid(bool verbose) const {
  bool all_valid = true;

  all_valid &= IS_PARAM_GT(once_every, 0.f, verbose);
  all_valid &= IS_PARAM_GT(decay_rate, 0.f, verbose);
  all_valid &= IS_PARAM_LT(decay_rate, 1.f, verbose);

  return all_valid;
}

DecayMapOperation::DecayMapOperation(const DecayMapOperationConfig& config,
                                     MapBase::Ptr occupancy_map,
                                     std::shared_ptr<ThreadPool> thread_pool)
    : MapOperationBase(std::move(occupancy_map)),
      config_(config.checkValid()),
      thread_pool_(std::move(thread_pool)) {}

bool DecayMapOperation::shouldRun(const ros::Time& current_time) {
  return config_.once_every < (current_time - last_run_timestamp_).toSec();
}

void DecayMapOperation::run(bool force_run) {
  const ros::Time current_time = ros::Time::now();
  if (!force_run && !shouldRun(current_time)) {
    return;
  }
  last_run_timestamp_ = current_time;

  // If the map is empty, there's no work to do
  if (occupancy_map_->empty()) {
    return;
  }

  // Decay the map
  timer_.start();
  if (auto* hashed_wavelet_octree =
          dynamic_cast<HashedWaveletOctree*>(occupancy_map_.get());
      hashed_wavelet_octree) {
    multiply(*hashed_wavelet_octree, config_.decay_rate, thread_pool_);
  } else if (auto* hashed_chunked_wavelet_octree =
                 dynamic_cast<HashedChunkedWaveletOctree*>(
                     occupancy_map_.get());
             hashed_chunked_wavelet_octree) {
    multiply(*hashed_chunked_wavelet_octree, config_.decay_rate, thread_pool_);
  } else {
    ROS_WARN("Map decay is only supported for hash-based map data structures.");
  }
  timer_.stop();
  ROS_DEBUG_STREAM("Decayed map in " << timer_.getLastEpisodeDuration()
                                     << "s. Total decaying time: "
                                     << timer_.getTotalDuration() << "s.");
}
}  // namespace wavemap
