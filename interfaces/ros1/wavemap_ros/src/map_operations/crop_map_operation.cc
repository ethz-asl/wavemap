#include "wavemap_ros/map_operations/crop_map_operation.h"

#include <memory>
#include <string>
#include <utility>

#include <wavemap/core/map/hashed_chunked_wavelet_octree.h>
#include <wavemap/core/map/hashed_wavelet_octree.h>
#include <wavemap/core/utils/edit/crop.h>

namespace wavemap {
DECLARE_CONFIG_MEMBERS(CropMapOperationConfig,
                      (once_every)
                      (body_frame)
                      (tf_time_offset)
                      (radius)
                      (max_update_resolution));

bool CropMapOperationConfig::isValid(bool verbose) const {
  bool all_valid = true;

  all_valid &= IS_PARAM_GT(once_every, 0.f, verbose);
  all_valid &= IS_PARAM_NE(body_frame, "", verbose);
  all_valid &=
      IS_PARAM_TRUE(tf_time_offset == -1.f || 0.f <= tf_time_offset, verbose);
  all_valid &= IS_PARAM_GT(radius, 0.f, verbose);
  all_valid &= IS_PARAM_GE(max_update_resolution, 0.f, verbose);

  return all_valid;
}

CropMapOperation::CropMapOperation(const CropMapOperationConfig& config,
                                   MapBase::Ptr occupancy_map,
                                   std::shared_ptr<ThreadPool> thread_pool,
                                   std::shared_ptr<TfTransformer> transformer,
                                   std::string world_frame)
    : MapOperationBase(std::move(occupancy_map)),
      config_(config.checkValid()),
      thread_pool_(std::move(thread_pool)),
      transformer_(std::move(transformer)),
      world_frame_(std::move(world_frame)) {}

bool CropMapOperation::shouldRun(const ros::Time& current_time) {
  return config_.once_every < (current_time - last_run_timestamp_).toSec();
}

void CropMapOperation::run(bool force_run) {
  const ros::Time current_time = ros::Time::now();
  if (!force_run && !shouldRun(current_time)) {
    return;
  }
  last_run_timestamp_ = current_time;

  // If the map is empty, there's no work to do
  if (occupancy_map_->empty()) {
    return;
  }

  // Get the center point of the cropping sphere (usually the robot's position)
  const bool use_most_recent_transform = config_.tf_time_offset < 0.f;
  const ros::Time timestamp =
      use_most_recent_transform
          ? ros::Time::UNINITIALIZED
          : current_time - ros::Duration(config_.tf_time_offset);
  const auto T_W_B = transformer_->lookupTransform(
      world_frame_, config_.body_frame, timestamp);
  if (!T_W_B) {
    if (use_most_recent_transform) {
      ROS_WARN_STREAM(
          "Could not look up center point for map cropping. No TF from "
          "body_frame \""
          << config_.body_frame << "\" to world_frame \"" << world_frame_
          << "\".");
    } else {
      ROS_WARN_STREAM(
          "Could not look up center point for map cropping. TF lookup from "
          "body_frame \""
          << config_.body_frame << "\" to world_frame \"" << world_frame_
          << "\" at time " << timestamp << " failed.");
    }
    return;
  }

  // Crop the map
  timer_.start();
  if (auto* hashed_wavelet_octree =
          dynamic_cast<HashedWaveletOctree*>(occupancy_map_.get());
      hashed_wavelet_octree) {
    edit::crop_to_sphere(*hashed_wavelet_octree, T_W_B->getPosition(),
                         config_.radius, termination_height_, thread_pool_);
  } else if (auto* hashed_chunked_wavelet_octree =
                 dynamic_cast<HashedChunkedWaveletOctree*>(
                     occupancy_map_.get());
             hashed_chunked_wavelet_octree) {
    edit::crop_to_sphere(*hashed_chunked_wavelet_octree, T_W_B->getPosition(),
                         config_.radius, termination_height_, thread_pool_);
  } else {
    ROS_WARN(
        "Map cropping is only supported for hash-based map data structures.");
  }
  timer_.stop();
  ROS_DEBUG_STREAM("Cropped map in " << timer_.getLastEpisodeDuration()
                                     << "s. Total cropping time: "
                                     << timer_.getTotalDuration() << "s.");
}
}  // namespace wavemap
