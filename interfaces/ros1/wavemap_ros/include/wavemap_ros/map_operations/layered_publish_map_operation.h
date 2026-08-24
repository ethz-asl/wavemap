#ifndef WAVEMAP_ROS_MAP_OPERATIONS_LAYERED_PUBLISH_MAP_OPERATION_H_
#define WAVEMAP_ROS_MAP_OPERATIONS_LAYERED_PUBLISH_MAP_OPERATION_H_

#include <memory>
#include <string>
#include <unordered_set>
#include <utility>

#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <wavemap/core/indexing/index_hashes.h>
#include <wavemap/core/utils/profile/profiler_interface.h>
#include <wavemap/core/utils/thread_pool.h>
#include <wavemap/core/utils/time/time.h>
#include <wavemap/pipeline/map_operations/map_operation_base.h>
#include <wavemap_msgs/Map.h>
#include <wavemap_msgs/LayeredMap.h>
#include <wavemap_ros_conversions/map_msg_conversions.h>
#include <wavemap_ros_conversions/layered_map_msg_conversions.h>

#include "wavemap_ros/map_operations/publish_map_operation.h"

namespace wavemap {

// Typed equivalent of PublishMapOperation for HashedWaveletOctreeT<Voxel>.
// It uses the same topic, batching, timestamps, full-map request service, and
// deletion semantics as the original occupancy-only publisher.
template <typename CellDataT, typename CellDataRosConverterT>
class LayeredPublishMapOperation final : public MapOperationBase {
 public:
  using Map = HashedWaveletOctreeT<CellDataT>;

  LayeredPublishMapOperation(
      const PublishMapOperationConfig& config, typename Map::Ptr map,
      std::shared_ptr<ThreadPool> thread_pool, std::string world_frame,
      ros::NodeHandle nh_private)
      : MapOperationBase(map),
        config_(config.checkValid()),
        map_(std::move(map)),
        thread_pool_(std::move(thread_pool)),
        world_frame_(std::move(world_frame)) {
    map_pub_ = nh_private.advertise<wavemap_msgs::Map>(config_.topic, 10);
    republish_whole_map_srv_ =
        nh_private.advertiseService<std_srvs::Empty::Request,
                                    std_srvs::Empty::Response>(
            config_.topic + "_request_full", [this](auto&, auto&) {
              publishMap(ros::Time::now(), true);
              return true;
            });
  }

  void run(bool force_run) override {
    const ros::Time current_time = ros::Time::now();
    if (force_run ||
        config_.once_every < (current_time - last_run_timestamp_).toSec()) {
      publishMap(current_time, false);
      last_run_timestamp_ = current_time;
    }
  }

 private:
  void publishMap(const ros::Time& current_time, bool republish_whole_map) {
    ProfilerZoneScoped;
    if (map_->empty()) {
      return;
    }

    const Timestamp selection_start = Time::now();
    std::unordered_set<Index3D, Index3DHash> changed_blocks;
    map_->forEachBlock(
        [&changed_blocks, republish_whole_map,
         last_published = last_run_timestamp_internal_](
            const Index3D& block_index, const auto& block) {
          if (republish_whole_map ||
              last_published < block.getLastUpdatedStamp()) {
            changed_blocks.emplace(block_index);
          }
        });
    last_run_timestamp_internal_ = selection_start;

    while (!changed_blocks.empty()) {
      std::unordered_set<Index3D, Index3DHash> blocks_to_publish;
      for (const auto& block_index : changed_blocks) {
        if (auto* block = map_->getBlock(block_index); block) {
          block->threshold();
          blocks_to_publish.emplace(block_index);
          if (config_.max_num_blocks_per_msg <=
              static_cast<int>(blocks_to_publish.size())) {
            break;
          }
        }
      }

      wavemap_msgs::Map map_msg;
      convert::mapToRosMsg<CellDataT, CellDataRosConverterT>(
          *map_, world_frame_, current_time, map_msg, blocks_to_publish,
          thread_pool_);
      map_pub_.publish(map_msg);

      for (const auto& block_index : blocks_to_publish) {
        changed_blocks.erase(block_index);
      }
    }
  }

  const PublishMapOperationConfig config_;
  const typename Map::Ptr map_;
  const std::shared_ptr<ThreadPool> thread_pool_;
  const std::string world_frame_;
  ros::Time last_run_timestamp_;
  Timestamp last_run_timestamp_internal_;
  ros::Publisher map_pub_;
  ros::ServiceServer republish_whole_map_srv_;
};

// Unified publisher for a complete LayeredMap. Occupancy and other continuous
// fields use Wavemap's block timestamps, while discrete layers use dirty
// compressed-parent tracking. Continuous-only schemas pay only one empty tuple
// traversal per publication cycle.
template <typename LayeredMapT, typename CellDataRosConverterT>
class UnifiedLayeredPublishMapOperation final : public MapOperationBase {
 public:
  using ContinuousMap = typename LayeredMapT::ContinuousMap;

  UnifiedLayeredPublishMapOperation(
      const PublishMapOperationConfig& config,
      LayeredMapT* layered_map,
      std::shared_ptr<ThreadPool> thread_pool, std::string world_frame,
      ros::NodeHandle nh_private)
      : MapOperationBase(CHECK_NOTNULL(layered_map)->continuousMapPtr()),
        config_(config.checkValid()),
        layered_map_(std::move(layered_map)),
        thread_pool_(std::move(thread_pool)),
        world_frame_(std::move(world_frame)) {
    map_pub_ = nh_private.advertise<wavemap_msgs::LayeredMap>(config_.topic, 10);
  }

  void run(bool force_run) override {
    const ros::Time current_time = ros::Time::now();
    if (force_run ||
        config_.once_every < (current_time - last_run_timestamp_).toSec()) {
      publishIncrement(current_time);
      last_run_timestamp_ = current_time;
    }
  }

 private:
  void publishIncrement(const ros::Time& stamp) {
    const Timestamp selection_start = Time::now();
    std::unordered_set<Index3D, Index3DHash> changed_blocks;
    layered_map_->continuousMap().forEachBlock(
        [&changed_blocks, last_published = last_run_timestamp_internal_](
            const Index3D& block_index, const auto& block) {
          if (last_published < block.getLastUpdatedStamp()) {
            changed_blocks.emplace(block_index);
          }
        });
    const bool has_discrete_changes =
        convert::discreteLayerBundleHasDirtyState(
            layered_map_->discreteLayers());
    last_run_timestamp_internal_ = selection_start;
    if (changed_blocks.empty() && !has_discrete_changes) {
      return;
    }

    bool discrete_patch_pending = has_discrete_changes;
    do {
      std::unordered_set<Index3D, Index3DHash> blocks_to_publish;
      for (const auto& block_index : changed_blocks) {
        if (auto* block =
                layered_map_->continuousMap().getBlock(block_index);
            block) {
          block->threshold();
          blocks_to_publish.emplace(block_index);
          if (config_.max_num_blocks_per_msg <=
              static_cast<int>(blocks_to_publish.size())) {
            break;
          }
        }
      }

      wavemap_msgs::LayeredMap msg;
      convert::layeredMapPatchToRosMsg<LayeredMapT,
                                       CellDataRosConverterT>(
          *layered_map_, world_frame_, stamp, msg, blocks_to_publish,
          thread_pool_);
      if (!discrete_patch_pending) {
        msg.discrete_layers.clear();
      }
      map_pub_.publish(msg);
      discrete_patch_pending = false;
      for (const auto& block_index : blocks_to_publish) {
        changed_blocks.erase(block_index);
      }
    } while (!changed_blocks.empty());

    convert::clearDiscreteLayerBundleDirtyState(
        layered_map_->discreteLayers());
  }

  const PublishMapOperationConfig config_;
  LayeredMapT* const layered_map_;
  const std::shared_ptr<ThreadPool> thread_pool_;
  const std::string world_frame_;
  ros::Time last_run_timestamp_;
  Timestamp last_run_timestamp_internal_;
  ros::Publisher map_pub_;
};

}  // namespace wavemap

#endif  // WAVEMAP_ROS_MAP_OPERATIONS_LAYERED_PUBLISH_MAP_OPERATION_H_
