#ifndef WAVEMAP_ROS_WAVEMAP_SERVER_H_
#define WAVEMAP_ROS_WAVEMAP_SERVER_H_

#include <memory>
#include <string>
#include <unordered_set>
#include <vector>

#include <glog/logging.h>
#include <ros/ros.h>
#include <wavemap/common.h>
#include <wavemap/config/config_base.h>
#include <wavemap/data_structure/volumetric/volumetric_data_structure_base.h>
#include <wavemap/indexing/index_hashes.h>
#include <wavemap/integrator/integrator_base.h>
#include <wavemap/utils/stopwatch.h>
#include <wavemap/utils/time.h>

#include "wavemap_ros/input_handler/input_handler.h"
#include "wavemap_ros/tf_transformer.h"

namespace wavemap {
struct WavemapServerConfig : ConfigBase<WavemapServerConfig, 5> {
  std::string world_frame = "odom";
  FloatingPoint thresholding_period = 1.f;
  FloatingPoint pruning_period = 10.f;
  FloatingPoint publication_period = 10.f;

  int max_num_blocks_per_msg = 1000;

  static MemberMap memberMap;

  bool isValid(bool verbose) const override;
};

class WavemapServer {
 public:
  WavemapServer(ros::NodeHandle nh, ros::NodeHandle nh_private);
  WavemapServer(ros::NodeHandle nh, ros::NodeHandle nh_private,
                const WavemapServerConfig& config);

  void publishMap(bool republish_whole_map = false);
  bool saveMap(const std::string& file_path) const;
  bool loadMap(const std::string& file_path);

  InputHandler* addInput(const param::Map& integrator_params,
                         const ros::NodeHandle& nh, ros::NodeHandle nh_private);

  VolumetricDataStructureBase::Ptr getMap() { return occupancy_map_; }
  VolumetricDataStructureBase::ConstPtr getMap() const {
    return occupancy_map_;
  }

 private:
  const WavemapServerConfig config_;

  VolumetricDataStructureBase::Ptr occupancy_map_;

  std::shared_ptr<TfTransformer> transformer_;
  std::vector<std::unique_ptr<InputHandler>> input_handlers_;

  void subscribeToTimers(const ros::NodeHandle& nh);
  ros::Timer map_pruning_timer_;
  ros::Timer map_thresholding_timer_;
  ros::Timer map_publication_timer_;

  void subscribeToTopics(ros::NodeHandle& nh);

  void advertiseTopics(ros::NodeHandle& nh_private);

  ros::Publisher map_pub_;

  Timestamp last_map_pub_time_;
  std::unordered_set<Index3D, Index3DHash> block_publishing_queue_;
  template <typename HashedMapT>
  void publishHashedMap(HashedMapT* hashed_map,
                        bool republish_whole_map = false);

  void advertiseServices(ros::NodeHandle& nh_private);
  ros::ServiceServer republish_whole_map_srv_;
  ros::ServiceServer save_map_srv_;
  ros::ServiceServer load_map_srv_;
};
}  // namespace wavemap

#include "wavemap_ros/impl/wavemap_server_inl.h"

#endif  // WAVEMAP_ROS_WAVEMAP_SERVER_H_
