#ifndef WAVEMAP_ROS_WAVEMAP_SERVER_H_
#define WAVEMAP_ROS_WAVEMAP_SERVER_H_

#include <memory>
#include <string>
#include <vector>

#include <glog/logging.h>
#include <ros/ros.h>
#include <wavemap/common.h>
#include <wavemap/config/config_base.h>
#include <wavemap/data_structure/volumetric/volumetric_data_structure_base.h>

#include "wavemap/integrator/integrator_base.h"
#include "wavemap_ros/input_handler/input_handler.h"
#include "wavemap_ros/tf_transformer.h"
#include "wavemap_ros/utils/timer.h"

namespace wavemap {
struct WavemapServerConfig : ConfigBase<WavemapServerConfig, 4> {
  std::string world_frame = "odom";
  float thresholding_period = 1.f;
  float pruning_period = 10.f;
  float visualization_period = 10.f;

  static MemberMap memberMap;

  bool isValid(bool verbose) const override;
};

class WavemapServer {
 public:
  WavemapServer(ros::NodeHandle nh, ros::NodeHandle nh_private);
  WavemapServer(ros::NodeHandle nh, ros::NodeHandle nh_private,
                const WavemapServerConfig& config);

  void visualizeMap();
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
  ros::Timer map_visualization_timer_;
  ros::Timer map_autosave_timer_;

  void subscribeToTopics(ros::NodeHandle& nh);

  void advertiseTopics(ros::NodeHandle& nh_private);
  ros::Publisher map_pub_;
  ros::Publisher performance_stats_pub_;

  void advertiseServices(ros::NodeHandle& nh_private);
  ros::ServiceServer visualize_map_srv_;
  ros::ServiceServer save_map_srv_;
  ros::ServiceServer load_map_srv_;
};
}  // namespace wavemap

#endif  // WAVEMAP_ROS_WAVEMAP_SERVER_H_
