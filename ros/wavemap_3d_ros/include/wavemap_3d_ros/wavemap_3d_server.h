#ifndef WAVEMAP_3D_ROS_WAVEMAP_3D_SERVER_H_
#define WAVEMAP_3D_ROS_WAVEMAP_3D_SERVER_H_

#include <memory>
#include <string>
#include <vector>

#include <glog/logging.h>
#include <ros/ros.h>
#include <wavemap_3d/data_structure/volumetric_data_structure_3d.h>
#include <wavemap_3d/integrator/pointcloud_integrator_3d.h>
#include <wavemap_common/common.h>
#include <wavemap_common/utils/config_utils.h>
#include <wavemap_common_ros/tf_transformer.h>
#include <wavemap_common_ros/utils/timer.h>

#include "wavemap_3d_ros/input_handler/input_handler.h"

namespace wavemap {
class Wavemap3DServer {
 public:
  struct Config : ConfigBase<Config> {
    struct General {
      std::string world_frame = "odom";
      bool publish_performance_stats = false;
    } general;

    struct Map {
      float pruning_period = 1.f;
      float visualization_period = 10.f;
      float autosave_period = -1.f;
      std::string autosave_path;
    } map;

    static Config from(const param::Map& params);
    bool isValid(bool verbose) const override;
  };

  Wavemap3DServer(ros::NodeHandle nh, ros::NodeHandle nh_private);
  Wavemap3DServer(ros::NodeHandle nh, ros::NodeHandle nh_private,
                  const Config& config);

  void visualizeMap();
  bool saveMap(const std::string& file_path) const;
  bool loadMap(const std::string& file_path);

  InputHandler* addInput(const param::Map& integrator_params,
                         const ros::NodeHandle& nh);

 private:
  static constexpr bool kSaveWithFloatingPointPrecision = true;

  const Config config_;

  VolumetricDataStructure3D::Ptr occupancy_map_;

  std::shared_ptr<TfTransformer> transformer_;
  std::vector<std::unique_ptr<InputHandler>> input_handlers_;

  void subscribeToTimers(const ros::NodeHandle& nh);
  ros::Timer map_pruning_timer_;
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

#endif  // WAVEMAP_3D_ROS_WAVEMAP_3D_SERVER_H_
