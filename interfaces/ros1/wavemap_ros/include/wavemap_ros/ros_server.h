#ifndef WAVEMAP_ROS_ROS_SERVER_H_
#define WAVEMAP_ROS_ROS_SERVER_H_

#include <algorithm>
#include <filesystem>
#include <memory>
#include <string>
#include <vector>

#include <glog/logging.h>
#include <ros/ros.h>
#include <wavemap/core/common.h>
#include <wavemap/core/config/config_base.h>
#include <wavemap/core/indexing/index_hashes.h>
#include <wavemap/core/integrator/integrator_base.h>
#include <wavemap/core/map/map_base.h>
#include <wavemap/core/utils/thread_pool.h>
#include <wavemap/pipeline/pipeline.h>

#include "wavemap_ros/inputs/ros_input_base.h"
#include "wavemap_ros/utils/logging_level.h"
#include "wavemap_ros/utils/tf_transformer.h"

namespace wavemap {
/**
 * Config struct for wavemap's ROS server.
 */
struct RosServerConfig : ConfigBase<RosServerConfig, 4, LoggingLevel> {
  //! Name of the coordinate frame in which to store the map.
  //! Will be used as the frame_id for ROS TF lookups.
  std::string world_frame = "odom";
  //! Minimum severity level for ROS logging messages to be logged.
  LoggingLevel logging_level = LoggingLevel::kInfo;
  //! Maximum number of threads to use.
  //! Defaults to the number of threads supported by the CPU.
  int num_threads =
      std::max(1, static_cast<int>(std::thread::hardware_concurrency()));
  //! Whether or not to allow resetting the map through the reset_map service.
  bool allow_reset_map_service = false;

  static MemberMap memberMap;

  bool isValid(bool verbose) const override;
};

class RosServer {
 public:
  RosServer(ros::NodeHandle nh, ros::NodeHandle nh_private);
  RosServer(ros::NodeHandle nh, ros::NodeHandle nh_private,
            const RosServerConfig& config);

  void clear();

  MapBase::Ptr getMap() { return occupancy_map_; }
  MapBase::ConstPtr getMap() const { return occupancy_map_; }

  Pipeline& getPipeline() { return *pipeline_; }
  const Pipeline& getPipeline() const { return *pipeline_; }

  MapOperationBase* addOperation(const param::Value& operation_params,
                                 ros::NodeHandle nh_private);

  RosInputBase* addInput(const param::Value& integrator_params,
                         const ros::NodeHandle& nh, ros::NodeHandle nh_private);
  RosInputBase* addInput(std::unique_ptr<RosInputBase> input);
  const std::vector<std::unique_ptr<RosInputBase>>& getInputs() {
    return inputs_;
  }
  void clearInputs() { inputs_.clear(); }

  bool saveMap(const std::filesystem::path& file_path) const;
  bool loadMap(const std::filesystem::path& file_path);

 private:
  const RosServerConfig config_;

  // Map data structure
  MapBase::Ptr occupancy_map_;

  // Threadpool shared among all input handlers and operations
  std::shared_ptr<ThreadPool> thread_pool_;

  // Map management pipeline
  std::shared_ptr<Pipeline> pipeline_;

  // Measurement and pose inputs
  std::vector<std::unique_ptr<RosInputBase>> inputs_;
  std::shared_ptr<TfTransformer> transformer_;

  // ROS services
  void advertiseServices(ros::NodeHandle& nh_private);
  ros::ServiceServer reset_map_srv_;
  ros::ServiceServer save_map_srv_;
  ros::ServiceServer load_map_srv_;
};
}  // namespace wavemap

#endif  // WAVEMAP_ROS_ROS_SERVER_H_
