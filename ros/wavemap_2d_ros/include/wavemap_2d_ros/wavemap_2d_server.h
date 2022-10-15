#ifndef WAVEMAP_2D_ROS_WAVEMAP_2D_SERVER_H_
#define WAVEMAP_2D_ROS_WAVEMAP_2D_SERVER_H_

#include <queue>
#include <string>

#include <glog/logging.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <std_srvs/Empty.h>
#include <wavemap_2d/data_structure/volumetric_data_structure_2d.h>
#include <wavemap_2d/integrator/pointcloud_integrator_2d.h>
#include <wavemap_common/common.h>
#include <wavemap_common_ros/tf_transformer.h>
#include <wavemap_common_ros/utils/timer.h>
#include <wavemap_msgs/FilePath.h>
#include <wavemap_msgs/MapEvaluationSummary.h>
#include <wavemap_msgs/PerformanceStats.h>

namespace wavemap {

class Wavemap2DServer {
 public:
  struct Config {
    // General
    std::string world_frame = "odom";
    bool publish_performance_stats = false;
    float pointcloud_queue_processing_retry_period_s = 0.1f;

    // Map
    float map_pruning_period_s = 1.f;
    float map_visualization_period_s = 10.f;
    float map_autosave_period_s = -1.f;
    std::string map_autosave_path;

    // Integrator
    std::string pointcloud_topic_name = "scan";
    int pointcloud_topic_queue_length = 10;
    float pointcloud_queue_max_wait_for_tf_s = 1.f;

    // Evaluations
    // NOTE: evaluation will only be performed if map_ground_truth_path is set
    float map_evaluation_period_s = 10.f;
    std::string map_ground_truth_path;

    static Config fromRosParams(ros::NodeHandle nh);
    bool isValid(bool verbose = true) const;
    const Config& checkValid() const {
      CHECK(isValid(true));
      return *this;
    }
  };

  Wavemap2DServer(ros::NodeHandle nh, ros::NodeHandle nh_private)
      : Wavemap2DServer(nh, nh_private, Config::fromRosParams(nh_private)) {}
  Wavemap2DServer(ros::NodeHandle nh, ros::NodeHandle nh_private,
                  const Config& config);

  void pointcloudCallback(const sensor_msgs::LaserScan& scan_msg);

  void visualizeMap();
  bool saveMap(const std::string& file_path) const;
  bool loadMap(const std::string& file_path);
  bool evaluateMap(const std::string& file_path);

 private:
  static constexpr bool kSaveWithFloatingPointPrecision = true;

  const Config config_;

  VolumetricDataStructure2D::Ptr occupancy_map_;
  PointcloudIntegrator2D::Ptr pointcloud_integrator_;
  TfTransformer transformer_;

  void processPointcloudQueue();
  std::queue<sensor_msgs::LaserScan> pointcloud_queue_;
  CpuTimer integration_timer;

  void subscribeToTimers(const ros::NodeHandle& nh);
  ros::Timer pointcloud_queue_processing_timer_;
  ros::Timer map_pruning_timer_;
  ros::Timer map_visualization_timer_;
  ros::Timer map_evaluation_timer_;
  ros::Timer map_autosave_timer_;

  void subscribeToTopics(ros::NodeHandle& nh);
  ros::Subscriber pointcloud_sub_;

  void advertiseTopics(ros::NodeHandle& nh_private);
  ros::Publisher occupancy_grid_pub_;
  ros::Publisher occupancy_grid_error_pub_;
  ros::Publisher occupancy_grid_ground_truth_pub_;
  ros::Publisher map_evaluation_summary_pub_;
  ros::Publisher performance_stats_pub_;

  void advertiseServices(ros::NodeHandle& nh_private);
  ros::ServiceServer visualize_map_srv_;
  ros::ServiceServer save_map_srv_;
  ros::ServiceServer load_map_srv_;
  ros::ServiceServer evaluate_map_srv_;
};
}  // namespace wavemap

#endif  // WAVEMAP_2D_ROS_WAVEMAP_2D_SERVER_H_
