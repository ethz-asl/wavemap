#ifndef WAVEMAP_3D_ROS_WAVEMAP_3D_SERVER_H_
#define WAVEMAP_3D_ROS_WAVEMAP_3D_SERVER_H_

#include <queue>
#include <string>

#include <glog/logging.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <wavemap_3d/data_structure/volumetric_data_structure_3d.h>
#include <wavemap_3d/integrator/pointcloud_integrator.h>
#include <wavemap_common/common.h>
#include <wavemap_common_ros/tf_transformer.h>
#include <wavemap_common_ros/utils/timer.h>

namespace wavemap {
class Wavemap3DServer {
 public:
  struct Config {
    float min_cell_width = 0.f;

    std::string world_frame = "odom";

    std::string data_structure_type = "dense_grid";
    std::string measurement_model_type = "beam_model";

    std::string pointcloud_topic_name = "scan";
    int pointcloud_topic_queue_length = 10;

    float map_pruning_period_s = 1.f;

    float map_visualization_period_s = 10.f;

    // NOTE: evaluation will only be performed if map_ground_truth_path is set
    float map_evaluation_period_s = 10.f;
    std::string map_ground_truth_path;

    float map_autosave_period_s = -1.f;
    std::string map_autosave_path;

    bool publish_performance_stats = false;

    float pointcloud_queue_processing_period_s = 0.1f;
    float pointcloud_queue_max_wait_for_transform_s = 1.f;

    static Config fromRosParams(ros::NodeHandle nh);
    bool isValid(bool verbose = true);
  };

  Wavemap3DServer(ros::NodeHandle nh, ros::NodeHandle nh_private)
      : Wavemap3DServer(nh, nh_private, Config::fromRosParams(nh_private)) {}
  Wavemap3DServer(ros::NodeHandle nh, ros::NodeHandle nh_private,
                  Config config);

  void pointcloudCallback(const sensor_msgs::PointCloud2& scan_msg);

  void visualizeMap();
  bool saveMap(const std::string& file_path) const;
  bool loadMap(const std::string& file_path);
  bool evaluateMap(const std::string& file_path);

 private:
  static constexpr bool kSaveWithFloatingPointPrecision = true;

  Config config_;

  VolumetricDataStructure3D::Ptr occupancy_map_;
  PointcloudIntegrator::Ptr pointcloud_integrator_;
  TfTransformer transformer_;

  void processPointcloudQueue();
  std::queue<sensor_msgs::PointCloud2> pointcloud_queue_;
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

#endif  // WAVEMAP_3D_ROS_WAVEMAP_3D_SERVER_H_
