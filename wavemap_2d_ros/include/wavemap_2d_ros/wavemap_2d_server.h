#ifndef WAVEMAP_2D_ROS_WAVEMAP_2D_SERVER_H_
#define WAVEMAP_2D_ROS_WAVEMAP_2D_SERVER_H_

#include <memory>
#include <queue>
#include <string>

#include <glog/logging.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <std_srvs/Empty.h>
#include <visualization_msgs/Marker.h>
#include <wavemap_2d/common.h>
#include <wavemap_2d/datastructure/cell.h>
#include <wavemap_2d/integrator/pointcloud_integrator.h>
#include <wavemap_2d_msgs/FilePath.h>
#include <wavemap_2d_msgs/MapEvaluationSummary.h>
#include <wavemap_2d_msgs/PerformanceStats.h>

#include "wavemap_2d_ros/tf_transformer.h"
#include "wavemap_2d_ros/utils/timer.h"

namespace wavemap_2d {

class Wavemap2DServer {
 public:
  struct Config {
    float map_resolution = 0.f;

    std::string world_frame = "odom";

    std::string data_structure_type = "dense_grid";
    std::string measurement_model_type = "beam_model";

    std::string pointcloud_topic_name = "scan";
    int pointcloud_topic_queue_length = 10;

    float map_visualization_period_s = 10.f;

    // NOTE: evaluation will only be performed if map_ground_truth_path is set
    float map_evaluation_period_s = 10.f;
    std::string map_ground_truth_path;

    float map_autosave_period_s = -1.f;
    std::string map_autosave_path;

    float pointcloud_queue_processing_period_s = 0.1f;
    float pointcloud_queue_max_wait_for_transform_s = 1.f;

    static Config fromRosParams(ros::NodeHandle nh);
    bool isValid(bool verbose = true);
  };

  Wavemap2DServer(ros::NodeHandle nh, ros::NodeHandle nh_private)
      : Wavemap2DServer(nh, nh_private, Config::fromRosParams(nh_private)) {}
  Wavemap2DServer(ros::NodeHandle nh, ros::NodeHandle nh_private,
                  Config config);

  void pointcloudCallback(const sensor_msgs::LaserScan& scan_msg);
  bool visualizeMapCallback(std_srvs::Empty::Request& /* request */,
                            std_srvs::Empty::Response& /* response */) {
    visualizeMap();
    return true;
  }
  bool saveMapCallback(wavemap_2d_msgs::FilePath::Request& request,
                       wavemap_2d_msgs::FilePath::Response& response) {
    response.success = saveMap(request.file_path);
    return true;
  }
  bool loadMapCallback(wavemap_2d_msgs::FilePath::Request& request,
                       wavemap_2d_msgs::FilePath::Response& response) {
    response.success = loadMap(request.file_path);
    return true;
  }
  bool evaluateMapCallback(wavemap_2d_msgs::FilePath::Request& request,
                           wavemap_2d_msgs::FilePath::Response& response) {
    response.success = evaluateMap(request.file_path);
    return true;
  }

  void visualizeMap();
  bool saveMap(const std::string& file_path) {
    return !occupancy_map_->empty() &&
           occupancy_map_->save(file_path, kSaveWithFloatingPointPrecision);
  }
  bool loadMap(const std::string& file_path) {
    return !occupancy_map_->empty() &&
           occupancy_map_->save(file_path, kSaveWithFloatingPointPrecision);
  }
  bool evaluateMap(const std::string& file_path);

 protected:
  static constexpr bool kSaveWithFloatingPointPrecision = true;

  Config config_;

  std::shared_ptr<DataStructureBase> occupancy_map_;
  std::shared_ptr<MeasurementModelBase> measurement_model_;
  std::shared_ptr<PointcloudIntegrator> pointcloud_integrator_;
  TfTransformer transformer_;

  std::queue<sensor_msgs::LaserScan> pointcloud_queue_;
  void processPointcloudQueue();
  CpuTimer integration_timer;

  ros::Timer pointcloud_queue_processing_timer_;
  ros::Timer map_visualization_timer_;
  ros::Timer map_evaluation_timer_;
  ros::Timer map_autosave_timer_;
  ros::Subscriber pointcloud_sub_;
  ros::Publisher occupancy_grid_pub_;
  ros::Publisher occupancy_grid_error_pub_;
  ros::Publisher occupancy_grid_ground_truth_pub_;
  ros::Publisher map_evaluation_summary_pub_;
  ros::Publisher performance_stats_pub_;
  ros::ServiceServer visualize_map_srv_;
  ros::ServiceServer save_map_srv_;
  ros::ServiceServer load_map_srv_;
  ros::ServiceServer evaluate_map_srv_;
  void subscribeToTimers(const ros::NodeHandle& nh);
  void subscribeToTopics(ros::NodeHandle& nh);
  void advertiseTopics(ros::NodeHandle& nh_private);
  void advertiseServices(ros::NodeHandle& nh_private);

  struct RGBAColor {
    FloatingPoint a;
    FloatingPoint r;
    FloatingPoint g;
    FloatingPoint b;
  };
  static constexpr RGBAColor kTransparent{0.f, 0.f, 0.f, 0.f};
  static constexpr RGBAColor kWhite{1.f, 1.f, 1.f, 1.f};
  static constexpr RGBAColor kBlack{1.f, 0.f, 0.f, 0.f};
  template <typename Map>
  static visualization_msgs::Marker gridToMarker(
      const Map& grid, const std::string& world_frame,
      const std::string& marker_namespace,
      const std::function<RGBAColor(FloatingPoint)>& color_map);
};
}  // namespace wavemap_2d

#include "wavemap_2d_ros/wavemap_2d_server_inl.h"

#endif  // WAVEMAP_2D_ROS_WAVEMAP_2D_SERVER_H_
