#include "wavemap_3d_ros/wavemap_3d_server.h"

#include <sensor_msgs/point_cloud2_iterator.h>
#include <std_srvs/Empty.h>
#include <visualization_msgs/MarkerArray.h>
#include <wavemap_3d/data_structure/volumetric_data_structure_3d_factory.h>
#include <wavemap_3d/integrator/pointcloud_integrator_3d_factory.h>
#include <wavemap_common/data_structure/pointcloud.h>
#include <wavemap_common_ros/utils/nameof.h>
#include <wavemap_common_ros/utils/visualization_utils.h>
#include <wavemap_msgs/FilePath.h>
#include <wavemap_msgs/MapEvaluationSummary.h>
#include <wavemap_msgs/PerformanceStats.h>

namespace wavemap {
Wavemap3DServer::Wavemap3DServer(ros::NodeHandle nh, ros::NodeHandle nh_private,
                                 wavemap::Wavemap3DServer::Config config)
    : config_(std::move(config)) {
  // Assert that the config is valid
  CHECK(config_.isValid(true));

  // Setup data structure and integrator
  occupancy_map_ = VolumetricDataStructure3DFactory::create(
      config_.data_structure_type, config_.min_cell_width,
      VolumetricDataStructure3DType::kHashedBlocks);
  CHECK_NOTNULL(occupancy_map_);

  pointcloud_integrator_ = PointcloudIntegrator3DFactory::create(
      config_.measurement_model_type, occupancy_map_,
      PointcloudIntegrator3DType::kSingleRayIntegrator);
  CHECK_NOTNULL(pointcloud_integrator_);

  // Connect to ROS
  subscribeToTimers(nh);
  subscribeToTopics(nh);
  advertiseTopics(nh_private);
  advertiseServices(nh_private);
}

void Wavemap3DServer::pointcloudCallback(
    const sensor_msgs::PointCloud2& scan_msg) {
  pointcloud_queue_.emplace(scan_msg);
  processPointcloudQueue();
}

void Wavemap3DServer::visualizeMap() {
  if (occupancy_map_ && !occupancy_map_->empty()) {
    visualization_msgs::MarkerArray occupancy_grid_marker = MapToMarkerArray(
        *occupancy_map_, config_.world_frame, "occupancy_grid",
        [](FloatingPoint cell_log_odds) {
          if (OccupancyState::isObserved(cell_log_odds) &&
              0.f < cell_log_odds) {
            const FloatingPoint cell_odds = std::exp(cell_log_odds);
            const FloatingPoint cell_prob = cell_odds / (1.f + cell_odds);
            const FloatingPoint cell_free_prob = 1.f - cell_prob;
            return RGBAColor{1.f, cell_free_prob, cell_free_prob,
                             cell_free_prob};
          }
          return RGBAColor::Transparent();
        });
    occupancy_grid_pub_.publish(occupancy_grid_marker);
  }
}

bool Wavemap3DServer::saveMap(const std::string& file_path) const {
  return !occupancy_map_->empty() &&
         occupancy_map_->save(file_path, kSaveWithFloatingPointPrecision);
}

bool Wavemap3DServer::loadMap(const std::string& file_path) {
  return occupancy_map_->load(file_path, kSaveWithFloatingPointPrecision);
}

bool Wavemap3DServer::evaluateMap(const std::string& /*file_path*/) {
  LOG(WARNING) << "Evaluations are not yet implemented for wavemap 3D";
  return false;
}

void Wavemap3DServer::processPointcloudQueue() {
  while (!pointcloud_queue_.empty()) {
    const sensor_msgs::PointCloud2& scan_msg = pointcloud_queue_.front();

    // Get the sensor pose in world frame
    Transformation3D T_W_C;
    if (!transformer_.isTransformAvailable(config_.world_frame,
                                           scan_msg.header.frame_id,
                                           scan_msg.header.stamp) ||
        !transformer_.lookupTransform(config_.world_frame,
                                      scan_msg.header.frame_id,
                                      scan_msg.header.stamp, T_W_C)) {
      if ((pointcloud_queue_.back().header.stamp - scan_msg.header.stamp)
              .toSec() < config_.pointcloud_queue_max_wait_for_transform_s) {
        // Try to get this pointcloud's pose again at the next iteration
        return;
      } else {
        ROS_WARN_STREAM(
            "Waited "
            << config_.pointcloud_queue_max_wait_for_transform_s
            << "s but still could not look up pose for pointcloud with frame \""
            << scan_msg.header.frame_id << "\" in world frame \""
            << config_.world_frame << "\" at timestamp "
            << scan_msg.header.stamp << "; skipping pointcloud.");
        pointcloud_queue_.pop();
        continue;
      }
    }

    // Convert the scan to a pointcloud
    // Get the index of the x field, and assert that the y and z fields follow
    auto x_field_iter = std::find_if(
        scan_msg.fields.begin(), scan_msg.fields.end(),
        [](const sensor_msgs::PointField& field) { return field.name == "x"; });
    if (x_field_iter == scan_msg.fields.end()) {
      ROS_WARN("Received pointcloud with missing field x");
      return;
    } else if ((++x_field_iter)->name != "y") {
      ROS_WARN("Received pointcloud with missing or out-of-order field y");
      return;
    } else if ((++x_field_iter)->name != "z") {
      ROS_WARN("Received pointcloud with missing or out-of-order field z");
      return;
    }
    // Load the points into our internal Pointcloud type
    const size_t num_rays = scan_msg.width * scan_msg.height;
    std::vector<Point3D> t_C_points;
    t_C_points.reserve(num_rays);
    for (sensor_msgs::PointCloud2ConstIterator<float> it(scan_msg, "x");
         it != it.end(); ++it) {
      t_C_points.emplace_back(it[0], it[1], it[2]);
    }

    // Integrate the pointcloud
    ROS_INFO_STREAM("Inserting pointcloud with "
                    << t_C_points.size()
                    << " points. Remaining pointclouds in queue: "
                    << pointcloud_queue_.size() - 1 << ".");
    const PosedPointcloud posed_pointcloud(T_W_C,
                                           Pointcloud<Point3D>(t_C_points));
    integration_timer.start();
    pointcloud_integrator_->integratePointcloud(posed_pointcloud);
    const double pointcloud_integration_time = integration_timer.stop();
    const double total_pointcloud_integration_time =
        integration_timer.getTotal();
    ROS_INFO_STREAM("Integrated new pointcloud in "
                    << pointcloud_integration_time
                    << "s. Total integration time: "
                    << total_pointcloud_integration_time << "s.");

    if (config_.publish_performance_stats) {
      const size_t map_memory_usage = occupancy_map_->getMemoryUsage();
      ROS_INFO_STREAM("Map memory usage: " << map_memory_usage / 1e6 << "MB.");

      wavemap_msgs::PerformanceStats performance_stats_msg;
      performance_stats_msg.map_memory_usage = map_memory_usage;
      performance_stats_msg.total_pointcloud_integration_time =
          total_pointcloud_integration_time;
      performance_stats_pub_.publish(performance_stats_msg);
    }

    // Remove the pointcloud from the queue
    pointcloud_queue_.pop();
  }
}

void Wavemap3DServer::subscribeToTimers(const ros::NodeHandle& nh) {
  pointcloud_queue_processing_timer_ = nh.createTimer(
      ros::Duration(config_.pointcloud_queue_processing_period_s),
      [this](const auto& /*event*/) { processPointcloudQueue(); });

  if (0.f < config_.map_pruning_period_s) {
    ROS_INFO_STREAM("Registering map pruning timer with period "
                    << config_.map_pruning_period_s << "s");
    map_pruning_timer_ = nh.createTimer(
        ros::Duration(config_.map_pruning_period_s),
        [this](const auto& /*event*/) { occupancy_map_->prune(); });
  }

  if (0.f < config_.map_visualization_period_s) {
    ROS_INFO_STREAM("Registering map visualization timer with period "
                    << config_.map_visualization_period_s << "s");
    map_visualization_timer_ =
        nh.createTimer(ros::Duration(config_.map_visualization_period_s),
                       [this](const auto& /*event*/) { visualizeMap(); });
  }

  if (0.f < config_.map_evaluation_period_s &&
      !config_.map_ground_truth_path.empty()) {
    ROS_INFO_STREAM("Registering map evaluation timer with period "
                    << config_.map_evaluation_period_s << "s");
    map_evaluation_timer_ =
        nh.createTimer(ros::Duration(config_.map_evaluation_period_s),
                       [this](const auto& /*event*/) {
                         evaluateMap(config_.map_ground_truth_path);
                       });
  }

  if (0.f < config_.map_autosave_period_s &&
      !config_.map_autosave_path.empty()) {
    ROS_INFO_STREAM("Registering autosave timer with period "
                    << config_.map_autosave_period_s << "s");
    map_autosave_timer_ = nh.createTimer(
        ros::Duration(config_.map_autosave_period_s),
        [this](const auto& /*event*/) { saveMap(config_.map_autosave_path); });
  }
}

void Wavemap3DServer::subscribeToTopics(ros::NodeHandle& nh) {
  pointcloud_sub_ = nh.subscribe(config_.pointcloud_topic_name,
                                 config_.pointcloud_topic_queue_length,
                                 &Wavemap3DServer::pointcloudCallback, this);
}

void Wavemap3DServer::advertiseTopics(ros::NodeHandle& nh_private) {
  occupancy_grid_pub_ = nh_private.advertise<visualization_msgs::MarkerArray>(
      "occupancy_grid", 10, true);
  occupancy_grid_error_pub_ =
      nh_private.advertise<visualization_msgs::MarkerArray>(
          "occupancy_grid_error", 10, true);
  occupancy_grid_ground_truth_pub_ =
      nh_private.advertise<visualization_msgs::MarkerArray>(
          "occupancy_grid_ground_truth", 10, true);
  map_evaluation_summary_pub_ =
      nh_private.advertise<wavemap_msgs::MapEvaluationSummary>(
          "map_evaluation_summary", 10, true);
  performance_stats_pub_ = nh_private.advertise<wavemap_msgs::PerformanceStats>(
      "performance_stats", 10, true);
}

void Wavemap3DServer::advertiseServices(ros::NodeHandle& nh_private) {
  visualize_map_srv_ = nh_private.advertiseService<std_srvs::Empty::Request,
                                                   std_srvs::Empty::Response>(
      "visualize_map", [this](auto& /*request*/, auto& /*response*/) {
        visualizeMap();
        return true;
      });

  save_map_srv_ = nh_private.advertiseService<wavemap_msgs::FilePath::Request,
                                              wavemap_msgs::FilePath::Response>(
      "save_map", [this](auto& request, auto& response) {
        response.success = saveMap(request.file_path);
        return true;
      });

  load_map_srv_ = nh_private.advertiseService<wavemap_msgs::FilePath::Request,
                                              wavemap_msgs::FilePath::Response>(
      "load_map", [this](auto& request, auto& response) {
        response.success = loadMap(request.file_path);
        return true;
      });

  evaluate_map_srv_ =
      nh_private.advertiseService<wavemap_msgs::FilePath::Request,
                                  wavemap_msgs::FilePath::Response>(
          "evaluate_map", [this](auto& request, auto& response) {
            response.success = evaluateMap(request.file_path);
            return true;
          });
}

Wavemap3DServer::Config Wavemap3DServer::Config::fromRosParams(
    ros::NodeHandle nh) {
  Config config;

  nh.param(NAMEOF(config.min_cell_width), config.min_cell_width,
           config.min_cell_width);

  nh.param(NAMEOF(config.world_frame), config.world_frame, config.world_frame);

  nh.param(NAMEOF(config.data_structure_type), config.data_structure_type,
           config.data_structure_type);
  nh.param(NAMEOF(config.measurement_model_type), config.measurement_model_type,
           config.measurement_model_type);

  nh.param(NAMEOF(config.pointcloud_topic_name), config.pointcloud_topic_name,
           config.pointcloud_topic_name);
  nh.param(NAMEOF(config.pointcloud_topic_queue_length),
           config.pointcloud_topic_queue_length,
           config.pointcloud_topic_queue_length);

  nh.param(NAMEOF(config.map_pruning_period_s), config.map_pruning_period_s,
           config.map_pruning_period_s);

  nh.param(NAMEOF(config.map_visualization_period_s),
           config.map_visualization_period_s,
           config.map_visualization_period_s);

  nh.param(NAMEOF(config.map_evaluation_period_s),
           config.map_evaluation_period_s, config.map_evaluation_period_s);
  nh.param(NAMEOF(config.map_ground_truth_path), config.map_ground_truth_path,
           config.map_ground_truth_path);

  nh.param(NAMEOF(config.map_autosave_period_s), config.map_autosave_period_s,
           config.map_autosave_period_s);
  nh.param(NAMEOF(config.map_autosave_path), config.map_autosave_path,
           config.map_autosave_path);

  nh.param(NAMEOF(config.publish_performance_stats),
           config.publish_performance_stats, config.publish_performance_stats);

  nh.param(NAMEOF(config.pointcloud_queue_processing_period_s),
           config.pointcloud_queue_processing_period_s,
           config.pointcloud_queue_processing_period_s);
  nh.param(NAMEOF(config.pointcloud_queue_max_wait_for_transform_s),
           config.pointcloud_queue_max_wait_for_transform_s,
           config.pointcloud_queue_max_wait_for_transform_s);

  return config;
}

bool Wavemap3DServer::Config::isValid(bool verbose) {
  bool all_valid = true;

  if (min_cell_width <= 0.f) {
    all_valid = false;
    LOG_IF(WARNING, verbose)
        << "Param " << NAMEOF(min_cell_width) << " must be a positive float";
  }

  if (world_frame.empty()) {
    all_valid = false;
    LOG_IF(WARNING, verbose)
        << "Param " << NAMEOF(world_frame) << " must be a non-empty string";
  }

  if (pointcloud_topic_name.empty()) {
    all_valid = false;
    LOG_IF(WARNING, verbose) << "Param " << NAMEOF(pointcloud_topic_name)
                             << " must be a non-empty string";
  }
  if (pointcloud_topic_queue_length <= 0) {
    all_valid = false;
    LOG_IF(WARNING, verbose)
        << "Param " << NAMEOF(pointcloud_topic_queue_length)
        << " must be a positive integer";
  }

  if (pointcloud_queue_processing_period_s <= 0.f) {
    all_valid = false;
    LOG_IF(WARNING, verbose)
        << "Param " << NAMEOF(pointcloud_queue_processing_period_s)
        << " must be a positive float";
  }

  if (pointcloud_queue_max_wait_for_transform_s < 0.f) {
    all_valid = false;
    LOG_IF(WARNING, verbose)
        << "Param " << NAMEOF(pointcloud_queue_max_wait_for_transform_s)
        << " must be a non-negative float";
  }

  return all_valid;
}
}  // namespace wavemap
