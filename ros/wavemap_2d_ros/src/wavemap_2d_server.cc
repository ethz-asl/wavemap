#include "wavemap_2d_ros/wavemap_2d_server.h"

#include <sensor_msgs/point_cloud2_iterator.h>
#include <visualization_msgs/MarkerArray.h>
#include <wavemap_2d/data_structure/dense_grid.h>
#include <wavemap_2d/data_structure/volumetric_data_structure_2d_factory.h>
#include <wavemap_2d/integrator/pointcloud_integrator_2d_factory.h>
#include <wavemap_2d/utils/evaluation_utils.h>
#include <wavemap_common/data_structure/volumetric/cell_types/occupancy_cell.h>
#include <wavemap_common/data_structure/volumetric/cell_types/occupancy_state.h>
#include <wavemap_common/utils/nameof.h>
#include <wavemap_common_ros/utils/color.h>
#include <wavemap_common_ros/utils/config_conversions.h>
#include <wavemap_common_ros/utils/visualization_utils.h>

namespace wavemap {
Wavemap2DServer::Wavemap2DServer(ros::NodeHandle nh, ros::NodeHandle nh_private,
                                 const Wavemap2DServer::Config& config)
    : config_(config.checkValid()) {
  // Setup data structure
  const param::Map data_structure_params =
      param::convert::toParamMap(nh_private, "map/data_structure");
  occupancy_map_ = VolumetricDataStructure2DFactory::create(
      data_structure_params, VolumetricDataStructure2DType::kWaveletQuadtree);
  CHECK_NOTNULL(occupancy_map_);

  // Setup integrator
  const param::Map integrator_params =
      param::convert::toParamMap(nh_private, "integrator");
  pointcloud_integrator_ = PointcloudIntegrator2DFactory::create(
      integrator_params, occupancy_map_,
      PointcloudIntegrator2DType::kWaveletScanIntegrator);
  CHECK_NOTNULL(pointcloud_integrator_);

  // Connect to ROS
  subscribeToTimers(nh);
  subscribeToTopics(nh);
  advertiseTopics(nh_private);
  advertiseServices(nh_private);
}

void Wavemap2DServer::pointcloudCallback(
    const sensor_msgs::LaserScan& scan_msg) {
  pointcloud_queue_.emplace(scan_msg);
  processPointcloudQueue();
}

void Wavemap2DServer::visualizeMap() {
  if (occupancy_map_ && !occupancy_map_->empty()) {
    visualization_msgs::MarkerArray occupancy_grid_marker = MapToMarkerArray(
        *occupancy_map_, config_.world_frame, "occupancy_grid",
        [](FloatingPoint cell_log_odds) {
          if (OccupancyState::isObserved(cell_log_odds)) {
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

bool Wavemap2DServer::saveMap(const std::string& file_path) const {
  return !occupancy_map_->empty() &&
         occupancy_map_->save(file_path, kSaveWithFloatingPointPrecision);
}

bool Wavemap2DServer::loadMap(const std::string& file_path) {
  return occupancy_map_->load(file_path, kSaveWithFloatingPointPrecision);
}

bool Wavemap2DServer::evaluateMap(const std::string& file_path) {
  if (!occupancy_map_) {
    ROS_ERROR("The occupancy map has not yet been created.");
    return false;
  }
  occupancy_map_->prune();

  // TODO(victorr): Make it possible to load maps without knowing the resolution
  //                on beforehand (e.g. through a static method)
  constexpr FloatingPoint kGroundTruthMapMinCellWidth = 1e-2f;
  using GTDataStructureType = DenseGrid<UnboundedScalarCell>;
  GTDataStructureType ground_truth_map(
      VolumetricDataStructureConfig{kGroundTruthMapMinCellWidth});
  if (!ground_truth_map.load(file_path, true)) {
    ROS_WARN("Could not load the ground truth map.");
    return false;
  }
  visualization_msgs::MarkerArray occupancy_grid_ground_truth_marker =
      MapToMarkerArray(ground_truth_map, config_.world_frame,
                       "occupancy_grid_ground_truth",
                       [](FloatingPoint gt_occupancy) {
                         if (!OccupancyState::isObserved(gt_occupancy)) {
                           return RGBAColor::Transparent();
                         } else if (gt_occupancy < 0.f) {
                           return RGBAColor::White();
                         } else {
                           return RGBAColor::Black();
                         }
                       });
  occupancy_grid_ground_truth_pub_.publish(occupancy_grid_ground_truth_marker);

  utils::MapEvaluationConfig evaluation_config;
  evaluation_config.iterate_over =
      utils::MapEvaluationConfig::Source::kPredicted;
  evaluation_config.crop_to = utils::MapEvaluationConfig::Source::kReference;
  evaluation_config.reference.cell_selector = {
      utils::CellSelector::Categories::kAny};
  evaluation_config.reference.treat_unknown_cells_as =
      OccupancyState::Occupied();
  evaluation_config.predicted.cell_selector = {
      utils::CellSelector::Categories::kAnyObserved};

  GTDataStructureType error_grid(
      VolumetricDataStructureConfig{kGroundTruthMapMinCellWidth});
  utils::MapEvaluationSummary map_evaluation_summary = utils::EvaluateMap(
      ground_truth_map, *occupancy_map_, evaluation_config, &error_grid);

  if (map_evaluation_summary.is_valid) {
    ROS_INFO_STREAM("Map evaluation overview:\n"
                    << map_evaluation_summary.toString());

    wavemap_msgs::MapEvaluationSummary map_evaluation_summary_msg;
    map_evaluation_summary_msg.is_valid = map_evaluation_summary.is_valid;
    map_evaluation_summary_msg.num_true_positive =
        map_evaluation_summary.num_true_positive;
    map_evaluation_summary_msg.num_true_negative =
        map_evaluation_summary.num_true_negative;
    map_evaluation_summary_msg.num_false_positive =
        map_evaluation_summary.num_false_positive;
    map_evaluation_summary_msg.num_false_negative =
        map_evaluation_summary.num_false_negative;
    map_evaluation_summary_msg.num_cells_ignored =
        map_evaluation_summary.num_cells_ignored;
    map_evaluation_summary_msg.num_cells_evaluated =
        map_evaluation_summary.num_cells_evaluated();
    map_evaluation_summary_msg.num_cells_considered =
        map_evaluation_summary.num_cells_considered();
    map_evaluation_summary_msg.precision = map_evaluation_summary.precision();
    map_evaluation_summary_msg.recall = map_evaluation_summary.recall();
    map_evaluation_summary_msg.f_1_score = map_evaluation_summary.f_1_score();
    map_evaluation_summary_pub_.publish(map_evaluation_summary_msg);

    visualization_msgs::MarkerArray occupancy_error_grid_marker =
        MapToMarkerArray(
            error_grid, config_.world_frame, "occupancy_grid_evaluation",
            [](FloatingPoint error_value) {
              if (error_value < 0.f) {
                return RGBAColor{1.f, 1.f - error_value / 2.f, 0.f, 0.f};
              } else if (0.f < error_value) {
                return RGBAColor{1.f, 0.f, error_value / 2.f, 0.f};
              } else {
                return RGBAColor::Transparent();
              }
            });
    occupancy_grid_error_pub_.publish(occupancy_error_grid_marker);
    return true;
  }

  ROS_WARN("Map evaluation failed.");
  return false;
}

void Wavemap2DServer::processPointcloudQueue() {
  while (!pointcloud_queue_.empty()) {
    const sensor_msgs::LaserScan& scan_msg = pointcloud_queue_.front();

    // Get the sensor pose in world frame
    Transformation3D T_W_C;
    if (!transformer_.isTransformAvailable(config_.world_frame,
                                           scan_msg.header.frame_id,
                                           scan_msg.header.stamp) ||
        !transformer_.lookupTransform(config_.world_frame,
                                      scan_msg.header.frame_id,
                                      scan_msg.header.stamp, T_W_C)) {
      if ((pointcloud_queue_.back().header.stamp - scan_msg.header.stamp)
              .toSec() < config_.pointcloud_queue_max_wait_for_tf_s) {
        // Try to get this pointcloud's pose again at the next iteration
        return;
      } else {
        ROS_WARN_STREAM(
            "Waited "
            << config_.pointcloud_queue_max_wait_for_tf_s
            << "s but still could not look up pose for pointcloud with frame \""
            << scan_msg.header.frame_id << "\" in world frame \""
            << config_.world_frame << "\" at timestamp "
            << scan_msg.header.stamp << "; skipping pointcloud.");
        pointcloud_queue_.pop();
        continue;
      }
    }

    // Convert the scan to a pointcloud
    const size_t num_rays = scan_msg.ranges.size();
    std::vector<Point2D> t_C_points_2d;
    t_C_points_2d.reserve(num_rays);
    for (size_t ray_idx = 0u; ray_idx < num_rays; ++ray_idx) {
      const FloatingPoint ray_range =
          std::min(scan_msg.ranges[ray_idx], scan_msg.range_max);
      const FloatingPoint ray_angle =
          static_cast<FloatingPoint>(ray_idx) * scan_msg.angle_increment +
          scan_msg.angle_min;
      const Point2D ray_endpoint =
          ray_range * Point2D(std::cos(ray_angle), std::sin(ray_angle));
      t_C_points_2d.emplace_back(ray_endpoint);
    }

    // Integrate the pointcloud
    ROS_INFO_STREAM("Inserting pointcloud with " << t_C_points_2d.size()
                                                 << " points");
    const Transformation2D T_W_C_2d(Rotation2D(T_W_C.getRotation().log().z()),
                                    T_W_C.getPosition().head<2>());
    const PosedPointcloud posed_pointcloud(T_W_C_2d,
                                           Pointcloud<Point2D>(t_C_points_2d));
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

void Wavemap2DServer::subscribeToTimers(const ros::NodeHandle& nh) {
  pointcloud_queue_processing_timer_ = nh.createTimer(
      ros::Duration(config_.pointcloud_queue_processing_retry_period_s),
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

void Wavemap2DServer::subscribeToTopics(ros::NodeHandle& nh) {
  pointcloud_sub_ = nh.subscribe(config_.pointcloud_topic_name,
                                 config_.pointcloud_topic_queue_length,
                                 &Wavemap2DServer::pointcloudCallback, this);
}

void Wavemap2DServer::advertiseTopics(ros::NodeHandle& nh_private) {
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

void Wavemap2DServer::advertiseServices(ros::NodeHandle& nh_private) {
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

Wavemap2DServer::Config Wavemap2DServer::Config::fromRosParams(
    ros::NodeHandle nh) {
  Config config;

  // General
  nh.param("general/" + NAMEOF(config.world_frame), config.world_frame,
           config.world_frame);
  nh.param("general/" + NAMEOF(config.publish_performance_stats),
           config.publish_performance_stats, config.publish_performance_stats);
  nh.param(
      "general/" + NAMEOF(config.pointcloud_queue_processing_retry_period_s),
      config.pointcloud_queue_processing_retry_period_s,
      config.pointcloud_queue_processing_retry_period_s);

  // Map
  nh.param("map/pruning_period", config.map_pruning_period_s,
           config.map_pruning_period_s);
  nh.param("map/visualization_period", config.map_visualization_period_s,
           config.map_visualization_period_s);
  nh.param("map/autosave_period", config.map_autosave_period_s,
           config.map_autosave_period_s);
  nh.param("map/autosave_path", config.map_autosave_path,
           config.map_autosave_path);

  // Integrator
  nh.param("integrator/pointcloud_input/topic_name",
           config.pointcloud_topic_name, config.pointcloud_topic_name);
  nh.param("integrator/pointcloud_input/topic_queue_length",
           config.pointcloud_topic_queue_length,
           config.pointcloud_topic_queue_length);
  nh.param("integrator/pointcloud_input/max_wait_for_tf",
           config.pointcloud_queue_max_wait_for_tf_s,
           config.pointcloud_queue_max_wait_for_tf_s);

  // Evaluations
  nh.param("evaluations/" + NAMEOF(config.map_evaluation_period_s),
           config.map_evaluation_period_s, config.map_evaluation_period_s);
  nh.param("evaluations/" + NAMEOF(config.map_ground_truth_path),
           config.map_ground_truth_path, config.map_ground_truth_path);

  return config;
}

bool Wavemap2DServer::Config::isValid(const bool verbose) const {
  bool all_valid = true;

  if (world_frame.empty()) {
    all_valid = false;
    LOG_IF(WARNING, verbose)
        << "Param " << NAMEOF(world_frame) << " must be a non-empty string";
  }

  if (pointcloud_queue_processing_retry_period_s <= 0.f) {
    all_valid = false;
    LOG_IF(WARNING, verbose)
        << "Param " << NAMEOF(pointcloud_queue_processing_retry_period_s)
        << " must be a positive float";
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

  if (pointcloud_queue_max_wait_for_tf_s < 0.f) {
    all_valid = false;
    LOG_IF(WARNING, verbose)
        << "Param " << NAMEOF(pointcloud_queue_max_wait_for_tf_s)
        << " must be a non-negative float";
  }

  return all_valid;
}

}  // namespace wavemap
