#include "wavemap_2d_ros/wavemap_2d_server.h"

#include <sensor_msgs/point_cloud2_iterator.h>
#include <wavemap_2d/utils/evaluation_utils.h>
#include <wavemap_2d_ros/utils/nameof.h>

namespace wavemap_2d {
Wavemap2DServer::Wavemap2DServer(ros::NodeHandle nh, ros::NodeHandle nh_private,
                                 Wavemap2DServer::Config config)
    : config_(config) {
  // Assert that the config is valid
  CHECK(config_.isValid(/*verbose*/ true));

  // Setup integrator
  occupancy_map_ = std::make_shared<DataStructureType>(config_.map_resolution);
  measurement_model_ =
      std::make_shared<MeasurementModelType>(config_.map_resolution);
  pointcloud_integrator_ = std::make_shared<PointcloudIntegrator>(
      occupancy_map_, measurement_model_);

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
    const size_t num_rays = scan_msg.ranges.size();
    std::vector<Point> t_C_points_2d;
    t_C_points_2d.reserve(num_rays);
    for (size_t ray_idx = 0u; ray_idx < num_rays; ++ray_idx) {
      const FloatingPoint ray_range =
          std::min(scan_msg.ranges[ray_idx], scan_msg.range_max);
      const FloatingPoint ray_angle =
          static_cast<FloatingPoint>(ray_idx) * scan_msg.angle_increment +
          scan_msg.angle_min;
      const Point ray_endpoint =
          ray_range * Point(std::cos(ray_angle), std::sin(ray_angle));
      t_C_points_2d.emplace_back(ray_endpoint);
    }

    // Integrate the pointcloud
    ROS_INFO_STREAM("Inserting pointcloud with " << t_C_points_2d.size()
                                                 << " points");
    const Transformation T_W_C_2d(Rotation(T_W_C.getRotation().log().z()),
                                  T_W_C.getPosition().head<2>());
    const PosedPointcloud posed_pointcloud(T_W_C_2d, Pointcloud(t_C_points_2d));
    pointcloud_integrator_->integratePointcloud(posed_pointcloud);

    // Remove the pointcloud from the queue
    pointcloud_queue_.pop();
  }
}

bool Wavemap2DServer::evaluateMap(const std::string& file_path) {
  DataStructureType ground_truth_map(occupancy_map_->getResolution());
  if (ground_truth_map.load(file_path, true)) {
    utils::EvaluationCellSelector evaluation_cell_selector;
    auto unknown_cell_handling = utils::UnknownCellHandling::kIgnore;
    utils::MapEvaluationSummary map_evaluation_summary = utils::EvaluateMap(
        ground_truth_map, *occupancy_map_, evaluation_cell_selector,
        unknown_cell_handling, true);
    if (map_evaluation_summary.is_valid) {
      ROS_INFO_STREAM("Map evaluation overview:\n"
                      << map_evaluation_summary.toString());
    } else {
      ROS_WARN("Map evaluation failed.");
    }
    return map_evaluation_summary.is_valid;
  }
  return false;
}

void Wavemap2DServer::subscribeToTimers(ros::NodeHandle nh) {
  pointcloud_queue_processing_timer_ = nh.createTimer(
      ros::Duration(config_.pointcloud_queue_processing_period_s),
      std::bind(&Wavemap2DServer::processPointcloudQueue, this));

  if (0.f < config_.map_visualization_period_s) {
    ROS_INFO_STREAM("Registering map visualization timer with period "
                    << config_.map_visualization_period_s << "s");
    map_visualization_timer_ =
        nh.createTimer(ros::Duration(config_.map_visualization_period_s),
                       std::bind(&Wavemap2DServer::visualizeMap, this));
  }

  if (0.f < config_.map_autosave_period_s &&
      !config_.map_autosave_path.empty()) {
    ROS_INFO_STREAM("Registering autosave timer with period "
                    << config_.map_autosave_period_s << "s");
    map_autosave_timer_ = nh.createTimer(
        ros::Duration(config_.map_autosave_period_s),
        std::bind(&Wavemap2DServer::saveMap, this, config_.map_autosave_path));
  }
}

void Wavemap2DServer::subscribeToTopics(ros::NodeHandle nh) {
  pointcloud_sub_ = nh.subscribe(config_.pointcloud_topic_name,
                                 config_.pointcloud_topic_queue_length,
                                 &Wavemap2DServer::pointcloudCallback, this);
}

void Wavemap2DServer::advertiseTopics(ros::NodeHandle /* nh_private */) {}

void Wavemap2DServer::advertiseServices(ros::NodeHandle nh_private) {
  save_map_srv_ = nh_private.advertiseService(
      "save_map", &Wavemap2DServer::saveMapCallback, this);
  load_map_srv_ = nh_private.advertiseService(
      "load_map", &Wavemap2DServer::loadMapCallback, this);
  evaluate_map_srv_ = nh_private.advertiseService(
      "evaluate_map", &Wavemap2DServer::evaluateMapCallback, this);
}

void Wavemap2DServer::visualizeMap() {
  if (!occupancy_map_->empty()) {
    ROS_INFO_STREAM("Showing map of size " << occupancy_map_->dimensions());
    occupancy_map_->showImage(true);
  }
}

Wavemap2DServer::Config Wavemap2DServer::Config::fromRosParams(
    ros::NodeHandle nh) {
  Config config;

  nh.param(NAMEOF(config.map_resolution), config.map_resolution,
           config.map_resolution);

  nh.param(NAMEOF(config.world_frame), config.world_frame, config.world_frame);

  nh.param(NAMEOF(config.pointcloud_topic_name), config.pointcloud_topic_name,
           config.pointcloud_topic_name);
  nh.param(NAMEOF(config.pointcloud_topic_queue_length),
           config.pointcloud_topic_queue_length,
           config.pointcloud_topic_queue_length);

  nh.param(NAMEOF(config.map_visualization_period_s),
           config.map_visualization_period_s,
           config.map_visualization_period_s);

  nh.param(NAMEOF(config.map_autosave_period_s), config.map_autosave_period_s,
           config.map_autosave_period_s);
  nh.param(NAMEOF(config.map_autosave_path), config.map_autosave_path,
           config.map_autosave_path);

  nh.param(NAMEOF(config.pointcloud_queue_processing_period_s),
           config.pointcloud_queue_processing_period_s,
           config.pointcloud_queue_processing_period_s);
  nh.param(NAMEOF(config.pointcloud_queue_max_wait_for_transform_s),
           config.pointcloud_queue_max_wait_for_transform_s,
           config.pointcloud_queue_max_wait_for_transform_s);

  return config;
}

bool Wavemap2DServer::Config::isValid(const bool verbose) {
  bool all_valid = true;

  if (map_resolution <= 0.f) {
    all_valid = false;
    LOG_IF(WARNING, verbose)
        << "Param " << NAMEOF(map_resolution) << " must be a positive float";
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

  if (map_visualization_period_s < 0.f) {
    all_valid = false;
    LOG_IF(WARNING, verbose) << "Param " << NAMEOF(map_visualization_period_s)
                             << " must be a non-negative float";
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

}  // namespace wavemap_2d
