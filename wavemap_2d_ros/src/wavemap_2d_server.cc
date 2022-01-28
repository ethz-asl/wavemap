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
  if (!occupancy_map_) {
    ROS_ERROR("The occupancy map has not yet been created.");
    return false;
  }
  const FloatingPoint resolution = occupancy_map_->getResolution();

  DataStructureType ground_truth_map(resolution);
  if (!ground_truth_map.load(file_path, true)) {
    ROS_WARN("Could not load the ground truth map.");
    return false;
  }
  visualization_msgs::Marker occupancy_grid_ground_truth_marker = gridToMarker(
      ground_truth_map, config_.world_frame, "occupancy_grid_ground_truth",
      [](FloatingPoint gt_occupancy) {
        if (std::abs(gt_occupancy) < kEpsilon) {
          return kTransparent;
        } else if (gt_occupancy < 0.f) {
          return kWhite;
        } else {
          return kBlack;
        }
      });
  occupancy_grid_ground_truth_pub_.publish(occupancy_grid_ground_truth_marker);

  utils::EvaluationCellSelector evaluation_cell_selector;
  auto unknown_cell_handling = utils::UnknownCellHandling::kIgnore;
  DataStructureType error_grid(resolution);
  utils::MapEvaluationSummary map_evaluation_summary = utils::EvaluateMap(
      ground_truth_map, *occupancy_map_, evaluation_cell_selector,
      unknown_cell_handling, &error_grid);

  if (map_evaluation_summary.is_valid) {
    ROS_INFO_STREAM("Map evaluation overview:\n"
                    << map_evaluation_summary.toString());
    visualization_msgs::Marker occupancy_error_grid_marker = gridToMarker(
        error_grid, config_.world_frame, "occupancy_grid_evaluation",
        [](FloatingPoint error_value) {
          if (error_value < 0.f) {
            return RGBAColor{1.f, error_value / 2.f, 0.f, 0.f};
          } else if (0.f < error_value) {
            return RGBAColor{1.f, 0.f, error_value / 2.f, 0.f};
          } else {
            return kTransparent;
          }
        });
    occupancy_grid_error_pub_.publish(occupancy_error_grid_marker);
    return true;
  }

  ROS_WARN("Map evaluation failed.");
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

void Wavemap2DServer::advertiseTopics(ros::NodeHandle nh_private) {
  occupancy_grid_pub_ = nh_private.advertise<visualization_msgs::Marker>(
      "occupancy_grid", 10, true);
  occupancy_grid_error_pub_ = nh_private.advertise<visualization_msgs::Marker>(
      "occupancy_grid_error", 10, true);
  occupancy_grid_ground_truth_pub_ =
      nh_private.advertise<visualization_msgs::Marker>(
          "occupancy_grid_ground_truth", 10, true);
}

void Wavemap2DServer::advertiseServices(ros::NodeHandle nh_private) {
  visualize_map_srv_ = nh_private.advertiseService(
      "visualize_map", &Wavemap2DServer::visualizeMapCallback, this);
  save_map_srv_ = nh_private.advertiseService(
      "save_map", &Wavemap2DServer::saveMapCallback, this);
  load_map_srv_ = nh_private.advertiseService(
      "load_map", &Wavemap2DServer::loadMapCallback, this);
  evaluate_map_srv_ = nh_private.advertiseService(
      "evaluate_map", &Wavemap2DServer::evaluateMapCallback, this);
}

void Wavemap2DServer::visualizeMap() {
  if (occupancy_map_ && !occupancy_map_->empty()) {
    visualization_msgs::Marker occupancy_grid_marker = gridToMarker(
        *occupancy_map_, config_.world_frame, "occupancy_grid",
        [](FloatingPoint cell_log_odds) {
          if (std::abs(cell_log_odds) < kEpsilon) {
            return kTransparent;
          }
          const FloatingPoint cell_odds = std::exp(cell_log_odds);
          const FloatingPoint cell_prob = cell_odds / (1.f + cell_odds);
          const FloatingPoint cell_free_prob = 1.f - cell_prob;
          return RGBAColor{1.f, cell_free_prob, cell_free_prob, cell_free_prob};
        });
    occupancy_grid_pub_.publish(occupancy_grid_marker);
  }
}

visualization_msgs::Marker Wavemap2DServer::gridToMarker(
    const Wavemap2DServer::DataStructureType& grid,
    const std::string& world_frame, const std::string& marker_namespace,
    const std::function<RGBAColor(FloatingPoint)>& color_map) {
  const FloatingPoint resolution = grid.getResolution();

  visualization_msgs::Marker grid_marker;
  grid_marker.header.frame_id = world_frame;
  grid_marker.header.stamp = ros::Time();
  grid_marker.ns = marker_namespace;
  grid_marker.id = 0;
  grid_marker.type = visualization_msgs::Marker::POINTS;
  grid_marker.action = visualization_msgs::Marker::MODIFY;
  grid_marker.pose.orientation.w = 1.0;
  grid_marker.scale.x = resolution;
  grid_marker.scale.y = resolution;
  grid_marker.scale.z = resolution;
  grid_marker.color.a = 1.0;

  for (const Index& index : Grid(grid.getMinIndex(), grid.getMaxIndex())) {
    const Point cell_position = index.cast<FloatingPoint>() * resolution;
    geometry_msgs::Point position_msg;
    position_msg.x = cell_position.x();
    position_msg.y = cell_position.y();
    position_msg.z = 0.0;
    grid_marker.points.emplace_back(position_msg);

    const FloatingPoint cell_value = grid.getCellValue(index);
    const RGBAColor color = color_map(cell_value);
    std_msgs::ColorRGBA color_msg;
    color_msg.a = color.a;
    color_msg.r = color.r;
    color_msg.g = color.g;
    color_msg.b = color.b;
    grid_marker.colors.emplace_back(color_msg);
  }

  return grid_marker;
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
