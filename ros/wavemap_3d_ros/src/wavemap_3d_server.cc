#include "wavemap_3d_ros/wavemap_3d_server.h"

#include <sensor_msgs/point_cloud2_iterator.h>
#include <std_srvs/Empty.h>
#include <visualization_msgs/MarkerArray.h>
#include <wavemap_3d/data_structure/volumetric_data_structure_3d_factory.h>
#include <wavemap_3d/data_structure/volumetric_octree.h>
#include <wavemap_3d/data_structure/wavelet_octree.h>
#include <wavemap_3d/integrator/pointcloud_integrator_3d_factory.h>
#include <wavemap_common/data_structure/pointcloud.h>
#include <wavemap_common/data_structure/volumetric/cell_types/occupancy_cell.h>
#include <wavemap_common/utils/nameof.h>
#include <wavemap_common_ros/utils/config_conversions.h>
#include <wavemap_common_ros/utils/visualization_utils.h>
#include <wavemap_msgs/FilePath.h>
#include <wavemap_msgs/Map.h>
#include <wavemap_msgs/MapEvaluationSummary.h>
#include <wavemap_msgs/PerformanceStats.h>

#include "wavemap_3d_ros/io/ros_msg_conversions.h"

namespace wavemap {
Wavemap3DServer::Wavemap3DServer(ros::NodeHandle nh, ros::NodeHandle nh_private,
                                 const Wavemap3DServer::Config& config)
    : config_(config.checkValid()) {
  // Setup data structure
  const param::Map data_structure_params =
      param::convert::toParamMap(nh_private, "map/data_structure");
  occupancy_map_ = VolumetricDataStructure3DFactory::create(
      data_structure_params, VolumetricDataStructure3DType::kHashedBlocks);
  CHECK_NOTNULL(occupancy_map_);

  // Setup integrator
  const param::Map integrator_params =
      param::convert::toParamMap(nh_private, "integrator");
  pointcloud_integrator_ = PointcloudIntegrator3DFactory::create(
      integrator_params, occupancy_map_,
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
    if (const auto& octree = std::dynamic_pointer_cast<
            VolumetricOctree<SaturatingOccupancyCell>>(occupancy_map_);
        octree) {
      wavemap_msgs::Map map_msg = mapToRosMsg(*octree);
      map_pub_.publish(map_msg);
    }
    if (const auto& wavelet_octree =
            std::dynamic_pointer_cast<WaveletOctree<SaturatingOccupancyCell>>(
                occupancy_map_);
        wavelet_octree) {
      wavemap_msgs::Map map_msg = mapToRosMsg(*wavelet_octree);
      map_pub_.publish(map_msg);
    }
  }
}

bool Wavemap3DServer::saveMap(const std::string& file_path) const {
  return !occupancy_map_->empty() &&
         occupancy_map_->save(file_path, kSaveWithFloatingPointPrecision);
}

bool Wavemap3DServer::loadMap(const std::string& file_path) {
  return occupancy_map_->load(file_path, kSaveWithFloatingPointPrecision);
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
    // Get the index of the x field, and assert that the y and z fields follow
    auto x_field_iter = std::find_if(
        scan_msg.fields.cbegin(), scan_msg.fields.cend(),
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
  map_pub_ = nh_private.advertise<wavemap_msgs::Map>("map", 10, true);
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
}

Wavemap3DServer::Config Wavemap3DServer::Config::fromRosParams(
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
  nh.param("map/" + NAMEOF(config.map_pruning_period_s),
           config.map_pruning_period_s, config.map_pruning_period_s);
  nh.param("map/" + NAMEOF(config.map_visualization_period_s),
           config.map_visualization_period_s,
           config.map_visualization_period_s);
  nh.param("map/" + NAMEOF(config.map_autosave_period_s),
           config.map_autosave_period_s, config.map_autosave_period_s);
  nh.param("map/" + NAMEOF(config.map_autosave_path), config.map_autosave_path,
           config.map_autosave_path);

  // Integrator
  nh.param("integrator/" + NAMEOF(config.pointcloud_topic_name),
           config.pointcloud_topic_name, config.pointcloud_topic_name);
  nh.param("integrator/" + NAMEOF(config.pointcloud_topic_queue_length),
           config.pointcloud_topic_queue_length,
           config.pointcloud_topic_queue_length);
  nh.param("integrator/" + NAMEOF(config.pointcloud_queue_max_wait_for_tf_s),
           config.pointcloud_queue_max_wait_for_tf_s,
           config.pointcloud_queue_max_wait_for_tf_s);

  return config;
}

bool Wavemap3DServer::Config::isValid(bool verbose) const {
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
