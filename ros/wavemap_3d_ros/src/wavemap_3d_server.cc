#include "wavemap_3d_ros/wavemap_3d_server.h"

#include <std_srvs/Empty.h>
#include <wavemap_3d/data_structure/volumetric_data_structure_3d_factory.h>
#include <wavemap_3d/data_structure/volumetric_octree.h>
#include <wavemap_3d/data_structure/wavelet_octree.h>
#include <wavemap_common/data_structure/volumetric/cell_types/occupancy_cell.h>
#include <wavemap_common/utils/nameof.h>
#include <wavemap_common_ros/utils/config_conversions.h>
#include <wavemap_common_ros/utils/visualization_utils.h>
#include <wavemap_msgs/FilePath.h>
#include <wavemap_msgs/Map.h>
#include <wavemap_msgs/MapEvaluationSummary.h>
#include <wavemap_msgs/PerformanceStats.h>

#include "wavemap_3d_ros/input_handler/depth_image_input_handler.h"
#include "wavemap_3d_ros/input_handler/pointcloud_input_handler.h"
#include "wavemap_3d_ros/io/ros_msg_conversions.h"

namespace wavemap {
Wavemap3DServer::Wavemap3DServer(ros::NodeHandle nh, ros::NodeHandle nh_private)
    : Wavemap3DServer(
          nh, nh_private,
          Config::from(param::convert::toParamMap(nh_private, ""))) {}

Wavemap3DServer::Wavemap3DServer(ros::NodeHandle nh, ros::NodeHandle nh_private,
                                 const Wavemap3DServer::Config& config)
    : config_(config.checkValid()) {
  // Setup data structure
  const param::Map data_structure_params =
      param::convert::toParamMap(nh_private, "map/data_structure");
  occupancy_map_ = VolumetricDataStructure3DFactory::create(
      data_structure_params, VolumetricDataStructure3DType::kHashedBlocks);
  CHECK_NOTNULL(occupancy_map_);

  // Setup input handlers
  const param::Array integrator_params_array =
      param::convert::toParamArray(nh_private, "integrators");
  for (const auto& integrator_params : integrator_params_array) {
    if (integrator_params.holds<param::Map>()) {
      const auto param_map = integrator_params.get<param::Map>();
      addInput(param_map, nh);
    }
  }

  // Connect to ROS
  subscribeToTimers(nh);
  subscribeToTopics(nh);
  advertiseTopics(nh_private);
  advertiseServices(nh_private);
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

InputHandler* Wavemap3DServer::addInput(const param::Map& integrator_params,
                                        const ros::NodeHandle& nh) {
  std::string error_msg;
  const auto input_type =
      InputHandlerType::fromParamMap(integrator_params, error_msg);
  if (!input_type.isValid()) {
    LOG(WARNING) << error_msg;
    return nullptr;
  }

  switch (input_type.toTypeId()) {
    case InputHandlerType::kPointcloud:
      return input_handlers_
          .emplace_back(std::make_unique<PointcloudInputHandler>(
              integrator_params, config_.general.world_frame, occupancy_map_,
              transformer_, nh))
          .get();
    case InputHandlerType::kDepthImage:
      return input_handlers_
          .emplace_back(std::make_unique<DepthImageInputHandler>(
              integrator_params, config_.general.world_frame, occupancy_map_,
              transformer_, nh))
          .get();
  }
  return nullptr;
}

void Wavemap3DServer::subscribeToTimers(const ros::NodeHandle& nh) {
  if (0.f < config_.map.pruning_period) {
    ROS_INFO_STREAM("Registering map pruning timer with period "
                    << config_.map.pruning_period << "s");
    map_pruning_timer_ = nh.createTimer(
        ros::Duration(config_.map.pruning_period),
        [this](const auto& /*event*/) { occupancy_map_->prune(); });
  }

  if (0.f < config_.map.visualization_period) {
    ROS_INFO_STREAM("Registering map visualization timer with period "
                    << config_.map.visualization_period << "s");
    map_visualization_timer_ =
        nh.createTimer(ros::Duration(config_.map.visualization_period),
                       [this](const auto& /*event*/) { visualizeMap(); });
  }

  if (0.f < config_.map.autosave_period && !config_.map.autosave_path.empty()) {
    ROS_INFO_STREAM("Registering autosave timer with period "
                    << config_.map.autosave_period << "s");
    map_autosave_timer_ = nh.createTimer(
        ros::Duration(config_.map.autosave_period),
        [this](const auto& /*event*/) { saveMap(config_.map.autosave_path); });
  }
}

void Wavemap3DServer::subscribeToTopics(ros::NodeHandle& /*nh*/) {}

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

Wavemap3DServer::Config Wavemap3DServer::Config::from(
    const param::Map& params) {
  Config config;

  // General
  if (param::map::keyHoldsValue<param::Map>(params, NAMEOF(config.general))) {
    const auto params_general =
        param::map::keyGetValue<param::Map>(params, NAMEOF(config.general));

    config.general.world_frame = param::map::keyGetValue(
        params_general, NAMEOF(config.general.world_frame),
        config.general.world_frame);

    config.general.publish_performance_stats = param::map::keyGetValue(
        params_general, NAMEOF(config.general.publish_performance_stats),
        config.general.publish_performance_stats);
  }

  // Map
  if (param::map::keyHoldsValue<param::Map>(params, NAMEOF(config.map))) {
    const auto params_map =
        param::map::keyGetValue<param::Map>(params, NAMEOF(config.map));

    config.map.pruning_period =
        param::convert::toSeconds(params_map, NAMEOF(config.map.pruning_period),
                                  config.map.pruning_period);

    config.map.visualization_period = param::convert::toSeconds(
        params_map, NAMEOF(config.map.visualization_period),
        config.map.visualization_period);

    config.map.autosave_period = param::convert::toSeconds(
        params_map, NAMEOF(config.map.autosave_period),
        config.map.autosave_period);

    config.map.autosave_path = param::map::keyGetValue(
        params_map, NAMEOF(config.map.autosave_path), config.map.autosave_path);
  }

  return config;
}

bool Wavemap3DServer::Config::isValid(bool verbose) const {
  bool all_valid = true;

  all_valid &= IS_PARAM_NE(general.world_frame, std::string(""), verbose);

  return all_valid;
}
}  // namespace wavemap
