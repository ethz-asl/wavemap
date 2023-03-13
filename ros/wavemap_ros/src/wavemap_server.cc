#include "wavemap_ros/wavemap_server.h"

#include <std_srvs/Empty.h>
#include <wavemap/data_structure/volumetric/volumetric_data_structure_factory.h>
#include <wavemap/data_structure/volumetric/volumetric_octree.h>
#include <wavemap/data_structure/volumetric/wavelet_octree.h>
#include <wavemap/utils/nameof.h>
#include <wavemap_msgs/FilePath.h>
#include <wavemap_msgs/Map.h>
#include <wavemap_msgs/MapEvaluationSummary.h>
#include <wavemap_msgs/PerformanceStats.h>

#include "wavemap_ros/input_handler/input_handler_factory.h"
#include "wavemap_ros/io/ros_msg_conversions.h"
#include "wavemap_ros/utils/config_conversions.h"
#include "wavemap_ros/utils/visualization_utils.h"

namespace wavemap {
WavemapServer::WavemapServer(ros::NodeHandle nh, ros::NodeHandle nh_private)
    : WavemapServer(nh, nh_private,
                    Config::from(param::convert::toParamMap(nh_private, ""))) {}

WavemapServer::WavemapServer(ros::NodeHandle nh, ros::NodeHandle nh_private,
                             const WavemapServer::Config& config)
    : config_(config.checkValid()),
      transformer_(std::make_shared<TfTransformer>()) {
  // Setup data structure
  const param::Map data_structure_params =
      param::convert::toParamMap(nh_private, "map/data_structure");
  occupancy_map_ = VolumetricDataStructureFactory::create(
      data_structure_params, VolumetricDataStructureType::kHashedBlocks);
  CHECK_NOTNULL(occupancy_map_);

  // Setup input handlers
  const param::Array integrator_params_array =
      param::convert::toParamArray(nh_private, "integrators");
  for (const auto& integrator_params : integrator_params_array) {
    if (integrator_params.holds<param::Map>()) {
      const auto param_map = integrator_params.get<param::Map>();
      addInput(param_map, nh, nh_private);
    }
  }

  // Connect to ROS
  subscribeToTimers(nh);
  subscribeToTopics(nh);
  advertiseTopics(nh_private);
  advertiseServices(nh_private);
}

void WavemapServer::visualizeMap() {
  if (occupancy_map_ && !occupancy_map_->empty()) {
    if (const auto octree =
            std::dynamic_pointer_cast<VolumetricOctree>(occupancy_map_);
        octree) {
      wavemap_msgs::Map map_msg =
          mapToRosMsg(*octree, config_.general.world_frame);
      map_pub_.publish(map_msg);
    }
    if (const auto wavelet_octree =
            std::dynamic_pointer_cast<WaveletOctree>(occupancy_map_);
        wavelet_octree) {
      wavemap_msgs::Map map_msg =
          mapToRosMsg(*wavelet_octree, config_.general.world_frame);
      map_pub_.publish(map_msg);
    }
    if (const auto hashed_wavelet_octree =
            std::dynamic_pointer_cast<HashedWaveletOctree>(occupancy_map_);
        hashed_wavelet_octree) {
      wavemap_msgs::Map map_msg =
          mapToRosMsg(*hashed_wavelet_octree, config_.general.world_frame);
      map_pub_.publish(map_msg);
    }
  }
}

bool WavemapServer::saveMap(const std::string& file_path) const {
  LOG(ERROR) << "Could not save map to " << file_path
             << ". Map saving not yet implemented.";
  return false;
}

bool WavemapServer::loadMap(const std::string& file_path) {
  LOG(ERROR) << "Could not load map from " << file_path
             << ". Map loading not yet implemented.";
  return false;
}

InputHandler* WavemapServer::addInput(const param::Map& integrator_params,
                                      const ros::NodeHandle& nh,
                                      ros::NodeHandle nh_private) {
  auto input_handler = InputHandlerFactory::create(
      integrator_params, config_.general.world_frame, occupancy_map_,
      transformer_, nh, std::move(nh_private));
  if (input_handler) {
    return input_handlers_.emplace_back(std::move(input_handler)).get();
  }
  return nullptr;
}

void WavemapServer::subscribeToTimers(const ros::NodeHandle& nh) {
  if (0.f < config_.map.thresholding_period) {
    ROS_INFO_STREAM("Registering map thresholding timer with period "
                    << config_.map.thresholding_period << "s");
    map_thresholding_timer_ = nh.createTimer(
        ros::Duration(config_.map.thresholding_period),
        [this](const auto& /*event*/) { occupancy_map_->threshold(); });
  }

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

void WavemapServer::subscribeToTopics(ros::NodeHandle& /*nh*/) {}

void WavemapServer::advertiseTopics(ros::NodeHandle& nh_private) {
  map_pub_ = nh_private.advertise<wavemap_msgs::Map>("map", 10, true);
  performance_stats_pub_ = nh_private.advertise<wavemap_msgs::PerformanceStats>(
      "performance_stats", 10, true);
}

void WavemapServer::advertiseServices(ros::NodeHandle& nh_private) {
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

WavemapServer::Config WavemapServer::Config::from(const param::Map& params) {
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

    config.map.thresholding_period = param::convert::toUnit<SiUnit::kSeconds>(
        params_map, NAMEOF(config.map.thresholding_period),
        config.map.thresholding_period);

    config.map.pruning_period = param::convert::toUnit<SiUnit::kSeconds>(
        params_map, NAMEOF(config.map.pruning_period),
        config.map.pruning_period);

    config.map.visualization_period = param::convert::toUnit<SiUnit::kSeconds>(
        params_map, NAMEOF(config.map.visualization_period),
        config.map.visualization_period);

    config.map.autosave_period = param::convert::toUnit<SiUnit::kSeconds>(
        params_map, NAMEOF(config.map.autosave_period),
        config.map.autosave_period);

    config.map.autosave_path = param::map::keyGetValue(
        params_map, NAMEOF(config.map.autosave_path), config.map.autosave_path);
  }

  return config;
}

bool WavemapServer::Config::isValid(bool verbose) const {
  bool all_valid = true;

  all_valid &= IS_PARAM_NE(general.world_frame, std::string(""), verbose);

  return all_valid;
}
}  // namespace wavemap
