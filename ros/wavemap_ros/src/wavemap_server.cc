#include "wavemap_ros/wavemap_server.h"

#include <std_srvs/Empty.h>
#include <wavemap/data_structure/volumetric/volumetric_data_structure_factory.h>
#include <wavemap/utils/nameof.h>
#include <wavemap_file_conversions/file_conversions.h>
#include <wavemap_msgs/FilePath.h>
#include <wavemap_msgs/Map.h>
#include <wavemap_msgs/MapEvaluationSummary.h>
#include <wavemap_ros_conversions/config_conversions.h>
#include <wavemap_ros_conversions/map_msg_conversions.h>

#include "wavemap_ros/input_handler/input_handler_factory.h"

namespace wavemap {
DECLARE_CONFIG_MEMBERS(WavemapServerConfig,
                      (world_frame)
                      (thresholding_period, SiUnit::kSeconds)
                      (pruning_period, SiUnit::kSeconds)
                      (visualization_period, SiUnit::kSeconds));

bool WavemapServerConfig::isValid(bool verbose) const {
  bool all_valid = true;

  all_valid &= IS_PARAM_NE(world_frame, std::string(""), verbose);

  return all_valid;
}

WavemapServer::WavemapServer(ros::NodeHandle nh, ros::NodeHandle nh_private)
    : WavemapServer(nh, nh_private,
                    WavemapServerConfig::from(param::convert::toParamMap(
                        nh_private, "map/general"))) {}

WavemapServer::WavemapServer(ros::NodeHandle nh, ros::NodeHandle nh_private,
                             const WavemapServerConfig& config)
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
      param::convert::toParamArray(nh_private, "inputs");
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
    occupancy_map_->threshold();

    const wavemap_msgs::Map map_msg =
        convert::mapToRosMsg(occupancy_map_, config_.world_frame,
                             ros::Time::now(), config_.visualization_period);
    map_pub_.publish(map_msg);
  }
}

bool WavemapServer::saveMap(const std::string& file_path) const {
  if (occupancy_map_) {
    return convert::mapToFile(*occupancy_map_, file_path);
  } else {
    LOG(ERROR) << "Could not save map because it has not yet been allocated.";
  }
}

bool WavemapServer::loadMap(const std::string& file_path) {
  return convert::fileToMap(file_path, occupancy_map_);
}

InputHandler* WavemapServer::addInput(const param::Map& integrator_params,
                                      const ros::NodeHandle& nh,
                                      ros::NodeHandle nh_private) {
  auto input_handler =
      InputHandlerFactory::create(integrator_params, config_.world_frame,
                                  occupancy_map_, transformer_, nh, nh_private);
  if (input_handler) {
    return input_handlers_.emplace_back(std::move(input_handler)).get();
  }
  return nullptr;
}

void WavemapServer::subscribeToTimers(const ros::NodeHandle& nh) {
  if (0.f < config_.thresholding_period) {
    ROS_INFO_STREAM("Registering map thresholding timer with period "
                    << config_.thresholding_period << "s");
    map_thresholding_timer_ = nh.createTimer(
        ros::Duration(config_.thresholding_period),
        [this](const auto& /*event*/) { occupancy_map_->threshold(); });
  }

  if (0.f < config_.pruning_period) {
    ROS_INFO_STREAM("Registering map pruning timer with period "
                    << config_.pruning_period << "s");
    map_pruning_timer_ = nh.createTimer(
        ros::Duration(config_.pruning_period),
        [this](const auto& /*event*/) { occupancy_map_->prune(); });
  }

  if (0.f < config_.visualization_period) {
    ROS_INFO_STREAM("Registering map visualization timer with period "
                    << config_.visualization_period << "s");
    map_visualization_timer_ =
        nh.createTimer(ros::Duration(config_.visualization_period),
                       [this](const auto& /*event*/) { visualizeMap(); });
  }
}

void WavemapServer::subscribeToTopics(ros::NodeHandle& /*nh*/) {}

void WavemapServer::advertiseTopics(ros::NodeHandle& nh_private) {
  map_pub_ = nh_private.advertise<wavemap_msgs::Map>("map", 10, true);
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
}  // namespace wavemap
