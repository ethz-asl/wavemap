#include "wavemap_ros/ros_server.h"

#include <std_srvs/Trigger.h>
#include <wavemap/core/map/map_factory.h>
#include <wavemap/io/file_conversions.h>
#include <wavemap_msgs/FilePath.h>
#include <wavemap_ros_conversions/config_conversions.h>

#include "wavemap_ros/inputs/input_factory.h"
#include "wavemap_ros/operations/operation_factory.h"

namespace wavemap {
DECLARE_CONFIG_MEMBERS(RosServerConfig,
                      (world_frame)
                      (num_threads)
                      (logging_level)
                      (allow_reset_map_service));

bool RosServerConfig::isValid(bool verbose) const {
  bool all_valid = true;

  all_valid &= IS_PARAM_NE(world_frame, std::string(""), verbose);
  all_valid &= IS_PARAM_GT(num_threads, 0, verbose);

  return all_valid;
}

// NOTE: If WavemapServerConfig::from(...) fails, accessing its value will throw
//       an exception and end the program.
RosServer::RosServer(ros::NodeHandle nh, ros::NodeHandle nh_private)
    : RosServer(nh, nh_private,
                RosServerConfig::from(
                    param::convert::toParamValue(nh_private, "general"))
                    .value()) {}

RosServer::RosServer(ros::NodeHandle nh, ros::NodeHandle nh_private,
                     const RosServerConfig& config)
    : config_(config.checkValid()),
      transformer_(std::make_shared<TfTransformer>()) {
  // Set the ROS logging level
  if (ros::console::set_logger_level(
          ROSCONSOLE_DEFAULT_NAME,
          LoggingLevel::ros_levels[config_.logging_level.toTypeId()])) {
    ros::console::notifyLoggerLevelsChanged();
  }

  // Setup data structure
  const auto data_structure_params =
      param::convert::toParamValue(nh_private, "map");
  occupancy_map_ =
      MapFactory::create(data_structure_params, MapType::kHashedBlocks);
  CHECK_NOTNULL(occupancy_map_);

  // Setup thread pool
  ROS_INFO_STREAM("Creating thread pool with " << config_.num_threads
                                               << " threads.");
  thread_pool_ = std::make_shared<ThreadPool>(config_.num_threads);
  CHECK_NOTNULL(thread_pool_);

  // Setup input handlers
  const param::Array integrator_params_array =
      param::convert::toParamArray(nh_private, "inputs");
  for (const auto& integrator_params : integrator_params_array) {
    addInput(integrator_params, nh, nh_private);
  }

  // Setup operation handlers
  const param::Array operation_params_array =
      param::convert::toParamArray(nh_private, "operations_pipeline");
  for (const auto& operation_params : operation_params_array) {
    addOperation(operation_params, nh_private);
  }

  // Connect to ROS
  advertiseServices(nh_private);
}

InputBase* RosServer::addInput(const param::Value& integrator_params,
                               const ros::NodeHandle& nh,
                               ros::NodeHandle nh_private) {
  auto input_handler = InputFactory::create(
      integrator_params, config_.world_frame, occupancy_map_, transformer_,
      thread_pool_, nh, nh_private, std::nullopt,
      [this](const ros::Time& current_time) { runOperations(current_time); });
  if (input_handler) {
    return input_handlers_.emplace_back(std::move(input_handler)).get();
  }
  return nullptr;
}

OperationBase* RosServer::addOperation(const param::Value& operation_params,
                                       ros::NodeHandle nh_private) {
  auto operation_handler = OperationFactory::create(
      operation_params, config_.world_frame, occupancy_map_, transformer_,
      thread_pool_, nh_private);
  if (operation_handler) {
    return operations_.emplace_back(std::move(operation_handler)).get();
  }
  return nullptr;
}

void RosServer::runOperations(const ros::Time& current_time,
                              bool force_run_all) {
  for (auto& operation : operations_) {
    operation->run(current_time, force_run_all);
  }
}

bool RosServer::saveMap(const std::filesystem::path& file_path) const {
  if (occupancy_map_) {
    occupancy_map_->threshold();
    return io::mapToFile(*occupancy_map_, file_path);
  } else {
    ROS_ERROR("Could not save map because it has not yet been allocated.");
  }
  return false;
}

bool RosServer::loadMap(const std::filesystem::path& file_path) {
  return io::fileToMap(file_path, occupancy_map_);
}

void RosServer::advertiseServices(ros::NodeHandle& nh_private) {
  reset_map_srv_ = nh_private.advertiseService<std_srvs::Trigger::Request,
                                               std_srvs::Trigger::Response>(
      "reset_map", [this](auto& /*request*/, auto& response) {
        response.success = false;
        if (config_.allow_reset_map_service) {
          if (occupancy_map_) {
            occupancy_map_->clear();
          }
          ROS_INFO("Map reset request was successfully executed.");
          response.success = true;
        } else {
          response.message =
              "Map resetting is forbidden. To change this, set ROS param \"" +
              NAMEOF(config_.allow_reset_map_service) + "\" to true.";
          ROS_INFO_STREAM("Received map reset request but ROS param \""
                          << NAMEOF(config_.allow_reset_map_service)
                          << "\" is set to false. Ignoring request.");
        }
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
