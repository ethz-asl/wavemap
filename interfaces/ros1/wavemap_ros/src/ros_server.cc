#include "wavemap_ros/ros_server.h"

#include <memory>
#include <utility>

#include <std_srvs/Trigger.h>
#include <wavemap/core/map/map_factory.h>
#include <wavemap/io/map/file_conversions.h>
#include <wavemap/pipeline/map_operations/map_operation_factory.h>
#include <wavemap_msgs/FilePath.h>
#include <wavemap_ros_conversions/config_conversions.h>

#include "wavemap_ros/inputs/ros_input_factory.h"
#include "wavemap_ros/map_operations/map_ros_operation_factory.h"

namespace wavemap {
DECLARE_CONFIG_MEMBERS(RosServerConfig,
                      (world_frame)
                      (num_threads)
                      (logging_level)
                      (allow_reset_map_service));

bool RosServerConfig::isValid(bool verbose) const {
  bool all_valid = true;

  all_valid &= IS_PARAM_NE(world_frame, "", verbose);
  all_valid &= IS_PARAM_GT(num_threads, 0, verbose);
  all_valid &= IS_PARAM_TRUE(logging_level.isValid(), verbose);

  return all_valid;
}

// NOTE: If RosServerConfig::from(...) fails, accessing its value will throw
//       an exception and end the program.
RosServer::RosServer(ros::NodeHandle nh, ros::NodeHandle nh_private)
    : RosServer(
          nh, nh_private,
          RosServerConfig::from(convert::rosToParams(nh_private, "general"))
              .value()) {}

RosServer::RosServer(ros::NodeHandle nh, ros::NodeHandle nh_private,
                     const RosServerConfig& config)
    : config_(config.checkValid()),
      transformer_(std::make_shared<TfTransformer>()) {
  // Set the logging level for wavemap's C++ library (uses glog) and ROS
  config_.logging_level.applyToGlog();
  config_.logging_level.applyToRosConsole();

  // Setup data structure
  const auto data_structure_params = convert::rosToParams(nh_private, "map");
  occupancy_map_ =
      MapFactory::create(data_structure_params, MapType::kHashedBlocks);
  CHECK_NOTNULL(occupancy_map_);

  // Setup thread pool
  ROS_INFO_STREAM("Creating thread pool with " << config_.num_threads
                                               << " threads.");
  thread_pool_ = std::make_shared<ThreadPool>(config_.num_threads);
  CHECK_NOTNULL(thread_pool_);

  // Setup the pipeline
  pipeline_ = std::make_shared<Pipeline>(occupancy_map_, thread_pool_);
  CHECK_NOTNULL(pipeline_);

  // Add map operations to pipeline
  const param::Array map_operation_param_array =
      convert::rosToParamArray(nh_private, "map_operations");
  for (const auto& operation_params : map_operation_param_array) {
    addOperation(operation_params, nh_private);
  }

  // Add measurement integrators to pipeline
  const param::Map measurement_integrator_param_map =
      convert::rosToParamMap(nh_private, "measurement_integrators");
  for (const auto& [integrator_name, integrator_params] :
       measurement_integrator_param_map) {
    pipeline_->addIntegrator(integrator_name, integrator_params);
  }

  // Setup measurement inputs
  const param::Array input_param_array =
      convert::rosToParamArray(nh_private, "inputs");
  for (const auto& integrator_params : input_param_array) {
    addInput(integrator_params, nh, nh_private);
  }

  // Connect to ROS
  advertiseServices(nh_private);
}

void RosServer::clear() {
  clearInputs();
  if (pipeline_) {
    pipeline_->clear();
  }
  if (occupancy_map_) {
    occupancy_map_->clear();
  }
}

RosInputBase* RosServer::addInput(const param::Value& integrator_params,
                                  const ros::NodeHandle& nh,
                                  ros::NodeHandle nh_private) {
  auto input =
      RosInputFactory::create(integrator_params, pipeline_, transformer_,
                              config_.world_frame, nh, nh_private);
  return addInput(std::move(input));
}

RosInputBase* RosServer::addInput(std::unique_ptr<RosInputBase> input) {
  if (input) {
    return inputs_.emplace_back(std::move(input)).get();
  }

  ROS_WARN("Ignoring request to add input. Input is null pointer.");
  return nullptr;
}

MapOperationBase* RosServer::addOperation(const param::Value& operation_params,
                                          ros::NodeHandle nh_private) {
  // Read the operation type name from params
  const auto type_name = param::getTypeStr(operation_params);
  if (!type_name) {
    // No type name was defined
    // NOTE: A message explaining the failure is already printed by getTypeStr.
    ROS_WARN_STREAM(
        "Could not add operation. No operation type specified. "
        "Please set it by adding a param with key \""
        << param::kTypeSelectorKey << "\".");
    return nullptr;
  }

  if (const auto type = MapRosOperationType{type_name.value()};
      type.isValid()) {
    auto operation = MapRosOperationFactory::create(
        type, operation_params, occupancy_map_, thread_pool_, transformer_,
        config_.world_frame, nh_private);
    return pipeline_->addOperation(std::move(operation));
  }

  if (const auto type = MapOperationType{type_name.value()}; type.isValid()) {
    auto operation =
        MapOperationFactory::create(type, operation_params, occupancy_map_);
    return pipeline_->addOperation(std::move(operation));
  }

  LOG(WARNING) << "Value of type name param \"" << param::kTypeSelectorKey
               << "\": \"" << type_name.value()
               << "\" does not match a known operation type name. Supported "
                  "type names are ["
               << print::sequence(MapRosOperationType::names) << ", "
               << print::sequence(MapOperationType::names) << "].";
  return nullptr;
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
