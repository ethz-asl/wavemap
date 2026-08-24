#ifndef WAVEMAP_ROS_LAYERED_SERVER_BUILDER_H_
#define WAVEMAP_ROS_LAYERED_SERVER_BUILDER_H_

#include <memory>
#include <string>
#include <utility>

#include <ros/node_handle.h>

#include "wavemap_ros/layered_hashed_wavelet_integrator_factory.h"
#include "wavemap_ros/layered_ros_server_extension.h"
#include "wavemap_ros/map_operations/layered_publish_map_operation.h"
#include "wavemap_ros/ros_server.h"

namespace wavemap {

// Installs one compile-time LayeredMap definition into the existing ROS server
// lifecycle. The original occupancy-only construction remains untouched.
// SensorBindingsT receives (RosServer&, TypedExtension&) and only registers
// how sensor data populates the schema's layers.
template <typename MapDefinitionT, typename RosConverterT>
class LayeredRosServerBuilder {
 public:
  using Definition = MapDefinitionT;
  using Map = typename Definition::Map;
  using Config = typename Definition::Config;
  using MapIo = typename Definition::MapIo;
  using Voxel = typename Definition::Voxel;
  using ObservationBatch =
      layered::LayerObservationBatch<typename Definition::Schema>;
  using Extension = LayeredRosServerExtension<Map, MapIo, RosConverterT>;

  template <typename SensorBindingsT>
  static Extension& install(RosServer& server, ros::NodeHandle nh_private,
                            std::string world_frame, Config config,
                            SensorBindingsT&& sensor_bindings) {
    auto extension = std::make_unique<Extension>(
        nh_private, world_frame, Map(config));
    auto* const extension_ptr = extension.get();

    std::forward<SensorBindingsT>(sensor_bindings)(server, *extension_ptr);

    server.layered_extension_ = std::move(extension);
    server.occupancy_map_ = server.layered_extension_->getContinuousMap();

    auto continuous_map = extension_ptr->getLayeredMap().continuousMapPtr();
    server.layered_integrator_factory_ =
        [continuous_map](const param::Value& params,
                         std::shared_ptr<ThreadPool> thread_pool) {
          return createLayeredHashedWaveletIntegrator<Voxel>(
              params, continuous_map, std::move(thread_pool));
        };

    server.layered_publish_operation_factory_ =
        [&server, extension_ptr, world_frame = std::move(world_frame)](
            const PublishMapOperationConfig& publish_config,
            ros::NodeHandle operation_nh_private) {
          using Operation =
              UnifiedLayeredPublishMapOperation<Map, RosConverterT>;
          return std::make_unique<Operation>(
              publish_config, &extension_ptr->getLayeredMap(),
              server.thread_pool_, world_frame,
              std::move(operation_nh_private));
        };

    return *extension_ptr;
  }
};

}  // namespace wavemap

#endif  // WAVEMAP_ROS_LAYERED_SERVER_BUILDER_H_
