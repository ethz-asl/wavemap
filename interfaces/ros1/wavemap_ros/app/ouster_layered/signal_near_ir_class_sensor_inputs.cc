#include "signal_map_layers.h"

#include <glog/logging.h>

#include <wavemap_ros/layered_server_builder.h>
#include <wavemap_ros_conversions/descriptor_layered_map_conversions.h>

namespace wavemap::examples::signal_map::near_ir_class_map {

using Definition = NearIrClassDefinition;
using RosConverter =
    convert::DescriptorContinuousRosConverter<Definition::Voxel>;
using ServerBuilder = LayeredRosServerBuilder<Definition, RosConverter>;

void registerSensorInputs(RosServer& server,
                          ServerBuilder::Extension& extension) {
  const layered::EndpointRange endpoint_range{0.5f, 25.f};
  server.bindNormalizedPointField<ReflectivityLayer>(
      extension, "reflectivity", 0.f, 255.f, endpoint_range);
  server.bindNormalizedPointField<SignalLayer>(
      extension, "intensity", 0.f, 65535.f, endpoint_range);
  server.bindNormalizedPointField<NearIrLayer>(
      extension, "ambient", 0.f, 65535.f, endpoint_range);

  layered::LocalElevationClassifierConfig class_config;
  class_config.grid_cell_width = 0.5f;
  class_config.ground_tolerance = 0.2f;
  class_config.obstacle_height = 0.35f;
  class_config.min_points_per_cell = 3u;
  server.bindGeometricClass<ClassLayer>(
      extension, class_config, endpoint_range, -kHalfPi, kHalfPi);
}

}  // namespace wavemap::examples::signal_map::near_ir_class_map

int main(int argc, char** argv) {
  ros::init(argc, argv, "wavemap_ros_server");
  google::InitGoogleLogging(argv[0]);
  google::InstallFailureSignalHandler();
  FLAGS_alsologtostderr = true;
  FLAGS_colorlogtostderr = true;

  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");
  wavemap::RosServer server(
      nh, nh_private,
      [](wavemap::RosServer& server, ros::NodeHandle private_nh) {
        namespace map = wavemap::examples::signal_map::near_ir_class_map;
        int discrete_block_height = 0;
        private_nh.param("discrete_block_height", discrete_block_height, 0);
        map::ServerBuilder::install(
            server, private_nh, server.worldFrame(),
            wavemap::examples::signal_map::makeMapConfigFor<map::Definition>(
                discrete_block_height),
            map::registerSensorInputs);
        ROS_INFO(
            "Installed occupancy+reflectivity+signal+near_ir+class benchmark "
            "application (H) with discrete block height %d.",
            discrete_block_height);
      });

  ros::spin();
  return 0;
}
