#include "reflectivity_map_layers.h"

#include <glog/logging.h>

#include <wavemap_ros/layered_server_builder.h>
#include <wavemap_ros_conversions/descriptor_layered_map_conversions.h>

namespace wavemap::examples::reflectivity_map {  // Ouster mapping example

using RosConverter =
    convert::DescriptorContinuousRosConverter<Definition::Voxel>;
using ServerBuilder = LayeredRosServerBuilder<Definition, RosConverter>;

void registerSensorInputs(RosServer& server,
                          ServerBuilder::Extension& extension) {
  // The Ouster PointCloud2 stores reflectivity as a raw numeric field in
  // [0, 255]. Decode it during the existing XYZ traversal, normalize it into
  // the map's [0, 1] storage domain, and update occupied endpoints only.
  server.bindNormalizedPointField<ReflectivityLayer>(
      extension, "reflectivity", 0.f, 255.f,
      layered::EndpointRange{0.5f, 25.f});

  // This bag has no class field. Derive a small example taxonomy directly
  // from each posed cloud: 1 = local ground and 2 = obstacle.
  layered::LocalElevationClassifierConfig class_config;
  class_config.grid_cell_width = 0.5f;
  class_config.ground_tolerance = 0.2f;
  class_config.obstacle_height = 0.35f;
  class_config.min_points_per_cell = 3u;
  server.bindGeometricClass<ClassLayer>(
      extension, class_config, layered::EndpointRange{0.5f, 25.f});
}

}  // namespace wavemap::examples::reflectivity_map

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
        namespace map = wavemap::examples::reflectivity_map;
        int discrete_block_height = 0;
        private_nh.param("discrete_block_height", discrete_block_height, 0);
        map::ServerBuilder::install(server, private_nh, server.worldFrame(),
                                    map::makeMapConfig(discrete_block_height),
                                    map::registerSensorInputs);
        ROS_INFO(
            "Installed occupancy+reflectivity+class benchmark application "
            "(D) with discrete block height %d.", discrete_block_height);
      });

  ros::spin();
  return 0;
}
