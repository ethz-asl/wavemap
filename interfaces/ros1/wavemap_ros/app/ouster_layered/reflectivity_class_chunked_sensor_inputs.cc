#include "reflectivity_map_layers.h"

#include <glog/logging.h>

#include <wavemap/layered/classification/local_elevation_classifier.h>
#include <wavemap_ros/layered_server_builder.h>
#include <wavemap_ros_conversions/descriptor_layered_map_conversions.h>

namespace wavemap::examples::reflectivity_map::chunked {

using Definition = ChunkedDefinition;
using RosConverter =
    convert::DescriptorContinuousRosConverter<Definition::Voxel>;
using ServerBuilder = LayeredRosServerBuilder<Definition, RosConverter>;

void registerSensorInputs(RosServer& server,
                          ServerBuilder::Extension& extension) {
  server.bindNormalizedPointField<ReflectivityLayer>(
      extension, "reflectivity", 0.f, 255.f,
      layered::EndpointRange{0.5f, 25.f});

  layered::LocalElevationClassifierConfig class_config;
  class_config.grid_cell_width = 0.5f;
  class_config.ground_tolerance = 0.2f;
  class_config.obstacle_height = 0.35f;
  class_config.min_points_per_cell = 3u;
  server.bindGeometricClass<ClassLayer>(
      extension, class_config, layered::EndpointRange{0.5f, 25.f});
}

}  // namespace wavemap::examples::reflectivity_map::chunked

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
        namespace map = wavemap::examples::reflectivity_map::chunked;
        map::ServerBuilder::install(
            server, private_nh, server.worldFrame(),
            wavemap::examples::reflectivity_map::
                makeMapConfig<map::Definition>(),
            map::registerSensorInputs);
        ROS_INFO(
            "Installed chunked occupancy+reflectivity+class benchmark "
            "application.");
      });

  ros::spin();
  return 0;
}
