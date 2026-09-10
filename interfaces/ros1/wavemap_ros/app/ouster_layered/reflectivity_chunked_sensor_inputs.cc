#include "reflectivity_only_map_layers.h"

#include <glog/logging.h>

#include <wavemap_ros/layered_server_builder.h>
#include <wavemap_ros_conversions/descriptor_layered_map_conversions.h>

namespace wavemap::examples::reflectivity_only_map {
namespace chunked {

using Definition = ChunkedDefinition;
using RosConverter =
    convert::DescriptorContinuousRosConverter<Definition::Voxel>;
using ServerBuilder = LayeredRosServerBuilder<Definition, RosConverter>;

void registerSensorInputs(RosServer& server,
                          ServerBuilder::Extension& extension) {
  server.bindNormalizedPointField<ReflectivityLayer>(
      extension, "reflectivity", 0.f, 255.f,
      layered::EndpointRange{0.5f, 25.f});
}

}  // namespace chunked
}  // namespace wavemap::examples::reflectivity_only_map

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
        namespace map =
            wavemap::examples::reflectivity_only_map::chunked;
        map::ServerBuilder::install(
            server, private_nh, server.worldFrame(),
            wavemap::examples::reflectivity_only_map::
                makeMapConfig<map::Definition>(),
            map::registerSensorInputs);
        ROS_INFO(
            "Installed chunked occupancy+reflectivity application.");
      });

  ros::spin();
  return 0;
}
