#include "signal_map_layers.h"

#include <glog/logging.h>

#include <wavemap_ros/layered_server_builder.h>
#include <wavemap_ros_conversions/descriptor_layered_map_conversions.h>

namespace wavemap::examples::signal_map::near_ir_map {

using Definition = NearIrDefinition;
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
}

}  // namespace wavemap::examples::signal_map::near_ir_map

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
        namespace map = wavemap::examples::signal_map::near_ir_map;
        map::ServerBuilder::install(
            server, private_nh, server.worldFrame(),
            wavemap::examples::signal_map::makeMapConfigFor<map::Definition>(),
            map::registerSensorInputs);
        ROS_INFO("Installed occupancy+reflectivity+signal+near_ir benchmark application (F).");
      });

  ros::spin();
  return 0;
}
