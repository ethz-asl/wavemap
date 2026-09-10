#include "occupancy_map_layers.h"

#include <glog/logging.h>

#include <wavemap_ros/layered_server_builder.h>
#include <wavemap_ros_conversions/descriptor_layered_map_conversions.h>

namespace wavemap::examples::occupancy_map {

using RosConverter =
    convert::DescriptorContinuousRosConverter<Definition::Voxel>;
using ServerBuilder = LayeredRosServerBuilder<Definition, RosConverter>;

void registerSensorInputs(RosServer& /*server*/,
                          ServerBuilder::Extension& /*extension*/) {
  // Occupancy continues to use Wavemap's original point-cloud integration.
}

}  // namespace wavemap::examples::occupancy_map

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
        namespace map = wavemap::examples::occupancy_map;
        map::ServerBuilder::install(server, private_nh, server.worldFrame(),
                                    map::makeMapConfig(),
                                    map::registerSensorInputs);
        ROS_INFO("Installed layered occupancy-only benchmark application (B).");
      });

  ros::spin();
  return 0;
}
