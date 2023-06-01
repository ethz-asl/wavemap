#include <gflags/gflags.h>

#include "wavemap_3d_ros/wavemap_3d_server.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "wavemap_3d_server");

  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, false);
  google::InstallFailureSignalHandler();

  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");
  wavemap::Wavemap3DServer wavemap_3d_server(nh, nh_private);

  ros::spin();
  return 0;
}
