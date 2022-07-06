#include <gflags/gflags.h>

#include "wavemap_2d_ros/wavemap_2d_server.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "wavemap");

  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, false);
  google::InstallFailureSignalHandler();

  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");
  wavemap::Wavemap2DServer wavemap_2d_server(nh, nh_private);

  ros::spin();
  return 0;
}
