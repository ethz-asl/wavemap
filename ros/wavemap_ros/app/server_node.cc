#include <gflags/gflags.h>

#include "wavemap_ros/wavemap_server.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "wavemap_server");

  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, false);
  google::InstallFailureSignalHandler();
  FLAGS_alsologtostderr = true;
  FLAGS_colorlogtostderr = true;

  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");
  wavemap::WavemapServer wavemap_server(nh, nh_private);

  ros::spin();
  return 0;
}
