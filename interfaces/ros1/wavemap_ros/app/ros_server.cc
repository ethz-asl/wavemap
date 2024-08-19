#include "wavemap_ros/ros_server.h"

#include <glog/logging.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "wavemap_ros_server");

  google::InitGoogleLogging(argv[0]);
  google::InstallFailureSignalHandler();
  FLAGS_alsologtostderr = true;
  FLAGS_colorlogtostderr = true;

  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");
  wavemap::RosServer wavemap_server(nh, nh_private);

  ros::spin();
  return 0;
}
