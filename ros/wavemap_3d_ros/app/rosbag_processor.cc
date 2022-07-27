#include <rosgraph_msgs/Clock.h>
#include <tf/tfMessage.h>
#include <wavemap_common_ros/rosbag_processor.h>

#include "wavemap_3d_ros/wavemap_3d_server.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "wavemap_3d_rosbag_processor");

  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, false);
  google::InstallFailureSignalHandler();

  // Setup the wavemap server node
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");
  wavemap::Wavemap3DServer wavemap_server(nh, nh_private);

  // Read the required ROS params
  std::string pointcloud_topic;
  nh_private.param<std::string>("pointcloud_topic", pointcloud_topic,
                                "/pointcloud");
  std::string rosbag_paths_str;
  nh_private.param<std::string>("rosbag_path", rosbag_paths_str, "");

  // Create the rosbag processor and load the rosbags
  wavemap::RosbagProcessor rosbag_processor;
  std::istringstream rosbag_paths_ss(rosbag_paths_str);
  std::string rosbag_path;
  while (rosbag_paths_ss >> rosbag_path) {
    rosbag_processor.addRosbag(rosbag_path);
  }

  // Setup the callbacks and topics to republish
  rosbag_processor.addCallback(pointcloud_topic,
                               &wavemap::Wavemap3DServer::pointcloudCallback,
                               &wavemap_server);
  rosbag_processor.addRepublisher<tf::tfMessage>("/tf", "/tf", nh, 10);
  rosbag_processor.addRepublisher<tf::tfMessage>("/tf_static", "/tf_static", nh,
                                                 10);
  if (rosbag_processor.bagsContainTopic("/clock")) {
    rosbag_processor.addRepublisher<rosgraph_msgs::Clock>("/clock", "/clock",
                                                          nh, 1);
  } else {
    rosbag_processor.enableSimulatedClock(nh);
  }

  // Process the rosbag
  if (!rosbag_processor.processAll()) {
    return -1;
  }

  wavemap_server.visualizeMap();

  return 0;
}
