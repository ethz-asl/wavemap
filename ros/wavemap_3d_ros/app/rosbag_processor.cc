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
  std::string pointcloud_topic = "/pointcloud";
  nh_private.param("pointcloud_topic", pointcloud_topic, pointcloud_topic);
  std::string rosbag_paths_str;
  nh_private.param("rosbag_path", rosbag_paths_str, rosbag_paths_str);
  std::string input_pointcloud_republishing_topic;
  nh_private.param("input_pointcloud_republishing_topic",
                   input_pointcloud_republishing_topic,
                   input_pointcloud_republishing_topic);

  // Create the rosbag processor and load the rosbags
  wavemap::RosbagProcessor rosbag_processor;
  std::istringstream rosbag_paths_ss(rosbag_paths_str);
  if (!rosbag_processor.addRosbags(rosbag_paths_ss)) {
    return -1;
  }

  // Setup the pointcloud callback
  if (input_pointcloud_republishing_topic.empty()) {
    rosbag_processor.addCallback(pointcloud_topic,
                                 &wavemap::Wavemap3DServer::pointcloudCallback,
                                 &wavemap_server);
  } else {
    ros::Publisher pointcloud_pub = nh.advertise<sensor_msgs::PointCloud2>(
        input_pointcloud_republishing_topic, 1);
    rosbag_processor.addCallback<sensor_msgs::PointCloud2>(
        pointcloud_topic,
        [&wavemap_server, pointcloud_pub](auto pointcloud_msg) {
          // Process the point cloud
          wavemap_server.pointcloudCallback(pointcloud_msg);
          // Fix its frame_id s.t. it works with TFs (incl. in Rviz) and publish
          pointcloud_msg.header.frame_id =
              wavemap::TfTransformer::sanitizeFrameId(
                  pointcloud_msg.header.frame_id);
          pointcloud_pub.publish(pointcloud_msg);
        });
  }

  // Republish TFs
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
