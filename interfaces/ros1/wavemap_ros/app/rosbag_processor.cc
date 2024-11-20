#include "wavemap_ros/utils/rosbag_processor.h"

#include <string>

#include <glog/logging.h>
#include <rosgraph_msgs/Clock.h>
#include <tf/tfMessage.h>
#include <wavemap/core/utils/profile/resource_monitor.h>
#include <wavemap_ros_conversions/config_conversions.h>

#include "wavemap_ros/inputs/depth_image_topic_input.h"
#include "wavemap_ros/inputs/pointcloud_topic_input.h"
#include "wavemap_ros/ros_server.h"

using namespace wavemap;  // NOLINT
int main(int argc, char** argv) {
  ros::init(argc, argv, "wavemap_rosbag_processor");

  google::InitGoogleLogging(argv[0]);
  google::InstallFailureSignalHandler();
  FLAGS_alsologtostderr = true;
  FLAGS_colorlogtostderr = true;

  // Setup the wavemap server node
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");
  RosServer wavemap_server(nh, nh_private);

  // Read the required ROS params
  std::string rosbag_paths_str;
  nh_private.param("rosbag_path", rosbag_paths_str, rosbag_paths_str);
  std::string input_pointcloud_republishing_topic;

  // Create the rosbag processor and load the rosbags
  RosbagProcessor rosbag_processor;
  std::istringstream rosbag_paths_ss(rosbag_paths_str);
  if (!rosbag_processor.addRosbags(rosbag_paths_ss)) {
    return -1;
  }

  // Setup input handlers
  size_t input_idx = 0u;
  for (const auto& input : wavemap_server.getInputs()) {
    if (auto pointcloud_input =
            dynamic_cast<PointcloudTopicInput*>(input.get());
        pointcloud_input) {
      PointcloudTopicInput::registerCallback(
          pointcloud_input->getTopicType(), [&](auto callback_ptr) {
            rosbag_processor.addCallback(input->getTopicName(), callback_ptr,
                                         pointcloud_input);
          });
    } else if (auto depth_image_input =
                   dynamic_cast<DepthImageTopicInput*>(input.get());
               depth_image_input) {
      rosbag_processor.addCallback<const sensor_msgs::Image&>(
          input->getTopicName(), &DepthImageTopicInput::callback,
          depth_image_input);
    } else {
      ROS_WARN_STREAM(
          "Failed to register callback for input number "
          << input_idx << ", with topic \"" << input->getTopicName()
          << "\". Support for inputs of type \"" << input->getType().toStr()
          << "\" is not yet implemented in the rosbag processing script.");
    }
    ++input_idx;
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

  // Start measuring resource usage
  ResourceMonitor resource_monitor;
  resource_monitor.start();

  // Process the rosbag
  if (!rosbag_processor.processAll()) {
    return -1;
  }

  // Finish processing the map
  wavemap_server.getPipeline().runOperations(/*force_run_all*/ true);
  wavemap_server.getMap()->prune();

  // Report the resource usage
  resource_monitor.stop();
  LOG(INFO) << "Processing complete.\nResource usage:\n"
            << resource_monitor.getLastEpisodeResourceUsageStats()
            << "\n* Map size: "
            << wavemap_server.getMap()->getMemoryUsage() / 1024 << " kB\n";

  if (nh_private.param("keep_alive", false)) {
    ros::spin();
  }

  return 0;
}
