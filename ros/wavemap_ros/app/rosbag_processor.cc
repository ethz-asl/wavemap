#include "wavemap_ros/utils/rosbag_processor.h"

#include <rosgraph_msgs/Clock.h>
#include <tf/tfMessage.h>
#include <wavemap_ros_conversions/config_conversions.h>

#include "wavemap_ros/inputs/depth_image_input.h"
#include "wavemap_ros/inputs/pointcloud_input.h"
#include "wavemap_ros/wavemap_server.h"

using namespace wavemap;  // NOLINT
int main(int argc, char** argv) {
  ros::init(argc, argv, "wavemap_rosbag_processor");

  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, false);
  google::InstallFailureSignalHandler();
  FLAGS_alsologtostderr = true;
  FLAGS_colorlogtostderr = true;

  // Setup the wavemap server node
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");
  WavemapServer wavemap_server(nh, nh_private);

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
  const param::Array integrator_params_array =
      param::convert::toParamArray(nh_private, "inputs");
  for (const auto& integrator_params : integrator_params_array) {
    InputBase* input_handler =
        wavemap_server.addInput(integrator_params, nh, nh_private);
    if (input_handler) {
      switch (input_handler->getType().toTypeId()) {
        case InputType::kPointcloud: {
          auto pointcloud_handler =
              dynamic_cast<PointcloudInput*>(input_handler);
          PointcloudInput::registerCallback(
              pointcloud_handler->getTopicType(), [&](auto callback_ptr) {
                rosbag_processor.addCallback(input_handler->getTopicName(),
                                             callback_ptr, pointcloud_handler);
              });
        }
          continue;
        case InputType::kDepthImage:
          rosbag_processor.addCallback<const sensor_msgs::Image&>(
              input_handler->getTopicName(), &DepthImageInput::callback,
              dynamic_cast<DepthImageInput*>(input_handler));
          continue;
      }
    }
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

  wavemap_server.getMap()->prune();
  wavemap_server.runOperations(/*force_run_all*/ true);

  if (nh_private.param("keep_alive", false)) {
    ros::spin();
  }

  return 0;
}
